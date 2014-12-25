#include "libavcodec/avcodec.h"  
#include "libavformat/avformat.h"  
#include "libswscale/swscale.h"  
#include <stdio.h>  
#include "SDL/SDL.h"  
#include "SDL_thread.h"  

#define PICTURE_W 720  
#define PICTURE_H 576  
#define PICTURE_PW 720  
#define PICTURE_PH 576  

#define MAX_QUEUE_SIZE 100  
#define VIDEO_PICTURE_QUEUE_SIZE 2  
#define MAX_AUDIO_FRAME_SIZE 192000

typedef struct VideoPicture_t{  
	SDL_Overlay *bmp;  
	double pts;  
	double duration;  
	int width,height;  
}VideoPicture;  


struct play_info_t{  
	double audio_clock;//½âʱÖ  
	double audio_current_pts;//µ±ǰÒƵµÄts  
	double vedio_clock;//½âʱÖ  
	double last_frame_pts;//Éһ֡µÄTSֵ  
	double last_frame_delay;//Éһ֡µÄÓ±  
	double frame_timer;  

	AVCodecContext *pVideoCodecCtx;  
	AVCodecContext *pAudioCodecCtx;  

	VideoPicture pictq[VIDEO_PICTURE_QUEUE_SIZE];  
	int pictq_size, pictq_rindex, pictq_windex;  
	SDL_mutex *pictq_mutex;  
	SDL_cond *pictq_cond;  

	struct SwrContext *audio_conv;  
	struct SwsContext *video_conv;  
};  

struct play_info_t g_play_info;  




typedef struct PacketQueue {  
	AVPacketList *first_pkt, *last_pkt;  
	int nb_packets;  
	int size;  
	SDL_mutex *mutex;  
	SDL_cond *cond;  
} PacketQueue;  

PacketQueue audio_queue;  
PacketQueue video_queue;  
int quit = 0;  
SDL_Surface *screen;  
SDL_Overlay *bmp;  
SDL_Rect rect;  

int global_video_pkt_pts=0;  
int our_get_buffer(struct AVCodecContext *c, AVFrame *pic) {  
	int ret = avcodec_default_get_buffer(c, pic);  
	uint64_t *pts = av_malloc(sizeof(uint64_t));  
	*pts = global_video_pkt_pts;  
	pic->opaque = pts;  
	return ret;  
}  
void our_release_buffer(struct AVCodecContext *c, AVFrame *pic) {  
	if(pic) av_freep(&pic->opaque);  
	avcodec_default_release_buffer(c, pic);  
}  

double get_audio_clock(void)  
{  
	return g_play_info.audio_current_pts;  
}  

double get_video_clock(void)  
{  
	return g_play_info.vedio_clock;  
}  

void picture_queque_init(struct play_info_t *play)  
{  
	int i;  
	play->pictq_cond=SDL_CreateCond();  
	play->pictq_mutex=SDL_CreateMutex();  
	play->pictq_rindex=play->pictq_windex=0;  
	play->pictq_size=0;  
	memset(play->pictq,0,sizeof(play->pictq));  
	for(i=0;i<VIDEO_PICTURE_QUEUE_SIZE;i++){  
		play->pictq[i].bmp=SDL_CreateYUVOverlay(PICTURE_W,PICTURE_H,SDL_YUY2_OVERLAY,screen);  
	}  
}  

void picture_queque_destroy(struct play_info_t *play)  
{  
	int i;  
	for(i=0;i<VIDEO_PICTURE_QUEUE_SIZE;i++){  
		SDL_FreeYUVOverlay(play->pictq[i].bmp);  
	}  
}  



void packet_queue_init(PacketQueue *q) {  
	memset(q, 0, sizeof(PacketQueue));  
	q->mutex = SDL_CreateMutex();  
	q->cond = SDL_CreateCond();  
}  

int packet_queue_put(PacketQueue *q, AVPacket *pkt) {  
	AVPacketList *pkt1;  
	if(av_dup_packet(pkt) < 0) {  
		return -1;  
	}  
	pkt1 = av_malloc(sizeof(AVPacketList));  
	if (!pkt1)  
		return -1;  
	pkt1->pkt = *pkt;  
	pkt1->next = NULL;  
	SDL_LockMutex(q->mutex);  
	if (!q->last_pkt)  
		q->first_pkt = pkt1;  
	else  
		q->last_pkt->next = pkt1;  
	q->last_pkt = pkt1;  
	q->nb_packets++;  
	q->size += pkt1->pkt.size;  
	SDL_CondSignal(q->cond);  
	SDL_UnlockMutex(q->mutex);  
	return 0;  
}  


static int packet_queue_get(PacketQueue *q, AVPacket *pkt, int block) {  
	AVPacketList *pkt1;  
	int ret;  
	SDL_LockMutex(q->mutex);  
	for(;;) {  
		if(quit) {  
			ret = -1;  
			break;  
		}  
		pkt1 = q->first_pkt;  
		if (pkt1) {  
			q->first_pkt = pkt1->next;  
			if (!q->first_pkt)  
				q->last_pkt = NULL;  
			q->nb_packets--;  
			q->size -= pkt1->pkt.size;  
			*pkt = pkt1->pkt;  
			av_free(pkt1);  
			ret = 1;  
			break;  
		} else if (!block) {  
			ret = 0;  
			break;  
		} else {  
			SDL_CondWait(q->cond, q->mutex);  
		}  
	}  
	SDL_UnlockMutex(q->mutex);  
	return ret;  
}  

int decode_interrupt_cb(void) {  
	return quit;  
}  

//¼ÆãȷµÄtsֵ  
double sync_video(struct play_info_t *play,AVFrame* frame,double pts)  
{  
	double frame_delay;  
	if(pts!=0)  
		play->vedio_clock=pts;  
	else  
		pts=play->vedio_clock;  
	frame_delay=av_q2d(play->pVideoCodecCtx->time_base);//һ֡ռÓµÄ±¼ä
	frame_delay+=frame->repeat_pict*(frame_delay*0.5);//¼Æã¸´֡  
	play->vedio_clock+=frame_delay;  
	return pts;  
}  

int audio_decode_frame(struct play_info_t* play, uint8_t *audio_buf,int buf_size) {  
	AVCodecContext *aCodecCtx=play->pAudioCodecCtx;  
	AVFrame *pAudioFrame=avcodec_alloc_frame();  
	AVPacket pkt,pkt1;  
	int frame_finished=0;  
	int pkt_pos,pkt_len;  
	int src_len=0,dst_len=0,data_size=0;  
	float pts=0;  
	avcodec_get_frame_defaults(pAudioFrame);  

	uint8_t *out[]={audio_buf};  

	for(;!quit;){  
		if(packet_queue_get(&audio_queue, &pkt, 1) < 0) {  
			av_free(pAudioFrame);  
			return -1;  
		}  
		pkt1=pkt;  
		pkt_pos=0;  
		pkt_len=pkt.size;  

		while(pkt_pos<pkt.size && !quit){  
			if((src_len=avcodec_decode_audio4(aCodecCtx,pAudioFrame,&frame_finished,&pkt1))<0){  
				av_free_packet(&pkt);  
				av_free(pAudioFrame);  
				return -1;  
			}  

			play->audio_clock+=(double)(pAudioFrame->linesize[0])/  
				(aCodecCtx->channels*aCodecCtx->sample_rate*av_get_bytes_per_sample(aCodecCtx->sample_fmt));  

			if(frame_finished){  
				int len=swr_convert(play->audio_conv,out,buf_size/aCodecCtx->channels/av_get_bytes_per_sample(AV_SAMPLE_FMT_S16),  
						pAudioFrame->data,pAudioFrame->linesize[0]/aCodecCtx->channels/av_get_bytes_per_sample(pAudioFrame->format));  
				fprintf(stdout, "len = %d , frame=%d\n", len, pAudioFrame->linesize[0]);
				printf("%d %d\n", buf_size/aCodecCtx->channels/av_get_bytes_per_sample(AV_SAMPLE_FMT_S16),pAudioFrame->linesize[0]/aCodecCtx->channels/av_get_bytes_per_sample(pAudioFrame->format));
				len=len*aCodecCtx->channels*av_get_bytes_per_sample(AV_SAMPLE_FMT_S16);  
				av_free(pAudioFrame);  
				av_free_packet(&pkt);  
				return len;  
			}else{  
				if (!pkt1.data && aCodecCtx->codec->capabilities & CODEC_CAP_DELAY){  
					break;  
				}  
			}  
			pkt_pos+=src_len;//Ò¾­½âµĳ¤¶È 
			pkt1.data=pkt.data+pkt_pos;  
			pkt1.size=pkt.size-pkt_pos;  
		}  
		av_free_packet(&pkt);  
	}  
	av_free(pAudioFrame);  
	return dst_len;  
}  

//decode the audio data ,and copy the result to stream  
void SDLCALL audio_callback(void *userdata, Uint8 *stream, int len)  
{  
	struct play_info_t *play=(struct play_info_t*)userdata;  
	AVCodecContext *aCodecCtx = play->pAudioCodecCtx;  
	int len1, audio_size;  
	static uint8_t audio_buf[(MAX_AUDIO_FRAME_SIZE * 3) / 2];  
	static unsigned int audio_buf_size = 0;  
	static unsigned int audio_buf_index = 0;  
	int bytes_per_sec=2*aCodecCtx->channels*aCodecCtx->sample_rate;  

	while(len > 0 && !quit) {  
		if(audio_buf_index >= audio_buf_size) {  
			audio_size = audio_decode_frame(play, audio_buf,sizeof(audio_buf));  
			if(audio_size < 0) {  
				audio_buf_size = 1024;  
				memset(audio_buf, 0, audio_buf_size);  
			} else {  
				audio_buf_size = audio_size;  
			}  
			audio_buf_index = 0;  
		}  
		len1 = audio_buf_size - audio_buf_index;  

		if(len1 > len)  
			len1 = len;  
		memcpy(stream, (uint8_t *)audio_buf + audio_buf_index, len1);  
		len -= len1;  
		stream += len1;  
		audio_buf_index += len1;  
	}  
	if(audio_buf_size > audio_buf_index){  
		play->audio_current_pts=play->audio_clock-((double)(audio_buf_size-audio_buf_index)/(double)bytes_per_sec);  
		if(play->audio_current_pts<0)  
			play->audio_current_pts=play->audio_clock;  
		//printf("audio-pts:%f\n",play->audio_current_pts);  
	}  
}  

int SDLCALL video_decode_callback(void* arg)  
{  
	printf("---in video decode thread ---\n");  
	struct play_info_t* play=(struct play_info_t*)arg;  
	AVCodecContext *pVideoCodecCtx=play->pVideoCodecCtx;  
	double pts;  
	VideoPicture *vp;  
	AVPacket pkt;  
	int frame_finished;  
	AVPicture pict = { { 0 } };  
	AVFrame *pVideoFrame=avcodec_alloc_frame();  
	if(pVideoFrame==NULL){  
		printf("can't alloc video frame!\n");  
		return -1;  
	}  

	while(!quit){  
		if(packet_queue_get(&video_queue, &pkt, 1) < 0) {  
			return -1;  
		}  
		pts = 0;  
		// Save global pts to be stored in pFrame in first call  
		global_video_pkt_pts = pkt.pts;  
		avcodec_decode_video2(pVideoCodecCtx,pVideoFrame,&frame_finished,&pkt);  
		if(pkt.dts == AV_NOPTS_VALUE && pVideoFrame->opaque && *(uint64_t*)pVideoFrame->opaque != AV_NOPTS_VALUE){  
			pts = *(uint64_t *)pVideoFrame->opaque;  
		} else if(pkt.dts!=AV_NOPTS_VALUE){  
			pts=pkt.dts;  
		}else{  
			pts=0;  
		}  
		pts*=av_q2d(pVideoCodecCtx->time_base);  

		av_free_packet(&pkt);  
		if(frame_finished){  
			SDL_LockMutex(play->pictq_mutex);  
			while(play->pictq_size >= VIDEO_PICTURE_QUEUE_SIZE && !quit){  
				SDL_CondWait(play->pictq_cond,play->pictq_mutex);  
			}  
			SDL_UnlockMutex(play->pictq_mutex);  
			if(quit)  
				goto out;  

			vp=&play->pictq[play->pictq_windex];  
			SDL_Overlay *bmp=vp->bmp;  
			//ÐÁ»¯֡  
			pts=sync_video(&g_play_info,pVideoFrame,pts);  
			vp->pts=pts;  
			//printf("video-pts:%f\n",pts);  

			SDL_LockYUVOverlay(bmp);  
			pict.data[0]=bmp->pixels[0];  
			pict.data[1]=bmp->pixels[2];  
			pict.data[2]=bmp->pixels[1];  
			pict.linesize[0]=bmp->pitches[0];  
			pict.linesize[1]=bmp->pitches[2];  
			pict.linesize[2]=bmp->pitches[1];  
			sws_scale(play->video_conv,pVideoFrame->data,pVideoFrame->linesize,0,  
					pVideoFrame->height,pict.data,pict.linesize);  
			SDL_UnlockYUVOverlay(bmp);  


			play->pictq_windex=(play->pictq_windex+1)%VIDEO_PICTURE_QUEUE_SIZE;  
			SDL_LockMutex(play->pictq_mutex);  
			play->pictq_size++;  
			SDL_UnlockMutex(play->pictq_mutex);  

		}  
	}  
out:  
	av_free(pVideoFrame);  
	printf("---- exit video_decode_callback ----\n");  
	return 0;  
}  

void video_image_display(struct play_info_t *play)  
{  
	double delay,time;  
	VideoPicture *vp;  
	double diff,sync_threshold;  
	if(play->pictq_size>0){  
		vp=&play->pictq[play->pictq_rindex];  
		delay=vp->pts - play->last_frame_pts;  
		if(delay>0 && delay<10){  
			play->last_frame_delay=delay;  
		}  
		delay=play->last_frame_delay;  

		diff=vp->pts-get_audio_clock();  
		sync_threshold = FFMAX(0.01, delay);  
		if (fabs(diff) < 10) {  
			if (diff <= -sync_threshold)  
				delay = 0;  
			else if (diff >= sync_threshold)  
				delay = 2 * delay;  
		}  

		time=av_gettime()/1000000.0;  
		if(time < play->frame_timer+delay)  
			return ;  

		if(delay>0)  
			play->frame_timer+=delay;  
		play->last_frame_pts=vp->pts;  

		SDL_DisplayYUVOverlay(vp->bmp,&rect);  

		play->pictq_rindex=(play->pictq_rindex+1)%VIDEO_PICTURE_QUEUE_SIZE;  

		SDL_LockMutex(play->pictq_mutex);  
		play->pictq_size--;  
		SDL_CondSignal(play->pictq_cond);  
		SDL_UnlockMutex(play->pictq_mutex);  

	}  
}  

int SDLCALL video_show_callback(void* arg)  
{  
	double delay=5000;  
	struct play_info_t *play=(struct play_info_t*)arg;  
	while(!quit){  
		video_image_display(play);  
		usleep(delay);  
	}  
	printf("----exit video_show_callback ----\n");  
	return 0;  
}  

void player_log_callback(void* ptr, int level, const char* fmt, va_list vl)  
{  
	static int print_prefix = 0;  
	static int count;  
	char line[1024];  
	av_log_format_line(ptr, level, fmt, vl, line, sizeof(line), &print_prefix);  
	printf("%s",line);  
}  




int main(int argc,char *argv[])  
{  
	if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER)<0){  
		printf("Init SDL err!\n");  
		return -1;  
	}  

	av_log_set_callback(player_log_callback);  
	av_register_all();  

	AVFormatContext *pFormatCtx;  
	AVDictionary *optionsDict = NULL;
	pFormatCtx=avformat_alloc_context();  
	if(avformat_open_input(&pFormatCtx,argv[1],NULL,NULL)<0){  
		printf("avformat_open_input err!\n");  
		return -1;  
	}  

	if(avformat_find_stream_info(pFormatCtx,NULL)<0){  
		printf("avformat_find_stream_info err!\n");  
		return -1;  
	}  

	//av_dump_format(pFormatCtx,0,argv[1],0);  

	int video_stream=-1,audio_stream=-1,i;  
	for(i=0;i<pFormatCtx->nb_streams;i++){  
		if(pFormatCtx->streams[i]->codec->codec_type==AVMEDIA_TYPE_VIDEO){  
			video_stream=i;  
		}  
		if(pFormatCtx->streams[i]->codec->codec_type==AVMEDIA_TYPE_AUDIO){  
			audio_stream=i;  
		}  
	}  
	if(video_stream==-1 || audio_stream==-1){  
		printf("not find video or audio stream!\n");  
		avformat_close_input(&pFormatCtx);  
		return -1;  
	}  
	g_play_info.pVideoCodecCtx=pFormatCtx->streams[video_stream]->codec;  
	g_play_info.pAudioCodecCtx=pFormatCtx->streams[audio_stream]->codec;  

	//find the decoder  
	AVCodec *pVideoCodec=avcodec_find_decoder(g_play_info.pVideoCodecCtx->codec_id);  
	if(pVideoCodec==NULL){  
		printf("not find video decoder!\n");  
		avformat_close_input(&pFormatCtx);  
		return -1;  
	}  
	AVCodec *pAudioCodec=avcodec_find_decoder(g_play_info.pAudioCodecCtx->codec_id);  
	if(pVideoCodec==NULL){  
		printf("not find audio decoder!\n");  
		avformat_close_input(&pFormatCtx);  
		return -1;  
	}  
	//open the codec  
	if(avcodec_open2(g_play_info.pVideoCodecCtx,pVideoCodec, optionsDict)<0){  
		printf("can't open the video decoder!\n");  
		avformat_close_input(&pFormatCtx);  
		return -1;  
	}  
	if(avcodec_open2(g_play_info.pAudioCodecCtx,pAudioCodec, optionsDict)<0){  
		printf("can't open the audio decoder!\n");  
		avcodec_close(g_play_info.pVideoCodecCtx);  
		avformat_close_input(&pFormatCtx);  
		return -1;  
	}  

	//setup SDL  
	screen=SDL_SetVideoMode(PICTURE_W,PICTURE_H,0,0);  
	picture_queque_init(&g_play_info);  
	packet_queue_init(&video_queue);  
	packet_queue_init(&audio_queue);  

	rect.x=0;  
	rect.y=0;  
	rect.w=PICTURE_W;  
	rect.h=PICTURE_H;  


	//setup the sdl audio  
	SDL_AudioSpec sdl_audio;  
	int64_t wanted_channel_layout = 0;  
	int wanted_nb_channels;  
	wanted_channel_layout =   
		(g_play_info.pAudioCodecCtx->channel_layout &&   
		 g_play_info.pAudioCodecCtx->channels == av_get_channel_layout_nb_channels(g_play_info.pAudioCodecCtx->channel_layout)) ? g_play_info.pAudioCodecCtx->channel_layout : av_get_default_channel_layout(g_play_info.pAudioCodecCtx->channels);  
	wanted_channel_layout &= ~AV_CH_LAYOUT_STEREO_DOWNMIX;  
	wanted_nb_channels = av_get_channel_layout_nb_channels(wanted_channel_layout);  
	/* SDL only supports 1, 2, 4 or 6 channels at the moment, so we have to make sure not to request anything else. */  
	while (wanted_nb_channels > 0 && (wanted_nb_channels == 3 || wanted_nb_channels == 5 || wanted_nb_channels > 6)) {  
		wanted_nb_channels--;  
		wanted_channel_layout = av_get_default_channel_layout(wanted_nb_channels);  
	}  
	sdl_audio.channels=av_get_channel_layout_nb_channels(wanted_channel_layout);  
	g_play_info.pAudioCodecCtx->channels=sdl_audio.channels;  
	g_play_info.pAudioCodecCtx->channel_layout=wanted_channel_layout;  

	g_play_info.audio_conv=swr_alloc_set_opts(NULL,wanted_channel_layout,AV_SAMPLE_FMT_S16,g_play_info.pAudioCodecCtx->sample_rate,  
			wanted_channel_layout,g_play_info.pAudioCodecCtx->sample_fmt,  
			g_play_info.pAudioCodecCtx->sample_rate,0,NULL);  
	swr_init(g_play_info.audio_conv);  

	sdl_audio.freq=g_play_info.pAudioCodecCtx->sample_rate;  
	sdl_audio.format=AUDIO_S16SYS;  
	sdl_audio.channels=g_play_info.pAudioCodecCtx->channels;  
	sdl_audio.silence=0;  
	sdl_audio.samples=1024;  
	sdl_audio.callback=audio_callback;//calls when the audio device need more data  
	sdl_audio.userdata=&g_play_info;  
	if(SDL_OpenAudio(&sdl_audio,NULL)<0){  
		printf("can't open audio device!\n");  
		goto err_alloc_frame1;  
	}  

	g_play_info.vedio_clock=0;  
	g_play_info.audio_clock=0;  
	g_play_info.audio_current_pts=0;  
	g_play_info.last_frame_pts=0;  
	g_play_info.last_frame_delay=0;  
	g_play_info.frame_timer=av_gettime()/1000000.0;  
	g_play_info.pVideoCodecCtx->get_buffer=our_get_buffer;  
	g_play_info.pVideoCodecCtx->release_buffer=our_release_buffer;  

	SDL_PauseAudio(0);  

	//create video decode thread  
	//setup the converter  

	g_play_info.video_conv=sws_getContext(g_play_info.pVideoCodecCtx->width,g_play_info.pVideoCodecCtx->height,  
			g_play_info.pVideoCodecCtx->pix_fmt,  
			PICTURE_PW,PICTURE_PH,PIX_FMT_YUYV422,SWS_POINT,NULL,NULL,NULL);  
	SDL_Thread * video_decode_thread=SDL_CreateThread(video_decode_callback,(void*)&g_play_info);  
	SDL_Thread * video_show_thread=SDL_CreateThread(video_show_callback,(void*)&g_play_info);  
	//start to decode  
	SDL_Event sdl_event;  
	AVPacket packet;  



	while(!quit){  
		SDL_PollEvent(&sdl_event);  
		if(sdl_event.type==SDL_QUIT){  
			SDL_CondSignal(video_queue.cond);  
			SDL_CondSignal(audio_queue.cond);  
			quit=1;  
			break;  
		}  
		if(video_queue.nb_packets>=MAX_QUEUE_SIZE || audio_queue.nb_packets>=MAX_QUEUE_SIZE){  
			//printf("%d---%d\n",video_queue.nb_packets,audio_queue.nb_packets);  
			usleep(10000);  
			continue;  
		}  
		if(av_read_frame(pFormatCtx,&packet)<0){  
			quit=1;  
			continue;  
		}  
		if(packet.stream_index==video_stream){  
			packet_queue_put(&video_queue,&packet);  
		}else if(packet.stream_index==audio_stream){  
			packet_queue_put(&audio_queue,&packet);  
		}else{  
			av_free_packet(&packet);  
		}  

		//usleep(5000);  
	}  
	SDL_CloseAudio();  
	SDL_CondSignal(g_play_info.pictq_cond);  
	SDL_WaitThread(video_decode_thread,NULL);  
	SDL_WaitThread(video_show_thread,NULL);  
	picture_queque_destroy(&g_play_info);  

	swr_free(&g_play_info.audio_conv);  
	sws_freeContext(g_play_info.video_conv);  

	SDL_FreeSurface(screen);  
	avcodec_close(g_play_info.pVideoCodecCtx);  
	avcodec_close(g_play_info.pAudioCodecCtx);  
	avformat_close_input(&pFormatCtx);  
	return 0;  

err_alloc_frame1:  
	avcodec_close(g_play_info.pAudioCodecCtx);  
	avcodec_close(g_play_info.pVideoCodecCtx);  
	avformat_close_input(&pFormatCtx);  
	return -1;  
}
