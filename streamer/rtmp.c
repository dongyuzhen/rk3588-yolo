#include <libavformat/avformat.h>
#include <libavutil/mathematics.h>
#include <libavutil/time.h>
#include <libavcodec/avcodec.h>
#include <string.h>
#include <stdlib.h>
#include "rtmp.h"
#include <unistd.h>  // 用于sleep函数
                
static AVPacket pkt;
static AVFormatContext *ofmt_ctx = NULL;
static int frame_index=0;

// 声明来自streamer.c的全局变量，用于控制RTMP推流开关
extern int g_enable_rtmp;

int write_frame(uint8_t*data,int size)
{
        // 如果RTMP被禁用，直接返回成功，不推流
        // 这样可以保留编码功能但跳过网络传输
        if (!g_enable_rtmp) {
            return 0;
        }

        static int64_t pts = 0;
        static int64_t dts = 0;
        static const int64_t frame_duration = 1000/30;  // 30fps对应的帧间隔

        // 限频日志：每 30 帧打印一次
        static int rtmp_log_counter = 0;
        if ((rtmp_log_counter++ % 30) == 0) {
                printf("RTMP发送帧: 大小=%d bytes, PTS=%ld, DTS=%ld\n", 
                       size, pts, dts);
        }

        pkt.size = size;
        pkt.data = data;
        pkt.flags = 0x01;
        pkt.stream_index = 0;
        pkt.pts = pts;
        pkt.dts = dts;
        pkt.duration = frame_duration;

        pts += frame_duration;
        dts += frame_duration;

        if (av_write_frame(ofmt_ctx, &pkt) < 0) {
                printf("Error muxing packet\n");
                return -1;
        }
        return 0;
}

/**
 * 初始化RTMP流媒体推送器
 * @param stream RTMP服务器地址
 * @param config RTMPStreamerConfig结构体，包含SPS/PPS数据和其他参数
 * @return 成功返回0，失败返回-1
 */
int init_rtmp_streamer(char* stream, RtmpContext *config)
{       
        printf("开始初始化RTMP流媒体推送...\n");
        printf("RTMP地址: %s\n", stream);
        printf("SPS/PPS数据大小: %d bytes\n", config->extradata_size);

        int ret;
        if((ret = avformat_network_init()) < 0)
        {
                fprintf(stderr, "avformat_network_init failed!");
                return -1;
        }

        printf("网络初始化成功\n");
        
        // 创建输出格式上下文，使用FLV格式（RTMP通常使用FLV封装）
        avformat_alloc_output_context2(&ofmt_ctx,NULL,"flv",stream);
        if(!ofmt_ctx)
        {
                fprintf(stderr, "Could not create output context\n");
                return -1;
        }

        printf("创建输出上下文成功\n");

        // 创建新的输出流
        AVStream *out_stream = avformat_new_stream(ofmt_ctx,NULL);
        if(! out_stream)
        {
                printf("Failed allocating output stream!\n");
                goto end;
        }

        out_stream->time_base = av_make_q(1, config->fps);

        // 创建编码器上下文
        AVCodecContext *o_codec_ctx = avcodec_alloc_context3(NULL);
        if (!o_codec_ctx) {
                printf("Failed to allocate codec context\n");
                goto end;
        }

        // 配置H.264编码器参数
        o_codec_ctx->codec_id = config->codec_id;
        o_codec_ctx->codec_type = AVMEDIA_TYPE_VIDEO;
        o_codec_ctx->codec_tag = 0;
        o_codec_ctx->pix_fmt = config->pix_fmt;
        o_codec_ctx->width = config->width;
        o_codec_ctx->height = config->height;
        o_codec_ctx->time_base = av_make_q(1, config->fps);
        o_codec_ctx->framerate = av_make_q(config->fps, 1);
        o_codec_ctx->gop_size = config->fps * 2;
        o_codec_ctx->max_b_frames = config->max_b_frames;
        o_codec_ctx->profile = config->profile;
        o_codec_ctx->level = config->level;
        o_codec_ctx->extradata = config->extradata;
        o_codec_ctx->extradata_size = config->extradata_size;

        printf("  输出格式: %s\n", ofmt_ctx->oformat->name);
        printf("  帧率: %d/%d\n", o_codec_ctx->framerate.num, o_codec_ctx->framerate.den);
        printf("设置编码器参数: 分辨率 %dx%d\n", o_codec_ctx->width, o_codec_ctx->height);
        
        if (!out_stream->codecpar) {
                printf("Error: out_stream->codecpar is NULL\n");
                goto end;
        }

        printf("开始复制编码器参数到流中...\n");
        ret = avcodec_parameters_from_context(out_stream->codecpar, o_codec_ctx);
        if (ret < 0) {
                printf("Failed to copy codec parameters from encoder context.\n");
                goto end;
        }
        else
        {
                printf("ok to copy codec parameters from encoder context.\n");
        }
        out_stream->codecpar->codec_tag = 0; // FLV编码器要求codec_tag为0

        printf("打开RTMP URL %s\n", stream);
        if (!(ofmt_ctx->oformat->flags & AVFMT_NOFILE)) {
                int retry_count = 0;
                while (1) {
                        ret = avio_open(&ofmt_ctx->pb, stream, AVIO_FLAG_WRITE);
                        if (ret >= 0) {
                                break;
                        }
                        retry_count++;
                        printf("无法连接到RTMP服务器 '%s'，5秒后重试... (第%d次尝试)\n", stream, retry_count);
                        sleep(5);
                }
        }
        printf("打开输出URL成功\n");

        ret = avformat_write_header(ofmt_ctx, NULL);
        if (ret < 0) {
                printf( "Error occurred when opening output URL\n");
                goto end;
        }
        printf("写入文件头成功\n");

        // 重要：将 extradata 置为 NULL，防止 avcodec_free_context 释放它
        // 因为 extradata 是外部传入的指针（sps_header.data），应该由调用者管理
        o_codec_ctx->extradata = NULL;
        o_codec_ctx->extradata_size = 0;
        avcodec_free_context(&o_codec_ctx);

        printf("创建输出流成功\n");
        printf("创建编码器上下文成功\n");
        printf("复制编码器参数到流中成功\n");
        printf("格式信息打印完成\n");
        printf("RTMP流媒体推送初始化完成\n");
        printf("RTMP流媒体推送器关键参数:\n");

        return 0;

        end:
        if (ofmt_ctx && !(ofmt_ctx->oformat->flags & AVFMT_NOFILE))
                avio_close(ofmt_ctx->pb);
        avformat_free_context(ofmt_ctx);
        if (ret < 0 && ret != AVERROR_EOF) {
                printf( "Error occurred.\n");
                printf("初始化失败，开始清理资源...\n");
                return -1;
        }
}
