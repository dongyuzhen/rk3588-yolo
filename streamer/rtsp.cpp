#include "rtsp.h"
#include "../SafeQueue.h"
#include <iostream>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/time.h>
}

RtspStreamer::RtspStreamer(const std::string& url, int width, int height, int fps, SafeQueue<EncodedPacket>& queue, uint8_t* extradata, int extradata_size)
    : url_(url), width_(width), height_(height), fps_(fps), packet_queue_(queue), extradata_(extradata), extradata_size_(extradata_size), is_running_(false)
{
    if(initFFmpeg())
    {
        is_running_ = true;
        push_thread_ = std::thread(&RtspStreamer::pushLoop, this);
    }
} 

RtspStreamer::~RtspStreamer(){
    is_running_ = false;
    packet_queue_.stop();
    if(push_thread_.joinable())
    {
        push_thread_.join();
    }
    cleanupFFmpeg();
}

bool RtspStreamer::initFFmpeg()
{
    // 初始化网络组件
    avformat_network_init();

    // 创建输出上下文
    if (avformat_alloc_output_context2(&ofmt_ctx_, nullptr, "rtsp", url_.c_str()) < 0) {
        std::cerr << "[RTSP] Could not create RTSP output context" << std::endl;
        return false;
    }

    // 创建H264视频流
    out_stream_ = avformat_new_stream(ofmt_ctx_, nullptr);
    if (!out_stream_) {
        std::cerr << "[RTSP] Failed allocating output stream" << std::endl;
        return false;
    }

   
    out_stream_->time_base = av_make_q(1, fps_);   // 设置视频流参数
    AVCodecParameters* codecpar = out_stream_->codecpar;
    codecpar->codec_id = AV_CODEC_ID_H264;
    codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    codecpar->width = width_;
    codecpar->height = height_;

    // 填入从 MPP 拿到的 SPS/PPS
    if (extradata_ != nullptr && extradata_size_ > 0) {
        codecpar->extradata_size = extradata_size_;
        codecpar->extradata = (uint8_t*)av_mallocz(extradata_size_ + AV_INPUT_BUFFER_PADDING_SIZE);
        memcpy(codecpar->extradata, extradata_, extradata_size_);
    }

    // 强行指定走 tcp 传输，防止弱网环境下 udp 乱序花屏
    AVDictionary* options = nullptr;
    av_dict_set(&options, "rtsp_transport", "tcp", 0);
    // 增加 3 秒超时时间（单位：微秒），防止服务器没开或者 IP 写错导致一直死等
    av_dict_set(&options, "timeout", "3000000", 0);
    
    // 如果要手动打开
    if (!(ofmt_ctx_->oformat->flags & AVFMT_NOFILE)) {
        //打开网络流
        if (avio_open2(&ofmt_ctx_->pb, url_.c_str(), AVIO_FLAG_WRITE, nullptr, &options) < 0) {
            std::cerr << "[RTSP] Could not open RTSP URL" << std::endl;
            av_dict_free(&options);
            return false;
        }
    }
    // 写入头信息
    if (avformat_write_header(ofmt_ctx_, &options) < 0) {
        std::cerr << "[RTSP] Error occurred when opening RTSP URL" << std::endl;
        av_dict_free(&options);
        return false;
    }

    // 释放字典
    av_dict_free(&options);
    return true;
    
}

void RtspStreamer::pushLoop(){
    AVPacket* pkt = av_packet_alloc();

    while(is_running_)
    {
        EncodedPacket enc_pkt;
        //线程阻塞在这里等待外层的 SafeQueue 给它喂数据
        if(!packet_queue_.dequeue(enc_pkt))
        {
            //运行停止，且dequeue flase（空了）
            if(!is_running_)
                break;
            continue;
        }

        //填充数据
        pkt->data  =enc_pkt.data.get();
        pkt->size = enc_pkt.size;
        pkt->pts = enc_pkt.pts;
        pkt->dts = enc_pkt.dts;
        pkt->stream_index = out_stream_->index;

        int ret = av_interleaved_write_frame(ofmt_ctx_, pkt);

        if(ret < 0)
        {
            std::cerr << "[RTSP] Network error, write failed. Attempting to reconnect..." << std::endl;
            cleanupFFmpeg();
            //断线了？尝试重连
            while(is_running_)
            {
                std::this_thread::sleep_for(std::chrono::seconds(2));
                if(initFFmpeg())
                {
                    std::cout << "[RTSP] Reconnected successfully!" << std::endl;
                    break;
                }
            }   
            av_packet_unref(pkt);
            continue;
        }
        av_packet_unref(pkt);
    }
    av_packet_free(&pkt);
}



void RtspStreamer::cleanupFFmpeg() {
    if (ofmt_ctx_) {
        av_write_trailer(ofmt_ctx_);
        if (!(ofmt_ctx_->oformat->flags & AVFMT_NOFILE)) {
            avio_closep(&ofmt_ctx_->pb);
        }
        avformat_free_context(ofmt_ctx_);
        ofmt_ctx_ = nullptr;
    }
}