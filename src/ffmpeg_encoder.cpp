/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "ffmpeg_image_transport/ffmpeg_encoder.h"
#include "ffmpeg_image_transport_msgs/FFMPEGPacket.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <iomanip>

namespace ffmpeg_image_transport
{
char av_error[AV_ERROR_MAX_STRING_SIZE] = { 0 };
#define av_err2str(errnum) av_make_error_string(av_error, AV_ERROR_MAX_STRING_SIZE, errnum)

FFMPEGEncoder::FFMPEGEncoder()
{
}

FFMPEGEncoder::~FFMPEGEncoder()
{
  reset();
}

void FFMPEGEncoder::reset()
{
  Lock lock(mutex_);
  closeCodec();
}

void FFMPEGEncoder::closeCodec()
{
  if (pkt_)
  {
    if (pkt_->data)
    {
      av_packet_unref(pkt_);  // free packet allocated by encoder
    }
    av_packet_free(&pkt_);
  }

  if (frame_)
  {
    av_freep(&frame_->data[0]);
    av_frame_free(&frame_);
  }

  if (codecContext_)
    avcodec_free_context(&codecContext_);
}

bool FFMPEGEncoder::initialize(int width, int height, Callback callback)
{
  Lock lock(mutex_);
  callback_ = callback;
  return (openCodec(width, height));
}

bool FFMPEGEncoder::checkImageSize(int witdh, int height) const
{
  Lock lock(mutex_);
  if (this->width == witdh && this->height == height)
    return true;
  else
    return false;
}

bool FFMPEGEncoder::openCodec(int width, int height)
{
  int ret       = -1;
  codecContext_ = NULL;
  try
  {
    if (codecName_.empty())
    {
      throw(std::runtime_error("no codec set!"));
    }
    if ((width % 32) != 0)
    {
      throw(
          std::runtime_error("image line width must be "
                             "multiple of 32 but is: " +
                             std::to_string(width)));
    }
    if (codecName_ == "libx264")
      av_log_set_level(AV_LOG_FATAL);
    else
      av_log_set_level(AV_LOG_INFO);

    // find codec
    AVCodec* codec = avcodec_find_encoder_by_name(codecName_.c_str());
    if (!codec)
    {
      throw(std::runtime_error("cannot find codec: " + codecName_));
    }

    // allocate codec context
    codecContext_ = avcodec_alloc_context3(codec);
    if (!codecContext_)
    {
      throw(std::runtime_error("cannot allocate codec context!"));
    }
    if (bitRate_ != -1)
      codecContext_->bit_rate = bitRate_ * 1000;
    if (qmax_ != -1)
      codecContext_->qmax = qmax_;  // 0: highest, 63: worst quality bound
    codecContext_->width     = width;
    codecContext_->height    = height;
    codecContext_->time_base = timeBase_;
    codecContext_->framerate = frameRate_;

    // gop size is number of frames between keyframes
    // small gop -> higher bandwidth, lower cpu consumption
    codecContext_->gop_size = GOPSize_;

    // number of bidirectional frames (per group?).
    // NVenc can only handle zero!
    codecContext_->max_b_frames = 0;

    // encoded pixel format. Must be supported by encoder
    // check with e.g.: ffmpeg -h encoder=h264_nvenc
    codecContext_->pix_fmt = pixFormat_;

    if (av_opt_set(codecContext_->priv_data, "profile", profile_.c_str(), AV_OPT_SEARCH_CHILDREN) != 0)
    {
      ROS_ERROR_STREAM("cannot set profile: " << profile_);
    }

    if (av_opt_set(codecContext_->priv_data, "preset", preset_.c_str(), AV_OPT_SEARCH_CHILDREN) != 0)
    {
      ROS_ERROR_STREAM("cannot set preset: " << preset_ << " " << av_err2str(ret));
    }

    if (zerolatency_)
    {
      if (codecName_ == "libx264")
      {
        if ((ret = av_opt_set(codecContext_->priv_data, "tune", "zerolatency", 0)) != 0)
        {
          ROS_ERROR_STREAM("cannot set tune: zerolatency "
                           << " " << av_err2str(ret));
        }
      }
      else if (codecName_ == "h264_nvenc" || codecName_ == "hevc_nvenc")
      {
        if ((ret = av_opt_set(codecContext_->priv_data, "zerolatency", "1", 0)) != 0)
        {
          ROS_ERROR_STREAM("cannot set zerolatency: " << av_err2str(ret));
        }
        if ((ret = av_opt_set(codecContext_->priv_data, "delay", "0", 0)) != 0)
        {
          ROS_ERROR_STREAM("cannot set delay: " << av_err2str(ret));
        }
      }
      else
      {
        ROS_WARN_STREAM("zerolatency is not supported for codec: " << codecName_);
      }
    }

    // unsigned char *mime_type = NULL;
    // av_opt_get(codecContext_->priv_data, "crf", AV_OPT_SEARCH_CHILDREN, &mime_type);
    // ROS_ERROR_STREAM("crf: " << mime_type);
    if (rc_mode_ == "crf")
    {
      if (av_opt_set(codecContext_->priv_data, "crf", std::to_string(rc_value_).c_str(), AV_OPT_SEARCH_CHILDREN))
        ROS_ERROR_STREAM("cannot set crf: " << rc_value_);
    }
    else if (rc_mode_ == "cqp")
    {
      if (av_opt_set(codecContext_->priv_data, "qp", std::to_string(rc_value_).c_str(), AV_OPT_SEARCH_CHILDREN))
        ROS_ERROR_STREAM("cannot set qp: " << rc_value_);
    }

    ROS_DEBUG_STREAM("cur bit_rate: " << codecContext_->bit_rate);

    ROS_DEBUG(
        "codec: %10s, profile: %10s, preset: %10s,"
        " bit_rate: %10ldK qmax: %2d",
        codecName_.c_str(), profile_.c_str(), preset_.c_str(), bitRate_, qmax_);
    /* other optimization options for nvenc
       if (av_opt_set_int(codecContext_->priv_data, "surfaces",
       0, AV_OPT_SEARCH_CHILDREN) != 0) {
       ROS_ERROR_STREAM("cannot set surfaces!");
       }
    */
    if ((ret = avcodec_open2(codecContext_, codec, NULL)) < 0)
    {
      throw(std::runtime_error("cannot open codec! ret: " + std::to_string(ret)));
    }
    ROS_DEBUG_STREAM("opened codec: " << codecName_);
    frame_ = av_frame_alloc();
    if (!frame_)
    {
      throw(std::runtime_error("cannot alloc frame!"));
    }
    frame_->width  = width;
    frame_->height = height;
    frame_->format = codecContext_->pix_fmt;
    // allocate image for frame
    if (av_image_alloc(frame_->data, frame_->linesize, width, height, (AVPixelFormat)frame_->format, 32) < 0)
    {
      throw(std::runtime_error("cannot alloc image!"));
    }
    // Initialize packet
    pkt_ = av_packet_alloc();
  }
  catch (const std::runtime_error& e)
  {
    ROS_ERROR_STREAM(e.what());
    closeCodec();
    return (false);
  }
  ROS_INFO_STREAM("intialized codec " << codecName_ << " for image: " << width << "x" << height);
  this->width  = width;
  this->height = height;
  return (true);
}

void FFMPEGEncoder::encodeImage(const sensor_msgs::Image& msg)
{
  ros::WallTime t0;
  if (measurePerformance_)
  {
    t0 = ros::WallTime::now();
  }
  cv::Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
  encodeImage(img, msg.header, t0);
  if (measurePerformance_)
  {
    const auto t1 = ros::WallTime::now();
    tdiffDebayer_.update((t1 - t0).toSec());
  }
}

void FFMPEGEncoder::encodeImage(const cv::Mat& img, const std_msgs::Header& header, const ros::WallTime& t0)
{
  Lock lock(mutex_);
  ros::WallTime t1, t2, t3;
  if (measurePerformance_)
  {
    frameCnt_++;
    t1 = ros::WallTime::now();
    totalInBytes_ += img.cols * img.rows;  // raw size!
  }

  const uint8_t* p              = img.data;
  const int width               = img.cols;
  const int height              = img.rows;
  const AVPixelFormat targetFmt = codecContext_->pix_fmt;
  if (targetFmt == AV_PIX_FMT_BGR0)
  {
    memcpy(frame_->data[0], p, width * height * 3);
  }
  else if (targetFmt == AV_PIX_FMT_YUV420P)
  {
    cv::Mat yuv;
    cv::cvtColor(img, yuv, cv::COLOR_BGR2YUV_I420);
    const uint8_t* p = yuv.data;
    memcpy(frame_->data[0], p, width * height);
    memcpy(frame_->data[1], p + width * height, width * height / 4);
    memcpy(frame_->data[2], p + width * (height + height / 4), (width * height) / 4);
  }
  else
  {
    ROS_ERROR_STREAM("cannot convert format bgr8 -> " << (int)codecContext_->pix_fmt);
    return;
  }
  if (measurePerformance_)
  {
    t2 = ros::WallTime::now();
    tdiffFrameCopy_.update((t2 - t1).toSec());
  }

  frame_->pts = pts_++;  //
  ptsToStamp_.insert(PTSMap::value_type(frame_->pts, header.stamp));

  int ret = avcodec_send_frame(codecContext_, frame_);
  if (measurePerformance_)
  {
    t3 = ros::WallTime::now();
    tdiffSendFrame_.update((t3 - t2).toSec());
  }
  // now drain all packets
  while (ret == 0)
  {
    ret = drainPacket(header, width, height);
  }
  if (measurePerformance_)
  {
    const ros::WallTime t4 = ros::WallTime::now();
    tdiffTotal_.update((t4 - t0).toSec());
  }
}

int FFMPEGEncoder::drainPacket(const std_msgs::Header& header, int width, int height)
{
  ros::WallTime t0, t1, t2;
  if (measurePerformance_)
  {
    t0 = ros::WallTime::now();
  }

  int ret = avcodec_receive_packet(codecContext_, pkt_);
  if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
  {
    return ret;
  }
  else if (ret < 0)
  {
    ROS_ERROR_STREAM("Error during encoding: ret=" << ret << " [" << av_err2str(ret) << "");
    return ret;
  }

  if (measurePerformance_)
  {
    t1 = ros::WallTime::now();
    tdiffReceivePacket_.update((t1 - t0).toSec());
  }

  if (ret == 0 && pkt_->size > 0)
  {
    ffmpeg_image_transport_msgs::FFMPEGPacketPtr msg = boost::make_shared<ffmpeg_image_transport_msgs::FFMPEGPacket>();
    msg->data.resize(pkt_->size);
    msg->img_width  = width;
    msg->img_height = height;
    msg->pts        = pkt_->pts;
    msg->flags      = pkt_->flags;
    memcpy(&(msg->data[0]), pkt_->data, pkt_->size);
    if (measurePerformance_)
    {
      t2 = ros::WallTime::now();
      totalOutBytes_ += pkt_->size;
      tdiffCopyOut_.update((t2 - t1).toSec());
    }
    msg->header = header;
    auto it     = ptsToStamp_.find(pkt_->pts);
    if (it != ptsToStamp_.end())
    {
      msg->header.stamp = it->second;
      msg->encoding     = codecName_;
      callback_(msg);  // deliver packet callback
      if (measurePerformance_)
      {
        const ros::WallTime t3 = ros::WallTime::now();
        tdiffPublish_.update((t3 - t2).toSec());
      }
      ptsToStamp_.erase(it);
    }
    else
    {
      ROS_ERROR_STREAM("pts " << pkt_->pts << " has no time stamp!");
    }
    av_packet_unref(pkt_);
  }
  return (ret);
}

void FFMPEGEncoder::printTimers(const std::string& prefix) const
{
  Lock lock(mutex_);
  ROS_INFO_STREAM(prefix << " pktsz: " << totalOutBytes_ / frameCnt_
                         << " compr: " << totalInBytes_ / (double)totalOutBytes_ << " debay: " << tdiffDebayer_
                         << " fmcp: " << tdiffFrameCopy_ << " send: " << tdiffSendFrame_
                         << " recv: " << tdiffReceivePacket_ << " cout: " << tdiffCopyOut_ << " publ: " << tdiffPublish_
                         << " tot: " << tdiffTotal_);
}
void FFMPEGEncoder::resetTimers()
{
  Lock lock(mutex_);
  tdiffDebayer_.reset();
  tdiffFrameCopy_.reset();
  tdiffSendFrame_.reset();
  tdiffReceivePacket_.reset();
  tdiffCopyOut_.reset();
  tdiffPublish_.reset();
  tdiffTotal_.reset();
  frameCnt_      = 0;
  totalOutBytes_ = 0;
  totalInBytes_  = 0;
}
void FFMPEGEncoder::setZerolatency(bool zerolatency)
{
  Lock lock(mutex_);
  zerolatency_ = zerolatency;
}
}  // namespace ffmpeg_image_transport
