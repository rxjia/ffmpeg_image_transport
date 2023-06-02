/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "ffmpeg_image_transport/ffmpeg_subscriber.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace ffmpeg_image_transport
{

void FFMPEGSubscriber::subscribeImpl(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                                     const Callback& callback, const ros::VoidPtr& tracked_object,
                                     const image_transport::TransportHints& transport_hints)
{
  ros::NodeHandle private_nh("~");
  private_nh.param<std::string>("ffmpeg/decoder/type", decoderType_, "");
  private_nh.param<std::string>("ffmpeg/decoder/hwacc", decoderHwAcc_, "cuda");
  ROS_DEBUG_STREAM("SUBSCRIBER: decoder type: " << decoderType_.c_str() << " hwacc: " << decoderHwAcc_.c_str());
  // bump queue size a bit to avoid lost packets
  queue_size = std::max((int)queue_size, 20);
  FFMPEGSubscriberPlugin::subscribeImpl(nh, base_topic, queue_size, callback, tracked_object, transport_hints);
}

void FFMPEGSubscriber::internalCallback(const FFMPEGPacket::ConstPtr& msg, const Callback& user_cb)
{
  if (decoder_.isInitialized() && decoder_.needReset(msg))
    decoder_.reset();

  if (!decoder_.isInitialized())
  {
    if (msg->flags == 0)
    {
      return;  // wait for key frame!
    }

    if (!decoder_.initialize(
            msg, [&user_cb](const ImageConstPtr& img, bool isKeyFrame) { user_cb(img); }, decoderType_, decoderHwAcc_))
    {
      ROS_ERROR_STREAM("cannot initialize decoder!");
      return;
    }
  }
  decoder_.decodePacket(msg);
}
}  // namespace ffmpeg_image_transport
