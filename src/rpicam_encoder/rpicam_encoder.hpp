/* SPDX-License-Identifier: BSD-2-Clause */
//
// Copyright (C) 2025 Matej Vargovcik.
//

#ifndef CAMERA_ROS_RPICAM_ENCODER_HPP
#define CAMERA_ROS_RPICAM_ENCODER_HPP

#include <rclcpp/rclcpp.hpp>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <libcamera/framebuffer.h>
#include "h264_encoder.hpp"

namespace camera {

struct buffer_info_t
{
  void *data;
  size_t size;
};

class RPiCamEncoder {
public:
  RPiCamEncoder(const rclcpp::Publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr &ffmpeg_image_pub);
  void encodeMessage(const libcamera::FrameBuffer *buffer, const std_msgs::msg::Header &header, const buffer_info_t &buffer_info, const libcamera::StreamConfiguration &cfg);
  void setEnabled(bool enabled);
private:
  enum Flag
  {
    FLAG_NONE = 0,
    FLAG_KEYFRAME = 1,
    FLAG_RESTART = 2
  };
  enum State
  {
    DISABLED = 0,
    WAITING_KEYFRAME = 1,
    RUNNING = 2
  };
  std::shared_ptr<VideoOptions> video_options_;
  std::shared_ptr<StreamInfo> stream_info_;
  std::shared_ptr<H264Encoder> encoder_;
  State state_ = DISABLED;
  std::mutex encoder_mutex_;
  bool enabled_ = true;
  std::queue<std_msgs::msg::Header> encoded_image_headers_queue_;
  rclcpp::Publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr ffmpeg_image_pub_;
  void encodeBufferDone(void *mem);
  void outputReady(void *mem, size_t size, int64_t timestamp_us, bool keyframe);
  void outputBuffer(void *mem, size_t size, uint32_t flags, const std_msgs::msg::Header &header);
};

}

#endif //CAMERA_ROS_RPICAM_ENCODER_HPP
