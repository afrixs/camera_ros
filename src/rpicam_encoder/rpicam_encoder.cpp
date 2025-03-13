/* SPDX-License-Identifier: BSD-2-Clause */
//
// Copyright (C) 2025 Matej Vargovcik.
//

#include "rpicam_encoder.hpp"

namespace camera {

RPiCamEncoder::RPiCamEncoder(const rclcpp::Publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr &ffmpeg_image_pub)
{
  ffmpeg_image_pub_ = ffmpeg_image_pub;
}

void RPiCamEncoder::setEnabled(bool enabled)
{
  std::unique_lock<std::mutex> lock(encoder_mutex_);
  enabled_ = enabled;
  if (!enabled && encoder_)
  {
    RCLCPP_INFO(rclcpp::get_logger("PacketEncoder"), "destroying encoder");
    encoder_.reset();  // Note: maybe setting state to disabled would be functionally enough. But keeping this here so that the first frame after re-enabling will be a keyframe.
    state_ = DISABLED;
  }
}

void RPiCamEncoder::encodeMessage(const libcamera::FrameBuffer *buffer, const std_msgs::msg::Header &header, const buffer_info_t &buffer_info, const libcamera::StreamConfiguration &cfg) {
  std::unique_lock<std::mutex> lock(encoder_mutex_);
  if (enabled_ && !encoder_) {
    RCLCPP_INFO(rclcpp::get_logger("PacketEncoder"), "creating encoder");
    stream_info_ = std::make_shared<StreamInfo>();
    stream_info_->width = cfg.size.width;
    stream_info_->height = cfg.size.height;
    stream_info_->stride = cfg.stride;
    stream_info_->pixel_format = cfg.pixelFormat;
    stream_info_->colour_space = cfg.colorSpace;

    video_options_ = std::make_shared<VideoOptions>();
    video_options_->Parse(0, nullptr);
    video_options_->width = stream_info_->width;
    video_options_->height = stream_info_->height;
    video_options_->inline_headers = true;
    encoder_ = std::make_shared<H264Encoder>(video_options_.get(), *stream_info_);
    encoder_->SetOutputReadyCallback(std::bind(&RPiCamEncoder::outputReady, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    encoder_->SetInputDoneCallback(std::bind(&RPiCamEncoder::encodeBufferDone, this, std::placeholders::_1));
    RCLCPP_INFO(rclcpp::get_logger("PacketEncoder"), "encoder created");
  }
  if (encoder_) {
    encoded_image_headers_queue_.push(header);
    auto stamp = header.stamp;
    encoder_->EncodeBuffer(buffer->planes()[0].fd.get(), buffer_info.size, nullptr, *stream_info_, stamp.sec * 1000000 + stamp.nanosec / 1000);
  }
}

void RPiCamEncoder::encodeBufferDone([[maybe_unused]] void *mem)
{
}

void RPiCamEncoder::outputReady(void *mem, size_t size, [[maybe_unused]] int64_t timestamp_us, bool keyframe)
{
  std::unique_lock<std::mutex> lock(encoder_mutex_);
  const auto header = encoded_image_headers_queue_.front();
  encoded_image_headers_queue_.pop();
  // When output is enabled, we may have to wait for the next keyframe.
  uint32_t flags = keyframe ? FLAG_KEYFRAME : FLAG_NONE;
  if (!encoder_)
    state_ = DISABLED;
  else if (state_ == DISABLED)
    state_ = WAITING_KEYFRAME;
  if (state_ == WAITING_KEYFRAME && keyframe)
    state_ = RUNNING, flags |= FLAG_RESTART;
  if (state_ != RUNNING)
    return;

  outputBuffer(mem, size, flags, header);
}

void RPiCamEncoder::outputBuffer(void *mem, size_t size, uint32_t flags, const std_msgs::msg::Header &header)
{
  ffmpeg_image_transport_msgs::msg::FFMPEGPacket packet;
  packet.header = header;
  packet.data.resize(size);
  memcpy(packet.data.data(), mem, size);
  packet.flags = flags;
  packet.encoding = "h264";
  packet.width = stream_info_->width;
  packet.height = stream_info_->height;
  packet.pts = packet.width*packet.height;
  ffmpeg_image_pub_->publish(packet);
}

}
