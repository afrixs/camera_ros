#include "clamp.hpp"
#include "cv_to_pv.hpp"
#include "format_mapping.hpp"
#include "parameter_conflict_check.hpp"
#include "pretty_print.hpp"
#include "pv_to_cv.hpp"
#include "type_extent.hpp"
#include "types.hpp"
#include <algorithm>
#include <camera_info_manager/camera_info_manager.hpp>
#include <cassert>
#include <cctype>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include "DispatchQueue.h"
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
#include <cv_bridge/cv_bridge.h>
#endif
#include <atomic>
#include <iostream>
#include <libcamera/base/shared_fd.h>
#include <libcamera/base/signal.h>
#include <libcamera/base/span.h>
#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/framebuffer.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/geometry.h>
#include <libcamera/pixel_format.h>
#include <libcamera/property_ids.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>
#include <memory>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <rcl/context.h>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <camera_ros/h264_encoder.hpp>
#include "libcamera/formats.h"
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>
#include <image_transport/image_transport.hpp>

namespace rclcpp
{
class NodeOptions;
}

#define CASE_RANGE(T, R)                                       \
  case libcamera::ControlType##T:                              \
    R.from_value = max<libcamera::ControlType##T>(info.min()); \
    R.to_value = min<libcamera::ControlType##T>(info.max());   \
    break;


namespace camera
{

class PacketEncoder {
public:
  PacketEncoder(const rclcpp::Publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr &ffmpeg_image_pub);
  void encodeMessage(const libcamera::FrameBuffer *buffer, const std_msgs::msg::Header &header, const libcamera::StreamConfiguration &cfg);
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
  void outputReady(void *mem, size_t size, int64_t timestamp_us, bool keyframe);
  void outputBuffer(void *mem, size_t size, uint32_t flags, const std_msgs::msg::Header &header);
};

class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions &options);

  ~CameraNode();

private:
  libcamera::CameraManager camera_manager;
  std::shared_ptr<libcamera::Camera> camera;
  libcamera::Stream *stream;
  std::shared_ptr<libcamera::FrameBufferAllocator> allocator;
  std::vector<std::unique_ptr<libcamera::Request>> requests;
  std::vector<std::thread> request_threads;
  std::unordered_map<const libcamera::Request *, std::mutex> request_mutexes;
  std::unordered_map<const libcamera::Request *, std::condition_variable> request_condvars;
  std::atomic<bool> running = false;
  rtv_utils::DispatchQueue publish_queue{"publish_queue", 1, true};

  struct buffer_info_t
  {
    void *data;
    size_t size;
  };
  std::unordered_map<const libcamera::FrameBuffer *, buffer_info_t> buffer_info;

  // timestamp offset (ns) from camera time to system time
  int64_t time_offset = 0;


  image_transport::Publisher pub_image;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_pure;
  rclcpp::Publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr pub_image_ffmpeg;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_ci;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_set_capture;

  std::shared_ptr<PacketEncoder> ffmpeg_encoder;

  camera_info_manager::CameraInfoManager cim;

  OnSetParametersCallbackHandle::SharedPtr callback_parameter_change;

  // map parameter names to libcamera control id
  std::unordered_map<std::string, const libcamera::ControlId *> parameter_ids;
  // parameters that are to be set for every request
  std::unordered_map<unsigned int, libcamera::ControlValue> parameters;
  // keep track of set parameters
  ParameterMap parameters_full;
  std::mutex parameters_mutex;
  std::unique_lock<std::mutex> parameters_lock {parameters_mutex, std::defer_lock};
  std::mutex publish_mutex;
  // compression quality parameter
  std::atomic_uint8_t jpeg_quality;
  bool publish_via_image_transport;

  bool
  setCaptureEnabled(bool enabled);

  void
  declareParameters();

  void
  requestComplete(libcamera::Request *const request);

  void
  process(libcamera::Request *const request);

  bool stop();

  rcl_interfaces::msg::SetParametersResult
  onParameterChange(const std::vector<rclcpp::Parameter> &parameters);
};

RCLCPP_COMPONENTS_REGISTER_NODE(camera::CameraNode)


libcamera::StreamRole
get_role(const std::string &role)
{
  static const std::unordered_map<std::string, libcamera::StreamRole> roles_map = {
    {"raw", libcamera::StreamRole::Raw},
    {"still", libcamera::StreamRole::StillCapture},
    {"video", libcamera::StreamRole::VideoRecording},
    {"viewfinder", libcamera::StreamRole::Viewfinder},
  };

  try {
    return roles_map.at(role);
  }
  catch (const std::out_of_range &) {
    throw std::runtime_error("invalid stream role: \"" + role + "\"");
  }
}


// The following function "compressImageMsg" is adapted from "CvImage::toCompressedImageMsg"
// (https://github.com/ros-perception/vision_opencv/blob/066793a23e5d06d76c78ca3d69824a501c3554fd/cv_bridge/src/cv_bridge.cpp#L512-L535)
// and covered by the BSD-3-Clause licence.
void
compressImageMsg(const sensor_msgs::msg::Image &source,
                 sensor_msgs::msg::CompressedImage &destination,
                 const std::vector<int> &params = std::vector<int>())
{
  namespace enc = sensor_msgs::image_encodings;

  std::shared_ptr<CameraNode> tracked_object;
  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(source, tracked_object);

  destination.header = source.header;
  cv::Mat image;
  if (cv_ptr->encoding == enc::BGR8 || cv_ptr->encoding == enc::BGRA8 ||
      cv_ptr->encoding == enc::MONO8 || cv_ptr->encoding == enc::MONO16)
  {
    image = cv_ptr->image;
  }
  else {
    cv_bridge::CvImagePtr tempThis = std::make_shared<cv_bridge::CvImage>(*cv_ptr);
    cv_bridge::CvImagePtr temp;
    if (enc::hasAlpha(cv_ptr->encoding)) {
      temp = cv_bridge::cvtColor(tempThis, enc::BGRA8);
    }
    else {
      temp = cv_bridge::cvtColor(tempThis, enc::BGR8);
    }
    image = temp->image;
  }

  destination.format = "jpg";
  cv::imencode(".jpg", image, destination.data, params);
}


CameraNode::CameraNode(const rclcpp::NodeOptions &options) : Node("camera", options), cim(this)
{
  // pixel format
  rcl_interfaces::msg::ParameterDescriptor param_descr_format;
  param_descr_format.description = "pixel format of streaming buffer";
  param_descr_format.read_only = true;
  declare_parameter<std::string>("format", {}, param_descr_format);

  // stream role
  rcl_interfaces::msg::ParameterDescriptor param_descr_role;
  param_descr_role.description = "stream role";
  param_descr_role.additional_constraints = "one of {raw, still, video, viewfinder}";
  param_descr_role.read_only = true;
  declare_parameter<std::string>("role", "viewfinder", param_descr_role);

  // image dimensions
  rcl_interfaces::msg::ParameterDescriptor param_descr_ro;
  param_descr_ro.read_only = true;
  declare_parameter<int64_t>("width", {}, param_descr_ro);
  declare_parameter<int64_t>("height", {}, param_descr_ro);

  // camera ID
  declare_parameter("camera", rclcpp::ParameterValue {}, param_descr_ro.set__dynamic_typing(true));

  declare_parameter("publish_via_image_transport", true);

  rcl_interfaces::msg::ParameterDescriptor jpeg_quality_description;
  jpeg_quality_description.name = "jpeg_quality";
  jpeg_quality_description.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  jpeg_quality_description.description = "Image quality for JPEG format";
  jpeg_quality_description.read_only = false;
  rcl_interfaces::msg::IntegerRange jpeg_range;
  jpeg_range.from_value = 1;
  jpeg_range.to_value = 100;
  jpeg_range.step = 1;
  jpeg_quality_description.integer_range = {jpeg_range};
  // default to 95
  jpeg_quality = declare_parameter<uint8_t>("jpeg_quality", 95, jpeg_quality_description);
  // publisher for raw and compressed image
  publish_via_image_transport = get_parameter("publish_via_image_transport").as_bool();
  if (publish_via_image_transport) {
    pub_image = image_transport::create_publisher(this, "image_raw", rclcpp::QoS(1).best_effort().get_rmw_qos_profile());
  }
  else {
    pub_image_pure = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 1);
    pub_image_ffmpeg = this->create_publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>("image_raw/ffmpeg", 1);
    ffmpeg_encoder = std::make_shared<PacketEncoder>(pub_image_ffmpeg);
  }
  pub_ci = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 1);

  service_set_capture = this->create_service<std_srvs::srv::SetBool>(
    "set_capture", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                            std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      bool success = setCaptureEnabled(request->data);
      response->success = success;

      if (success)
        response->message = std::string("Capture ") + (request->data ? "enabled" : "disabled");
      else
        response->message = std::string("Failed to ") + (request->data ? "enable" : "disable") + " capture";

      RCLCPP_INFO_STREAM(get_logger(), response->message);
    });

  // start camera manager and check for cameras
  camera_manager.start();
  if (camera_manager.cameras().empty())
    throw std::runtime_error("no cameras available");

  // get the camera
  switch (get_parameter("camera").get_type()) {
  case rclcpp::ParameterType::PARAMETER_NOT_SET:
    // use first camera as default
    camera = camera_manager.cameras().front();
    RCLCPP_INFO_STREAM(get_logger(), camera_manager);
    RCLCPP_WARN_STREAM(get_logger(),
                       "no camera selected, using default: \"" << camera->id() << "\"");
    RCLCPP_WARN_STREAM(get_logger(), "set parameter 'camera' to silence this warning");
    break;
  case rclcpp::ParameterType::PARAMETER_INTEGER:
  {
    const size_t id = get_parameter("camera").as_int();
    if (id >= camera_manager.cameras().size()) {
      RCLCPP_INFO_STREAM(get_logger(), camera_manager);
      throw std::runtime_error("camera with id " + std::to_string(id) + " does not exist");
    }
    camera = camera_manager.cameras().at(id);
    RCLCPP_DEBUG_STREAM(get_logger(), "found camera by id: " << id);
  } break;
  case rclcpp::ParameterType::PARAMETER_STRING:
  {
    const std::string name = get_parameter("camera").as_string();
    camera = camera_manager.get(name);
    if (!camera) {
      RCLCPP_INFO_STREAM(get_logger(), camera_manager);
      throw std::runtime_error("camera with name " + name + " does not exist");
    }
    RCLCPP_DEBUG_STREAM(get_logger(), "found camera by name: \"" << name << "\"");
  } break;
  default:
    RCLCPP_ERROR_STREAM(get_logger(), "unsupported camera parameter type: "
                                        << get_parameter("camera").get_type_name());
    break;
  }

  if (!camera)
    throw std::runtime_error("failed to find camera");

  if (camera->acquire())
    throw std::runtime_error("failed to acquire camera");

  // configure camera stream
  const libcamera::StreamRole role = get_role(get_parameter("role").as_string());
  std::unique_ptr<libcamera::CameraConfiguration> cfg =
    camera->generateConfiguration({role});

  if (!cfg)
    throw std::runtime_error("failed to generate configuration");

  assert(cfg->size() == 1);
  libcamera::StreamConfiguration &scfg = cfg->at(0);

  // list all camera formats, including those not supported by the ROS message
  RCLCPP_DEBUG_STREAM(get_logger(), "default " << role << " stream configuration: \"" << scfg.toString() << "\"");
  RCLCPP_DEBUG_STREAM(get_logger(), scfg.formats());

  // get common pixel formats that are supported by the camera and the node
  const libcamera::StreamFormats stream_formats = get_common_stream_formats(scfg.formats());
  const std::vector<libcamera::PixelFormat> common_fmt = stream_formats.pixelformats();

  RCLCPP_INFO_STREAM(get_logger(), stream_formats);

  if (common_fmt.empty())
    throw std::runtime_error("camera does not provide any of the supported pixel formats");

  const std::string format = get_parameter("format").as_string();
  if (format.empty()) {
    // check that the default pixel format is supported by the ROS encoding
    if (std::find(common_fmt.cbegin(), common_fmt.cend(), scfg.pixelFormat) == common_fmt.cend()) {
      // auto select first common pixel format
      RCLCPP_WARN_STREAM(get_logger(), "default pixel format (" << scfg.pixelFormat << ") not supported");
      scfg.pixelFormat = common_fmt.front();
    }

    RCLCPP_WARN_STREAM(get_logger(),
                       "no pixel format selected, auto-selecting: \"" << scfg.pixelFormat << "\"");
    RCLCPP_WARN_STREAM(get_logger(), "set parameter 'format' to silence this warning");
  }
  else {
    // get pixel format from provided string
    const libcamera::PixelFormat format_requested = libcamera::PixelFormat::fromString(format);
    if (!format_requested.isValid()) {
      throw std::runtime_error("invalid pixel format: \"" + format + "\"");
    }
    // check that the requested format is supported by camera and the node
    if (std::find(common_fmt.begin(), common_fmt.end(), format_requested) == common_fmt.end()) {
      throw std::runtime_error("unsupported pixel format \"" + format + "\"");
    }
    scfg.pixelFormat = format_requested;
  }

  RCLCPP_INFO_STREAM(get_logger(), list_format_sizes(scfg));

  const libcamera::Size size(get_parameter("width").as_int(), get_parameter("height").as_int());
  if (size.isNull()) {
    RCLCPP_WARN_STREAM(get_logger(), "no dimensions selected, auto-selecting: \"" << scfg.size << "\"");
    RCLCPP_WARN_STREAM(get_logger(), "set parameter 'width' or 'height' to silence this warning");
  }
  else {
    scfg.size = size;
  }

  // store selected stream configuration
  const libcamera::StreamConfiguration selected_scfg = scfg;

  switch (cfg->validate()) {
  case libcamera::CameraConfiguration::Valid:
    break;
  case libcamera::CameraConfiguration::Adjusted:
    RCLCPP_WARN_STREAM(get_logger(), "stream configuration adjusted from \""
                                       << selected_scfg.toString() << "\" to \"" << scfg.toString()
                                       << "\"");
    break;
  case libcamera::CameraConfiguration::Invalid:
    throw std::runtime_error("failed to validate stream configurations");
    break;
  }

  switch (camera->configure(cfg.get())) {
  case -ENODEV:
    throw std::runtime_error("configure: camera has been disconnected from the system");
  case -EACCES:
    throw std::runtime_error("configure: camera is not in a state where it can be configured");
  case -EINVAL:
    throw std::runtime_error("configure: configuration \"" + scfg.toString() + "\" is not valid");
  default:
    RCLCPP_INFO_STREAM(get_logger(), "camera \"" << camera->id() << "\" configured with "
                                                 << scfg.toString() << " stream");
    break;
  }

  // format camera name for calibration file
  const libcamera::ControlList &props = camera->properties();
  std::string cname = camera->id() + '_' + scfg.size.toString();
  const std::optional<std::string> model = props.get(libcamera::properties::Model);
  if (model)
    cname = model.value() + '_' + cname;

  // clean camera name of non-alphanumeric characters
  cname.erase(
    std::remove_if(cname.begin(), cname.end(), [](const char &x) { return std::isspace(x); }),
    cname.cend());
  std::replace_if(
    cname.begin(), cname.end(), [](const char &x) { return !std::isalnum(x); }, '_');

  if (!cim.setCameraName(cname))
    throw std::runtime_error("camera name must only contain alphanumeric characters");

  declareParameters();

  // allocate stream buffers and create one request per buffer
  stream = scfg.stream();

  allocator = std::make_shared<libcamera::FrameBufferAllocator>(camera);
  allocator->allocate(stream);

  // register callback
  camera->requestCompleted.connect(this, &CameraNode::requestComplete);

  if (!setCaptureEnabled(true))
    throw std::runtime_error("failed to start camera");
}

CameraNode::~CameraNode()
{
  if (!stop())
    std::cerr << "failed to stop camera" << std::endl;
  allocator->free(stream);
  allocator.reset();
  camera->release();
  camera.reset();
  camera_manager.stop();
  for (const auto &e : buffer_info)
    if (munmap(e.second.data, e.second.size) == -1)
      std::cerr << "munmap failed: " << std::strerror(errno) << std::endl;
}

bool CameraNode::setCaptureEnabled(bool enabled) {
  if (enabled) {
    if (running)
      return true;  // already running

    requests.clear();
    request_threads.clear();
    for (const std::unique_ptr<libcamera::FrameBuffer> &buffer : allocator->buffers(stream)) {
      std::unique_ptr<libcamera::Request> request = camera->createRequest();
      if (!request)
        throw std::runtime_error("Can't create request");

      // multiple planes of the same buffer use the same file descriptor
      size_t buffer_length = 0;
      int fd = -1;
      for (const libcamera::FrameBuffer::Plane &plane : buffer->planes()) {
        if (plane.offset == libcamera::FrameBuffer::Plane::kInvalidOffset)
          throw std::runtime_error("invalid offset");
        buffer_length = std::max<size_t>(buffer_length, plane.offset + plane.length);
        if (!plane.fd.isValid())
          throw std::runtime_error("file descriptor is not valid");
        if (fd == -1)
          fd = plane.fd.get();
        else if (fd != plane.fd.get())
          throw std::runtime_error("plane file descriptors differ");
      }

      // memory-map the frame buffer planes
      void *data = mmap(nullptr, buffer_length, PROT_READ, MAP_SHARED, fd, 0);
      if (data == MAP_FAILED)
        throw std::runtime_error("mmap failed: " + std::string(std::strerror(errno)));
      buffer_info[buffer.get()] = {data, buffer_length};

      if (request->addBuffer(stream, buffer.get()) < 0)
        throw std::runtime_error("Can't set buffer for request");

      requests.push_back(std::move(request));
      break;
    }

    // create a processing thread per request
    running = true;
    for (const std::unique_ptr<libcamera::Request> &request : requests) {
      // create mutexes in-place
      request_mutexes[request.get()];
      request_condvars[request.get()];
      request_threads.emplace_back(&CameraNode::process, this, request.get());
    }

    libcamera::ControlList controls_ = camera->controls();
    for (const auto &[id, value] : parameters)
      controls_.set(id, value);

    // start camera and queue all requests
    if (camera->start(&controls_)) {
      running = false;
      for (std::thread &thread : request_threads)
        thread.join();
      return false;
    }
    else {
      for (std::unique_ptr<libcamera::Request> &request: requests)
        camera->queueRequest(request.get());
      return true;
    }
  }
  else {
    if (!running)
      return true;  // already stopped
    return stop();
  }
}

bool CameraNode::stop() {
  // stop request callbacks
  for (std::unique_ptr<libcamera::Request> &request : requests)
    camera->requestCompleted.disconnect(request.get());

  // stop request processing threads
  running = false;

  // unlock all threads
  for (auto &[req, condvar] : request_condvars)
    condvar.notify_all();

  // wait for all currently running threads to finish
  for (std::thread &thread : request_threads)
    thread.join();

  // stop camera
  return !camera->stop();
}

void
CameraNode::declareParameters()
{
  // dynamic camera configuration
  ParameterMap parameters_init;
  for (const auto &[id, info] : camera->controls()) {
    // store control id with name
    parameter_ids[id->name()] = id;

    if (info.min().numElements() != info.max().numElements())
      throw std::runtime_error("minimum and maximum parameter array sizes do not match");

    // check if the control can be mapped to a parameter
    rclcpp::ParameterType pv_type;
    try {
      pv_type = cv_to_pv_type(id);
      if (pv_type == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        RCLCPP_WARN_STREAM(get_logger(), "unsupported control '" << id->name() << "'");
        continue;
      }
    }
    catch (const std::runtime_error &e) {
      // ignore
      RCLCPP_WARN_STREAM(get_logger(), e.what());
      continue;
    }

    // format type description
    rcl_interfaces::msg::ParameterDescriptor param_descr;
    try {
      const std::size_t extent = get_extent(id);
      const bool scalar = (extent == 0);
      const bool dynamic = (extent == libcamera::dynamic_extent);
      const std::string cv_type_descr =
        scalar ? "scalar" : "array[" + (dynamic ? std::string() : std::to_string(extent)) + "]";
      param_descr.description =
        std::to_string(id->type()) + " " + cv_type_descr + " range {" + info.min().toString() +
        "}..{" + info.max().toString() + "}" +
        (info.def().isNone() ? std::string {} : " (default: {" + info.def().toString() + "})");
    }
    catch (const std::runtime_error &e) {
      // ignore
      RCLCPP_WARN_STREAM(get_logger(), e.what());
      continue;
    }

    // get smallest bounds for minimum and maximum set
    rcl_interfaces::msg::IntegerRange range_int;
    rcl_interfaces::msg::FloatingPointRange range_float;

    switch (id->type()) {
      CASE_RANGE(Integer32, range_int)
      CASE_RANGE(Integer64, range_int)
      CASE_RANGE(Float, range_float)
#if LIBCAMERA_VER_GE(0, 4, 0)
      CASE_RANGE(Unsigned16, range_int)
      CASE_RANGE(Unsigned32, range_int)
#endif
    default:
      break;
    }

    if (range_int.from_value != range_int.to_value)
      param_descr.integer_range = {range_int};
    if (range_float.from_value != range_float.to_value)
      param_descr.floating_point_range = {range_float};

    // clamp default ControlValue to min/max range and cast ParameterValue
    rclcpp::ParameterValue value;
    try {
      value = cv_to_pv(clamp(info.def(), info.min(), info.max()));
    }
    catch (const invalid_conversion &e) {
      RCLCPP_ERROR_STREAM(get_logger(), "unsupported control '"
                                          << id->name()
                                          << "' (type: " << std::to_string(info.def().type()) << "): "
                                          << e.what());
      continue;
    }

    // declare parameters and set default or initial value
    RCLCPP_DEBUG_STREAM(get_logger(),
                        "declare " << id->name() << " with default " << rclcpp::to_string(value));

    if (value.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      declare_parameter(id->name(), pv_type, param_descr);
    }
    else {
      declare_parameter(id->name(), value, param_descr);
    }
    parameters_init[id->name()] = value;
  }

  // register callback to handle parameter changes
  // We have to register the callback after parameter declaration
  // to avoid callbacks interfering with the default parameter check.
  callback_parameter_change = add_on_set_parameters_callback(
    std::bind(&CameraNode::onParameterChange, this, std::placeholders::_1));

  // resolve conflicts of default libcamera configuration and user provided overrides
  std::vector<std::string> status;
  std::tie(parameters_init, status) =
    resolve_conflicts(parameters_init, get_node_parameters_interface()->get_parameter_overrides());

  for (const std::string &s : status)
    RCLCPP_WARN_STREAM(get_logger(), s);

  std::vector<rclcpp::Parameter> parameters_init_list;
  for (const auto &[name, value] : parameters_init) {
    if (value.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET)
      parameters_init_list.emplace_back(name, value);
  }
  const rcl_interfaces::msg::SetParametersResult param_set_result =
    set_parameters_atomically(parameters_init_list);
  if (!param_set_result.successful)
    RCLCPP_ERROR_STREAM(get_logger(), "Cannot declare parameters with default value: " << param_set_result.reason);
}

void
CameraNode::requestComplete(libcamera::Request *const request)
{
  std::unique_lock lk(request_mutexes.at(request));
  request_condvars.at(request).notify_one();
}

void
CameraNode::process(libcamera::Request *const request)
{
  while (true) {
    // block until request is available
    std::unique_lock lk(request_mutexes.at(request));
    request_condvars.at(request).wait(lk);

    if (!running)
      return;

    if (request->status() == libcamera::Request::RequestComplete) {
      assert(request->buffers().size() == 1);

      // get the stream and buffer from the request
      const libcamera::FrameBuffer *buffer = request->findBuffer(stream);
      const libcamera::FrameMetadata &metadata = buffer->metadata();
      size_t bytesused = 0;
      for (const libcamera::FrameMetadata::Plane &plane : metadata.planes())
        bytesused += plane.bytesused;

      // set time offset once for accurate timing using the device time
      if (time_offset == 0)
        time_offset = this->now().nanoseconds() - metadata.timestamp;

      // send image data
      std_msgs::msg::Header hdr;
      hdr.stamp = rclcpp::Time(time_offset + int64_t(metadata.timestamp));
      hdr.frame_id = "camera";
      const libcamera::StreamConfiguration &cfg = stream->configuration();

      bool publish_image = (publish_via_image_transport && pub_image.getNumSubscribers() > 0) || pub_image_pure->get_subscription_count() > 0;
      bool publish_ffmpeg = !publish_via_image_transport && pub_image_ffmpeg->get_subscription_count() > 0;

      sensor_msgs::msg::Image::SharedPtr msg_img;
      ffmpeg_image_transport_msgs::msg::FFMPEGPacket::SharedPtr ffmpeg_packet;

      if (format_type(cfg.pixelFormat) == FormatType::RAW) {
        // raw uncompressed image
        assert(buffer_info[buffer].size == bytesused);
        if (publish_image) {
          msg_img = std::make_shared<sensor_msgs::msg::Image>();
          msg_img->header = hdr;
          msg_img->width = cfg.size.width;
          msg_img->height = cfg.size.height;
          msg_img->step = cfg.stride;
          msg_img->encoding = get_ros_encoding(cfg.pixelFormat);
          msg_img->is_bigendian = (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__);
          msg_img->data.resize(buffer_info[buffer].size);
          memcpy(msg_img->data.data(), buffer_info[buffer].data, buffer_info[buffer].size);
        }
        if (publish_ffmpeg) {
          ffmpeg_encoder->encodeMessage(buffer, hdr, cfg);
        }
      }
      else if (format_type(cfg.pixelFormat) == FormatType::COMPRESSED) {
        // compressed image
        assert(bytesused < buffer_info[buffer].size);

        // decompress into raw rgb8 image
        if (publish_image) {
          auto msg_img_compressed = std::make_unique<sensor_msgs::msg::CompressedImage>();
          msg_img_compressed->header = hdr;
          msg_img_compressed->format = get_ros_encoding(cfg.pixelFormat);
          msg_img_compressed->data.resize(bytesused);
          memcpy(msg_img_compressed->data.data(), buffer_info[buffer].data, bytesused);

          msg_img = std::make_shared<sensor_msgs::msg::Image>();
          cv_bridge::toCvCopy(*msg_img_compressed, "rgb8")->toImageMsg(*msg_img);
        }
        if (publish_ffmpeg) {
          ffmpeg_encoder->encodeMessage(buffer, hdr, cfg);
        }
      }
      else {
        throw std::runtime_error("unsupported pixel format: " +
                                 stream->configuration().pixelFormat.toString());
      }

//       std::unique_lock<std::mutex> lock(publish_mutex);
      if (publish_image)
        publish_queue.dispatch([this, msg_img, hdr, ffmpeg_packet]() {
          //
          // RCLCPP_INFO(get_logger(), "publishing image %lf", rclcpp::Time(msg_img->header.stamp).seconds());
          if (publish_via_image_transport)
            pub_image.publish(msg_img);
          else {
            if (msg_img)
              pub_image_pure->publish(*msg_img);
            if (ffmpeg_packet)
              pub_image_ffmpeg->publish(*ffmpeg_packet);
          }
          // RCLCPP_INFO(get_logger(), "publishing done");

          sensor_msgs::msg::CameraInfo ci = cim.getCameraInfo();
          ci.header = hdr;
          pub_ci->publish(ci);
        });
      else {
        sensor_msgs::msg::CameraInfo ci = cim.getCameraInfo();
        ci.header = hdr;
        pub_ci->publish(ci);
      }
    }
    else if (request->status() == libcamera::Request::RequestCancelled) {
      RCLCPP_ERROR_STREAM(get_logger(), "request '" << request->toString() << "' cancelled");
    }

    // queue the request again for the next frame
    request->reuse(libcamera::Request::ReuseBuffers);

    // update parameters
    parameters_lock.lock();
    if (!parameters.empty()) {
      RCLCPP_DEBUG_STREAM(get_logger(), request->toString() << " setting " << parameters.size() << " controls");
      for (const auto &[id, value] : parameters) {
        const std::string &name = libcamera::controls::controls.at(id)->name();
        RCLCPP_DEBUG_STREAM(get_logger(), "apply " << name << ": " << value.toString());
        request->controls().set(id, value);
      }
      parameters.clear();
    }
    parameters_lock.unlock();

    camera->queueRequest(request);
  }
}

PacketEncoder::PacketEncoder(const rclcpp::Publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr &ffmpeg_image_pub)
{
  ffmpeg_image_pub_ = ffmpeg_image_pub;
}

void PacketEncoder::encodeMessage(const libcamera::FrameBuffer *buffer, const std_msgs::msg::Header &header, const libcamera::StreamConfiguration &cfg) {
  RCLCPP_INFO(rclcpp::get_logger("PacketEncoder"), "encoding message");
  std::unique_lock<std::mutex> lock(encoder_mutex_);
  if (enabled_ && !encoder_) {
    RCLCPP_INFO(rclcpp::get_logger("PacketEncoder"), "creating encoder");
    stream_info_ = std::make_shared<StreamInfo>();
    stream_info_->width = cfg.size.width;
    stream_info_->height = cfg.size.height;
    stream_info_->stride = cfg.stride;
    stream_info_->pixel_format = cfg.pixelFormat;

    video_options_ = std::make_shared<VideoOptions>();
    encoder_ = std::make_shared<H264Encoder>(video_options_.get(), *stream_info_);
    encoder_->SetOutputReadyCallback(std::bind(&PacketEncoder::outputReady, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
  }
  if (encoder_) {
    encoded_image_headers_queue_.push(header);
    auto stamp = header.stamp;
    RCLCPP_INFO(rclcpp::get_logger("PacketEncoder"), "using encoder");
    encoder_->EncodeBuffer(buffer->planes()[0].fd.get(), cfg.stride * cfg.size.height, nullptr, *stream_info_, stamp.sec * 1000000 + stamp.nanosec / 1000);
  }
}

void PacketEncoder::outputReady(void *mem, size_t size, int64_t timestamp_us, bool keyframe)
{
  RCLCPP_INFO(rclcpp::get_logger("PacketEncoder"), "output ready");
  std::unique_lock<std::mutex> lock(encoder_mutex_);
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

  RCLCPP_INFO(rclcpp::get_logger("PacketEncoder"), "outputBuffer");
  const auto &header = encoded_image_headers_queue_.front();
  outputBuffer(mem, size, flags, header);
  encoded_image_headers_queue_.pop();
}

void PacketEncoder::outputBuffer(void *mem, size_t size, uint32_t flags, const std_msgs::msg::Header &header)
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
  RCLCPP_INFO(rclcpp::get_logger("PacketEncoder"), "publishing");
  ffmpeg_image_pub_->publish(packet);
}

rcl_interfaces::msg::SetParametersResult
CameraNode::onParameterChange(const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  // check target parameter state (current and new parameters)
  // for conflicting configuration
  const std::vector<std::string> msgs = check_conflicts(parameters, parameters_full);
  if (!msgs.empty()) {
    result.successful = false;
    for (size_t i = 0; i < msgs.size(); i++) {
      if (msgs.size() > 1)
        result.reason += "(" + std::to_string(i) + ") ";
      result.reason += msgs[i];
      if (i < msgs.size() - 1)
        result.reason += "; ";
    }
    return result;
  }

  result.successful = true;

  for (const rclcpp::Parameter &parameter : parameters) {
    RCLCPP_DEBUG_STREAM(get_logger(), "setting " << parameter.get_type_name() << " parameter "
                                                 << parameter.get_name() << " to "
                                                 << parameter.value_to_string());

    if (parameter_ids.count(parameter.get_name())) {
      const libcamera::ControlId *id = parameter_ids.at(parameter.get_name());
      libcamera::ControlValue value = pv_to_cv(parameter, id->type());

      if (!value.isNone()) {
        // verify parameter type and dimension against default
        const libcamera::ControlInfo &ci = camera->controls().at(id);

        if (value.type() != id->type()) {
          result.successful = false;
          result.reason = parameter.get_name() + ": parameter types mismatch, expected '" +
                          std::to_string(id->type()) + "', got '" + std::to_string(value.type()) +
                          "'";
          return result;
        }

        const std::size_t extent = get_extent(id);
        if (value.isArray() &&
            (extent != libcamera::dynamic_extent) &&
            (value.numElements() != extent))
        {
          result.successful = false;
          result.reason = parameter.get_name() + ": array dimensions mismatch, expected " +
                          std::to_string(extent) + ", got " + std::to_string(value.numElements());
          return result;
        }

        // check bounds and return error
        if (value < ci.min() || value > ci.max()) {
          result.successful = false;
          result.reason =
            "parameter value " + value.toString() + " outside of range: " + ci.toString();
          return result;
        }

        parameters_lock.lock();
        this->parameters[parameter_ids.at(parameter.get_name())->id()] = value;
        parameters_lock.unlock();

        parameters_full[parameter.get_name()] = parameter.get_parameter_value();
      }
    }
    else if (!parameter.get_name().compare("jpeg_quality")) {
      jpeg_quality = parameter.get_parameter_value().get<uint8_t>();
    }
  }

  return result;
}

} // namespace camera
