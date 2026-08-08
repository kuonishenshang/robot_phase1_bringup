#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rknn_yolov8_ros/msg/detection_array.hpp>
#include <person_tracking_msgs/msg/fused_person_target.hpp>
#include <person_tracking_msgs/msg/radar_track_array.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

class PersonTargetFusionNode final : public rclcpp::Node
{
public:
  using DetectionArray = rknn_yolov8_ros::msg::DetectionArray;
  using RadarTrackArray = person_tracking_msgs::msg::RadarTrackArray;
  using RadarTrack = person_tracking_msgs::msg::RadarTrack;
  using FusedTarget = person_tracking_msgs::msg::FusedPersonTarget;

  explicit PersonTargetFusionNode()
  : Node("person_target_fusion_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    vision_topic_ = declare_parameter("vision_topic", std::string("/yolo/detections"));
    radar_topic_ = declare_parameter("radar_topic", std::string("/radar/tracks"));
    output_topic_ = declare_parameter("output_topic", std::string("/person_tracking/target"));
    tracking_frame_ = declare_parameter("tracking_frame", std::string("odom"));
    person_class_id_ = declare_parameter("person_class_id", 0);
    min_vision_confidence_ = declare_parameter("min_vision_confidence", 0.70);
    association_confirm_frames_ = declare_parameter("association_confirm_frames", 3);
    association_max_distance_m_ = declare_parameter("association_max_distance_m", 0.80);
    association_max_time_diff_sec_ = declare_parameter("association_max_time_diff_sec", 0.15);
    association_min_margin_m_ = declare_parameter("association_min_margin_m", 0.25);
    vision_timeout_sec_ = declare_parameter("vision_timeout_sec", 0.30);
    radar_timeout_sec_ = declare_parameter("radar_timeout_sec", 0.60);
    sensor_gap_timeout_sec_ = declare_parameter("sensor_gap_timeout_sec", 0.80);
    radar_min_confidence_ = declare_parameter("radar_min_confidence", 0.0);
    use_radar_velocity_ = declare_parameter("use_radar_velocity", false);
    vision_only_navigation_ = declare_parameter("vision_only_navigation", true);
    transform_timeout_sec_ = declare_parameter("transform_timeout_sec", 0.08);
    const double rate = declare_parameter("publish_rate_hz", 20.0);

    target_pub_ = create_publisher<FusedTarget>(output_topic_, rclcpp::QoS(5).reliable());
    vision_sub_ = create_subscription<DetectionArray>(
      vision_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PersonTargetFusionNode::vision_callback, this, std::placeholders::_1));
    radar_sub_ = create_subscription<RadarTrackArray>(
      radar_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PersonTargetFusionNode::radar_callback, this, std::placeholders::_1));
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / std::max(1.0, rate)));
    timer_ = create_wall_timer(period, std::bind(&PersonTargetFusionNode::timer_callback, this));
    RCLCPP_INFO(get_logger(), "Fusion listening to %s and %s; output %s in %s",
      vision_topic_.c_str(), radar_topic_.c_str(), output_topic_.c_str(), tracking_frame_.c_str());
  }

private:
  struct VisionObservation
  {
    geometry_msgs::msg::Point point;
    builtin_interfaces::msg::Time stamp;
    double confidence{0.0};
    int64_t tracking_id{-1};
  };

  struct RadarObservation
  {
    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Vector3 velocity;
    geometry_msgs::msg::Vector3 acceleration;
    builtin_interfaces::msg::Time stamp;
    uint32_t frame_number{0};
    float confidence{0.0F};
    float position_variance{std::numeric_limits<float>::quiet_NaN()};
  };

  static bool finite_point(const geometry_msgs::msg::Point &p)
  {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
  }

  static double planar_distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
  {
    return std::hypot(a.x - b.x, a.y - b.y);
  }

  rclcpp::Time message_time(const builtin_interfaces::msg::Time &stamp) const
  {
    if (stamp.sec == 0 && stamp.nanosec == 0) {
      return now();
    }
    return rclcpp::Time(stamp, RCL_ROS_TIME);
  }

  double age_sec(const builtin_interfaces::msg::Time &stamp) const
  {
    return (now() - message_time(stamp)).seconds();
  }

  bool transform_point(
    const geometry_msgs::msg::Point &input, const std_msgs::msg::Header &header,
    geometry_msgs::msg::Point &output) const
  {
    geometry_msgs::msg::PointStamped in;
    in.header = header;
    in.point = input;
    geometry_msgs::msg::PointStamped out;
    try {
      tf_buffer_.transform(in, out, tracking_frame_, tf2::durationFromSec(transform_timeout_sec_));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Vision TF failed: %s", ex.what());
      return false;
    }
    output = out.point;
    return finite_point(output);
  }

  bool transform_track(
    const RadarTrack &input, const std_msgs::msg::Header &header, RadarObservation &output) const
  {
    if (!transform_point(input.position, header, output.point)) {
      return false;
    }
    geometry_msgs::msg::Vector3Stamped vin;
    vin.header = header;
    vin.vector = input.velocity;
    geometry_msgs::msg::Vector3Stamped vout;
    try {
      tf_buffer_.transform(vin, vout, tracking_frame_, tf2::durationFromSec(transform_timeout_sec_));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Radar velocity TF failed: %s", ex.what());
      return false;
    }
    output.velocity = vout.vector;
    output.acceleration = input.acceleration;
    output.stamp = header.stamp;
    output.frame_number = input_frame_number_;
    output.confidence = input.confidence;
    output.position_variance = input.position_variance;
    return true;
  }

  void vision_callback(const DetectionArray::ConstSharedPtr msg)
  {
    vision_candidates_.clear();
    last_vision_stamp_ = msg->header.stamp;
    for (const auto &detection : msg->detections) {
      if (detection.class_id != person_class_id_ || detection.confidence < min_vision_confidence_ ||
        !detection.depth_valid || !finite_point(detection.position)) {
        continue;
      }
      VisionObservation obs;
      if (!transform_point(detection.position, msg->header, obs.point)) {
        continue;
      }
      obs.stamp = msg->header.stamp;
      obs.confidence = detection.confidence;
      obs.tracking_id = detection.tracking_id;
      vision_candidates_.push_back(obs);
    }
    std::sort(vision_candidates_.begin(), vision_candidates_.end(),
      [](const VisionObservation &a, const VisionObservation &b) { return a.confidence > b.confidence; });
    if (!vision_candidates_.empty()) {
      last_vision_ = vision_candidates_.front();
      have_vision_ = true;
      lost_latched_ = false;
      try_lock_from_current_observations();
    } else {
      have_vision_ = false;
    }
    publish_target();
  }

  void radar_callback(const RadarTrackArray::ConstSharedPtr msg)
  {
    last_radar_stamp_ = msg->header.stamp;
    input_frame_number_ = msg->frame_number;
    const uint64_t token = msg->frame_number != 0 ?
      static_cast<uint64_t>(msg->frame_number) : message_time(msg->header.stamp).nanoseconds();
    if (have_last_radar_token_ && token < last_radar_token_) {
      RCLCPP_WARN(get_logger(), "Radar frame number moved backwards; clearing radar association");
      radar_tracks_.clear();
      association_count_ = 0;
      locked_radar_id_ = -1;
    }
    last_radar_token_ = token;
    have_last_radar_token_ = true;
    radar_tracks_.clear();
    for (const auto &track : msg->tracks) {
      RadarObservation obs;
      if (transform_track(track, msg->header, obs)) {
        radar_tracks_[track.track_id] = obs;
      }
    }
    try_lock_from_current_observations();
    publish_target();
  }

  bool radar_is_fresh(const RadarObservation &obs) const
  {
    const double age = age_sec(obs.stamp);
    return age >= -0.10 && age <= radar_timeout_sec_ && obs.confidence >= radar_min_confidence_ &&
      finite_point(obs.point);
  }

  const VisionObservation *vision_for_radar(uint32_t id) const
  {
    if (!have_vision_ || age_sec(last_vision_stamp_) > vision_timeout_sec_) {
      return nullptr;
    }
    const auto it = radar_tracks_.find(id);
    if (it == radar_tracks_.end()) {
      return nullptr;
    }
    const double dt = std::abs((message_time(last_vision_stamp_) - message_time(it->second.stamp)).seconds());
    if (dt > association_max_time_diff_sec_) {
      return nullptr;
    }
    const VisionObservation *best = nullptr;
    double best_distance = association_max_distance_m_;
    for (const auto &candidate : vision_candidates_) {
      const double distance = planar_distance(candidate.point, it->second.point);
      if (distance <= best_distance) {
        best_distance = distance;
        best = &candidate;
      }
    }
    return best;
  }

  int64_t association_candidate() const
  {
    if (!have_vision_ || vision_candidates_.empty() || age_sec(last_vision_stamp_) > vision_timeout_sec_) {
      return -1;
    }
    double best = std::numeric_limits<double>::max();
    double second = std::numeric_limits<double>::max();
    int64_t best_id = -1;
    for (const auto &pair : radar_tracks_) {
      if (!radar_is_fresh(pair.second)) {
        continue;
      }
      const double dt = std::abs((message_time(last_vision_stamp_) - message_time(pair.second.stamp)).seconds());
      if (dt > association_max_time_diff_sec_) {
        continue;
      }
      const double d = planar_distance(last_vision_.point, pair.second.point);
      if (d < best) {
        second = best;
        best = d;
        best_id = static_cast<int64_t>(pair.first);
      } else if (d < second) {
        second = d;
      }
    }
    if (best_id < 0 || best > association_max_distance_m_ ||
      (second < std::numeric_limits<double>::max() && second - best < association_min_margin_m_)) {
      return -1;
    }
    return best_id;
  }

  void try_lock_from_current_observations()
  {
    if (lost_latched_ || !have_vision_ || radar_tracks_.empty()) {
      return;
    }
    if (locked_radar_id_ >= 0) {
      return;
    }
    const int64_t candidate = association_candidate();
    if (candidate < 0 || !have_last_radar_token_ || (candidate == confirmed_candidate_id_ &&
      last_radar_token_ == confirmed_radar_token_)) {
      return;
    }
    if (candidate == confirmed_candidate_id_) {
      ++association_count_;
    } else {
      confirmed_candidate_id_ = candidate;
      association_count_ = 1;
    }
    confirmed_radar_token_ = last_radar_token_;
    if (association_count_ >= association_confirm_frames_) {
      locked_radar_id_ = candidate;
      RCLCPP_INFO(get_logger(), "Radar track %ld locked after %d distinct frames",
        static_cast<long>(locked_radar_id_), association_count_);
    }
  }

  void timer_callback() { publish_target(); }

  void publish_target()
  {
    FusedTarget output;
    output.header.stamp = now();
    output.header.frame_id = tracking_frame_;
    output.vision_tracking_id = have_vision_ ? last_vision_.tracking_id : -1;
    output.radar_track_id = locked_radar_id_;
    output.pose.pose.orientation.w = 1.0;
    output.confidence = 0.0F;
    output.vision_age_sec = have_vision_ ? static_cast<float>(std::max(0.0, age_sec(last_vision_stamp_))) :
      std::numeric_limits<float>::infinity();
    output.radar_age_sec = std::numeric_limits<float>::infinity();

    const VisionObservation *locked_vision = locked_radar_id_ >= 0 ? vision_for_radar(
      static_cast<uint32_t>(locked_radar_id_)) : nullptr;
    const auto radar_it = locked_radar_id_ >= 0 ? radar_tracks_.find(
      static_cast<uint32_t>(locked_radar_id_)) : radar_tracks_.end();
    const bool radar_ok = radar_it != radar_tracks_.end() && radar_is_fresh(radar_it->second);
    const bool vision_ok = locked_radar_id_ >= 0 ? locked_vision != nullptr :
      (have_vision_ && age_sec(last_vision_stamp_) <= vision_timeout_sec_);

    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Vector3 velocity;
    if (locked_radar_id_ >= 0 && vision_ok && radar_ok) {
      output.tracking_state = FusedTarget::FUSED_LOCKED;
      output.source_mask = FusedTarget::SOURCE_VISION | FusedTarget::SOURCE_RADAR;
      point = locked_vision->point;
      if (use_radar_velocity_) {
        velocity = radar_it->second.velocity;
      }
      output.confidence = static_cast<float>(locked_vision->confidence);
      output.radar_age_sec = static_cast<float>(std::max(0.0, age_sec(radar_it->second.stamp)));
      output.navigation_allowed = true;
    } else if (locked_radar_id_ >= 0 && vision_ok) {
      output.tracking_state = FusedTarget::TRACKING_VISION;
      output.source_mask = FusedTarget::SOURCE_VISION;
      point = locked_vision->point;
      output.confidence = static_cast<float>(locked_vision->confidence);
      output.navigation_allowed = vision_only_navigation_;
    } else if (locked_radar_id_ >= 0 && radar_ok) {
      output.tracking_state = FusedTarget::TRACKING_RADAR;
      output.source_mask = FusedTarget::SOURCE_RADAR;
      point = radar_it->second.point;
      if (use_radar_velocity_) {
        velocity = radar_it->second.velocity;
      }
      output.confidence = radar_it->second.confidence;
      output.radar_age_sec = static_cast<float>(std::max(0.0, age_sec(radar_it->second.stamp)));
      output.navigation_allowed = true;
    } else if (locked_radar_id_ < 0 && vision_ok) {
      output.tracking_state = FusedTarget::VISION_CONFIRMED;
      output.source_mask = FusedTarget::SOURCE_VISION;
      point = last_vision_.point;
      output.confidence = static_cast<float>(last_vision_.confidence);
      output.navigation_allowed = vision_only_navigation_;
    } else {
      const double va = have_vision_ ? age_sec(last_vision_stamp_) : std::numeric_limits<double>::infinity();
      const double ra = radar_it != radar_tracks_.end() ? age_sec(radar_it->second.stamp) :
        (last_radar_stamp_.sec != 0 || last_radar_stamp_.nanosec != 0 ? age_sec(last_radar_stamp_) :
        std::numeric_limits<double>::infinity());
      if (std::min(va, ra) <= sensor_gap_timeout_sec_) {
        output.tracking_state = FusedTarget::SENSOR_GAP;
      } else {
        output.tracking_state = FusedTarget::LOST;
        output.radar_track_id = -1;
        if (locked_radar_id_ >= 0) {
          locked_radar_id_ = -1;
          association_count_ = 0;
          confirmed_candidate_id_ = -1;
          lost_latched_ = true;
        }
      }
      output.source_mask = FusedTarget::SOURCE_NONE;
      output.navigation_allowed = false;
    }
    output.pose.pose.position = point;
    output.velocity.twist.linear = velocity;
    output.pose.covariance[0] = 0.25;
    output.pose.covariance[7] = 0.25;
    output.pose.covariance[14] = 0.50;
    output.velocity.covariance[0] = 1.0;
    output.velocity.covariance[7] = 1.0;
    target_pub_->publish(output);
  }

  std::string vision_topic_, radar_topic_, output_topic_, tracking_frame_;
  int person_class_id_{0};
  int association_confirm_frames_{3};
  double min_vision_confidence_{0.7};
  double association_max_distance_m_{0.8};
  double association_max_time_diff_sec_{0.15};
  double association_min_margin_m_{0.25};
  double vision_timeout_sec_{0.3};
  double radar_timeout_sec_{0.6};
  double sensor_gap_timeout_sec_{0.8};
  double radar_min_confidence_{0.0};
  bool vision_only_navigation_{true};
  bool use_radar_velocity_{false};
  double transform_timeout_sec_{0.08};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<DetectionArray>::SharedPtr vision_sub_;
  rclcpp::Subscription<RadarTrackArray>::SharedPtr radar_sub_;
  rclcpp::Publisher<FusedTarget>::SharedPtr target_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<VisionObservation> vision_candidates_;
  VisionObservation last_vision_;
  bool have_vision_{false};
  builtin_interfaces::msg::Time last_vision_stamp_;
  builtin_interfaces::msg::Time last_radar_stamp_;
  std::map<uint32_t, RadarObservation> radar_tracks_;
  uint32_t input_frame_number_{0};
  uint64_t last_radar_token_{0};
  uint64_t confirmed_radar_token_{0};
  bool have_last_radar_token_{false};
  int64_t confirmed_candidate_id_{-1};
  int association_count_{0};
  int64_t locked_radar_id_{-1};
  bool lost_latched_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PersonTargetFusionNode>());
  rclcpp::shutdown();
  return 0;
}
