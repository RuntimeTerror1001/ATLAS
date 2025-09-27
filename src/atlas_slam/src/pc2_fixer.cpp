#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <cmath>
#include <algorithm>
#include <cstdint>

class PC2Fixer : public rclcpp::Node {
public:
  PC2Fixer() : Node("pc2_fixer") {
    input_topic_  = declare_parameter<std::string>("input_cloud",  "/world/warehouse/model/atlas_robot/link/robot_base_footprint/sensor/front_laser/scan/points");
    output_topic_ = declare_parameter<std::string>("output_cloud", "/atlas/front_lidar/points");
    min_v_deg_    = declare_parameter<double>("min_v_deg", -15.0);
    max_v_deg_    = declare_parameter<double>("max_v_deg",  15.0);
    rings_        = declare_parameter<int>("vertical_rings", 16);
    scan_rate_    = declare_parameter<double>("scan_rate_hz", 10.0);
    frame_out_    = declare_parameter<std::string>("frame_out", "robot_front_laser_link");

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PC2Fixer::cb, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::SensorDataQoS());
  }

private:
  void cb(const sensor_msgs::msg::PointCloud2::SharedPtr in) {
    const double scan_period = 1.0 / std::max(1e-3, scan_rate_);

    sensor_msgs::msg::PointCloud2 out = *in;
    out.header = in->header;
    if (!frame_out_.empty()) out.header.frame_id = frame_out_;

    out.height = 1;
    out.width  = in->width * in->height;
    out.is_dense = false;
    out.is_bigendian = in->is_bigendian;

    sensor_msgs::PointCloud2Modifier mod(out);
    mod.setPointCloud2Fields(
      5,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "ring", 1, sensor_msgs::msg::PointField::UINT16,
      "time", 1, sensor_msgs::msg::PointField::FLOAT32
    );
    mod.resize(out.width);

    // Input iterators
    sensor_msgs::PointCloud2ConstIterator<float> ix(*in, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(*in, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(*in, "z");

    // Output iterators
    sensor_msgs::PointCloud2Iterator<float> ox(out, "x");
    sensor_msgs::PointCloud2Iterator<float> oy(out, "y");
    sensor_msgs::PointCloud2Iterator<float> oz(out, "z");
    sensor_msgs::PointCloud2Iterator<uint16_t> oring(out, "ring");
    sensor_msgs::PointCloud2Iterator<float> ot(out, "time");

    const double min_v = min_v_deg_ * M_PI / 180.0;
    const double max_v = max_v_deg_ * M_PI / 180.0;
    const double span_v = std::max(1e-6, max_v - min_v);

    for (; ix != ix.end(); ++ix, ++iy, ++iz, ++ox, ++oy, ++oz, ++oring, ++ot) {
      const float x = *ix, y = *iy, z = *iz;
      *ox = x; *oy = y; *oz = z;

      const double rxy = std::sqrt(double(x)*x + double(y)*y);
      const double v_ang = std::atan2(double(z), rxy);
      static const std::array<double,16> VLP16_DEG = {
        -15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0,
        -7.0, 9.0,  -5.0,11.0,  -3.0,13.0, -1.0,15.0
      };
      double v_deg = v_ang * 180.0 / M_PI;
      int ring = std::distance(VLP16_DEG.begin(),
        std::min_element(VLP16_DEG.begin(), VLP16_DEG.end(),
          [&](double a, double b){ return std::abs(v_deg-a) < std::abs(v_deg-b); }));
      *oring = static_cast<uint16_t>(ring);

      double az = std::atan2(double(y), double(x));
      if (az < 0) az += 2.0 * M_PI;
      const float rel_time = static_cast<float>(az / (2.0 * M_PI) * scan_period);
      *ot = rel_time;
    }

    pub_->publish(out);
  }

  std::string input_topic_, output_topic_, frame_out_;
  double min_v_deg_, max_v_deg_, scan_rate_;
  int rings_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PC2Fixer>());
  rclcpp::shutdown();
  return 0;
}
