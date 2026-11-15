#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "CYdLidar.h"
#include <string>
#include <vector>
#include <memory>
#include <cmath>

using namespace ydlidar;

class YDlidarNode : public rclcpp::Node {
public:
    YDlidarNode() : Node("ydlidar_node"), is_running_(false) {
        // Declare and retrieve parameters
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baudrate", 115200);
        this->declare_parameter<std::string>("frame_id", "laser_frame");
        this->declare_parameter<std::string>("scan_topic", "scan");
        this->declare_parameter<bool>("reversion", false);
        this->declare_parameter<bool>("inverted", false);
        this->declare_parameter<bool>("auto_reconnect", true);
        this->declare_parameter<int>("lidar_type", static_cast<int>(TYPE_TRIANGLE));
        this->declare_parameter<int>("device_type", static_cast<int>(YDLIDAR_TYPE_SERIAL));
        this->declare_parameter<int>("sample_rate", 5);
        this->declare_parameter<float>("scan_frequency", 7.0f);
        this->declare_parameter<bool>("single_channel", false);
        this->declare_parameter<bool>("intensity", true);
        this->declare_parameter<float>("range_min", 0.12f);
        this->declare_parameter<float>("range_max", 16.0f);
        this->declare_parameter<float>("angle_min", -M_PI);
        this->declare_parameter<float>("angle_max", M_PI);

        std::string port = this->get_parameter("port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        std::string scan_topic = this->get_parameter("scan_topic").as_string();
        bool reversion = this->get_parameter("reversion").as_bool();
        bool inverted = this->get_parameter("inverted").as_bool();
        bool auto_reconnect = this->get_parameter("auto_reconnect").as_bool();
        int lidar_type = this->get_parameter("lidar_type").as_int();
        int device_type = this->get_parameter("device_type").as_int();
        int sample_rate = this->get_parameter("sample_rate").as_int();
        float scan_frequency = this->get_parameter("scan_frequency").as_double();
        bool single_channel = this->get_parameter("single_channel").as_bool();
        bool intensity = this->get_parameter("intensity").as_bool();
        range_min_ = this->get_parameter("range_min").as_double();
        range_max_ = this->get_parameter("range_max").as_double();
        angle_min_ = this->get_parameter("angle_min").as_double();
        angle_max_ = this->get_parameter("angle_max").as_double();

        RCLCPP_INFO(this->get_logger(), "Initializing YDLIDAR G2 on %s at %d baud...", port.c_str(), baudrate);

        // Configure CYdLidar options
        laser_.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
        laser_.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));
        laser_.setlidaropt(LidarPropLidarType, &lidar_type, sizeof(int));
        laser_.setlidaropt(LidarPropDeviceType, &device_type, sizeof(int));
        laser_.setlidaropt(LidarPropSampleRate, &sample_rate, sizeof(int));
        laser_.setlidaropt(LidarPropScanFrequency, &scan_frequency, sizeof(float));
        laser_.setlidaropt(LidarPropSingleChannel, &single_channel, sizeof(bool));
        laser_.setlidaropt(LidarPropIntenstiy, &intensity, sizeof(bool));
        laser_.setlidaropt(LidarPropReversion, &reversion, sizeof(bool));
        laser_.setlidaropt(LidarPropInverted, &inverted, sizeof(bool));
        laser_.setlidaropt(LidarPropAutoReconnect, &auto_reconnect, sizeof(bool));
        laser_.setlidaropt(LidarPropMinRange, &range_min_, sizeof(float));
        laser_.setlidaropt(LidarPropMaxRange, &range_max_, sizeof(float));
        laser_.setlidaropt(LidarPropMinAngle, &angle_min_, sizeof(float));
        laser_.setlidaropt(LidarPropMaxAngle, &angle_max_, sizeof(float));

        // Create ROS 2 LaserScan publisher
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, rclcpp::SensorDataQoS());

        // Initialize LiDAR
        if (!laser_.initialize()) {
            RCLCPP_ERROR(this->get_logger(), "Fail to initialize YDLIDAR on port %s!", port.c_str());
            return;
        }

        if (!laser_.turnOn()) {
            RCLCPP_ERROR(this->get_logger(), "Fail to start YDLIDAR scanning!");
            laser_.disconnecting();
            return;
        }

        is_running_ = true;
        RCLCPP_INFO(this->get_logger(), "YDLIDAR started successfully! Publishing to /%s", scan_topic.c_str());

        // Start processing thread/timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&YDlidarNode::publishScan, this)
        );
    }

    ~YDlidarNode() override {
        if (is_running_) {
            RCLCPP_INFO(this->get_logger(), "Stopping YDLIDAR...");
            laser_.turnOff();
            laser_.disconnecting();
        }
    }

private:
    void publishScan() {
        if (!is_running_) return;

        LaserScan scan;
        if (laser_.doProcessSimple(scan)) {
            auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
            scan_msg->header.stamp = this->now();
            scan_msg->header.frame_id = frame_id_;

            scan_msg->angle_min = angle_min_;
            scan_msg->angle_max = angle_max_;
            scan_msg->range_min = range_min_;
            scan_msg->range_max = range_max_;

            float scan_time = (scan.scanFreq > 0.0f) ? (1.0f / scan.scanFreq) : 0.1f;
            scan_msg->scan_time = scan_time;

            if (!scan.points.empty()) {
                scan_msg->time_increment = scan_time / static_cast<float>(scan.points.size());
                scan_msg->angle_increment = (angle_max_ - angle_min_) / static_cast<float>(scan.points.size());

                scan_msg->ranges.resize(scan.points.size());
                scan_msg->intensities.resize(scan.points.size());

                for (size_t i = 0; i < scan.points.size(); ++i) {
                    float r = scan.points[i].range;
                    if (r >= range_min_ && r <= range_max_) {
                        scan_msg->ranges[i] = r;
                    } else {
                        scan_msg->ranges[i] = std::numeric_limits<float>::infinity();
                    }
                    scan_msg->intensities[i] = static_cast<float>(scan.points[i].intensity);
                }

                scan_pub_->publish(std::move(scan_msg));
            }
        }
    }

    CYdLidar laser_;
    bool is_running_;
    std::string frame_id_;
    float range_min_;
    float range_max_;
    float angle_min_;
    float angle_max_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YDlidarNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
