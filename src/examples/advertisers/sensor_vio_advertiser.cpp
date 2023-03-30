
/**
 * @brief Debug Vect uORB topic adverstiser example
 * @file debug_vect_advertiser.cpp
 * @addtogroup examples
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sprain_msgs/msg/sensor_vio.hpp>

using namespace std::chrono_literals;

class SensorVioAdvertiser : public rclcpp::Node
{
public:
	SensorVioAdvertiser() : Node("sensor_vio_advertiser") {
#ifdef ROS_DEFAULT_API
		publisher_ = this->create_publisher<sprain_msgs::msg::SensorVio>("fmu/sensor_vio/in", 10);
#else
		publisher_ = this->create_publisher<sprain_msgs::msg::SensorVio>("fmu/sensor_vio/in");
#endif
		auto timer_callback =
		[this]()->void {
			auto sensor_vio = sprain_msgs::msg::SensorVio();
			sensor_vio.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
			
			
			RCLCPP_INFO(this->get_logger(), "\033[97m Publishing sensor_vio: time: %llu \033[0m",
                                sensor_vio.timestamp);
			this->publisher_->publish(sensor_vio);
		};
		timer_ = this->create_wall_timer(500ms, timer_callback);
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<sprain_msgs::msg::SensorVio>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting sensor_vio advertiser node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SensorVioAdvertiser>());

	rclcpp::shutdown();
	return 0;
}
