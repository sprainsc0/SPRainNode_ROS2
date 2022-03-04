
/**
 * @brief Sensor Combined uORB topic listener example
 * @file sensor_combined_listener.cpp
 * @addtogroup examples
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 * @author Vicente Monge
 */

 #include <rclcpp/rclcpp.hpp>
 #include <sprain_msgs/msg/sensor_delta.hpp>

/**
 * @brief Sensor Combined uORB topic data callback
 */
class SensorDeltaListener : public rclcpp::Node
{
public:
	explicit SensorDeltaListener() : Node("sensor_delta_listener") {
		subscription_ = this->create_subscription<sprain_msgs::msg::SensorDelta>(
			"fmu/sensor_delta/out",
#ifdef ROS_DEFAULT_API
            10,
#endif
			[this](const sprain_msgs::msg::SensorDelta::UniquePtr msg) {
			std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
			std::cout << "RECEIVED SENSOR DELTA DATA"   << std::endl;
			std::cout << "============================="   << std::endl;
			std::cout << "ts: "          << msg->timestamp    << std::endl;
			std::cout << "last_ts: "     << msg->last_update_usec    << std::endl;
			std::cout << "delta_angle[0]: " << msg->delta_angle[0]  << std::endl;
			std::cout << "delta_angle[1]: " << msg->delta_angle[1]  << std::endl;
			std::cout << "delta_angle[2]: " << msg->delta_angle[2]  << std::endl;
			std::cout << "delta_ang_dt: " << msg->delta_ang_dt << std::endl;
			std::cout << "delta_velocity[0]: " << msg->delta_velocity[0] << std::endl;
			std::cout << "delta_velocity[1]: " << msg->delta_velocity[1] << std::endl;
			std::cout << "delta_velocity[2]: " << msg->delta_velocity[2] << std::endl;
			std::cout << "delta_vel_dt: " << msg->delta_vel_dt << std::endl;
			std::cout << "loop_dt: " << msg->loop_dt << std::endl;
			std::cout << "instance: " << msg->instance << std::endl;
			std::cout << "calibrated: " << msg->calibrated << std::endl;
			std::cout << "healthy: " << msg->healthy << std::endl;
			std::cout << "posoffset[0]: " << msg->posoffset[0] << std::endl;
			std::cout << "posoffset[1]: " << msg->posoffset[1] << std::endl;
			std::cout << "posoffset[2]: " << msg->posoffset[2] << std::endl;
		});
	}

private:
	rclcpp::Subscription<sprain_msgs::msg::SensorDelta>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting sensor_delta listener node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SensorDeltaListener>());

	rclcpp::shutdown();
	return 0;
}
