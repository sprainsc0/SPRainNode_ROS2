#include "transform_imu.h"

Transform_Imu::Transform_Imu():
	_transform_node(std::make_shared<rclcpp::Node>("transform_node"))
{ }

Transform_Imu::~Transform_Imu()
{
	stop();
}

void Transform_Imu::start()
{
	stop();

	auto spin_node = [this]() {
		rclcpp::spin(_transform_node);
	};

	imu_publisher = _transform_node->create_publisher<sensor_msgs::msg::Imu>("/fmu/imu/out", 10);

	_transform_node_thread.reset(new std::thread(spin_node));
}

void Transform_Imu::stop()
{
	if (_transform_node_thread && _transform_node_thread->joinable()) { 
		_transform_node_thread->join();
	}
}

void Transform_Imu::publish(SensorImu_msg_t *st)
{
	Imu_msg_t imu_msg;

	int64_t  time = st->timestamp() * 1000;

	rclcpp::Time timestamp(time);

	imu_msg.header.stamp = timestamp;

	imu_msg.header.frame_id = "imu_data";

	imu_msg.angular_velocity.x = st->gyro()[0];
	imu_msg.angular_velocity.y = st->gyro()[1];
	imu_msg.angular_velocity.z = st->gyro()[2];

  	imu_msg.linear_acceleration.x = st->accel()[0];
	imu_msg.linear_acceleration.y = st->accel()[1];
	imu_msg.linear_acceleration.z = st->accel()[2];

	imu_publisher->publish(imu_msg);
}
