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

void Transform_Imu::publish(SensorDelta_msg_t *st)
{
	if(st->delta_ang_dt() < 0.001f || st->delta_ang_dt() > 0.05f) {
		return;
	}

	Imu_msg_t imu_msg;

	int64_t  time = st->timestamp() * 1000;

	rclcpp::Time timestamp(time);

	imu_msg.header.stamp = timestamp;

	imu_msg.header.frame_id = "imu_data";

	double gyro_x = st->delta_angle()[0];
	double gyro_y = st->delta_angle()[1];
	double gyro_z = st->delta_angle()[2];

	double accl_x = st->delta_velocity()[0];
	double accl_y = st->delta_velocity()[1];
	double accl_z = st->delta_velocity()[2];

	double ang_dt = st->delta_ang_dt();
	double vel_dt = st->delta_vel_dt();

	imu_msg.angular_velocity.x = gyro_x / ang_dt;
	imu_msg.angular_velocity.y = gyro_y / ang_dt;
	imu_msg.angular_velocity.z = gyro_z / ang_dt;

  	imu_msg.linear_acceleration.x = accl_x / vel_dt;
	imu_msg.linear_acceleration.y = accl_y / vel_dt;
	imu_msg.linear_acceleration.z = accl_z / vel_dt;

	imu_publisher->publish(imu_msg);
}
