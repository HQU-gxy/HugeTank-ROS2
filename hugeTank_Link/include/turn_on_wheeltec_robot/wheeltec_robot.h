
#ifndef __WHEELTEC_ROBOT_H_
#define __WHEELTEC_ROBOT_H_

#include <iostream>
#include <string.h>
#include <string>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <rcl/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdbool.h>

#include <sys/types.h>

#include <serial/serial.h>

#include <tf2_ros/transform_broadcaster.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>
// #include "wheeltec_robot_msg/msg/data.hpp"
// #include "wheeltec_robot_msg/msg/supersonic.hpp"

using namespace std;

#define RESET string("\033[0m")
#define RED "\033[31m"
#define GREEN "\033[32m"
#define YELLOW "\033[33m"
#define BLUE "\033[34m"
#define PURPLE "\033[35m"
#define CYAN "\033[36m"

// Macro definition
// 宏定义
#define SEND_DATA_CHECK 1	 // Send data check flag bits //发送数据校验标志位
#define READ_DATA_CHECK 0	 // Receive data to check flag bits //接收数据校验标志位
#define FRAME_HEADER 0X7B	 // Frame head //帧头
#define FRAME_TAIL 0X7D		 // Frame tail //帧尾
#define RECEIVE_DATA_SIZE 24 // The length of the data sent by the lower computer //下位机发送过来的数据的长度
#define SEND_DATA_SIZE 11	 // The length of data sent by ROS to the lower machine //ROS向下位机发送的数据的长度
#define PI 3.1415926f		 // PI //圆周率

// Relative to the range set by the IMU gyroscope, the range is ±500°, corresponding data range is ±32768
// The gyroscope raw data is converted in radian (rad) units, 1/65.5/57.30=0.00026644
// 与IMU陀螺仪设置的量程有关，量程±500°，对应数据范围±32768
// 陀螺仪原始数据转换位弧度(rad)单位，1/65.5/57.30=0.00026644
#define GYROSCOPE_RATIO 0.00026644f
// Relates to the range set by the IMU accelerometer, range is ±2g, corresponding data range is ±32768
// Accelerometer original data conversion bit m/s^2 units, 32768/2g=32768/19.6=1671.84
// 与IMU加速度计设置的量程有关，量程±2g，对应数据范围±32768
// 加速度计原始数据转换位m/s^2单位，32768/2g=32768/19.6=1671.84
#define ACCEl_RATIO 1671.84f

extern sensor_msgs::msg::Imu Mpu6050; // External variables, IMU topic data //外部变量，IMU话题数据

// Covariance matrix for speedometer topic data for robt_pose_ekf feature pack
// 协方差矩阵，用于里程计话题数据，用于robt_pose_ekf功能包
const double odom_pose_covariance[36] = {1e-3, 0, 0, 0, 0, 0,
										 0, 1e-3, 0, 0, 0, 0,
										 0, 0, 1e6, 0, 0, 0,
										 0, 0, 0, 1e6, 0, 0,
										 0, 0, 0, 0, 1e6, 0,
										 0, 0, 0, 0, 0, 1e3};

const double odom_pose_covariance2[36] = {1e-9, 0, 0, 0, 0, 0,
										  0, 1e-3, 1e-9, 0, 0, 0,
										  0, 0, 1e6, 0, 0, 0,
										  0, 0, 0, 1e6, 0, 0,
										  0, 0, 0, 0, 1e6, 0,
										  0, 0, 0, 0, 0, 1e-9};

const double odom_twist_covariance[36] = {1e-3, 0, 0, 0, 0, 0,
										  0, 1e-3, 0, 0, 0, 0,
										  0, 0, 1e6, 0, 0, 0,
										  0, 0, 0, 1e6, 0, 0,
										  0, 0, 0, 0, 1e6, 0,
										  0, 0, 0, 0, 0, 1e3};

const double odom_twist_covariance2[36] = {1e-9, 0, 0, 0, 0, 0,
										   0, 1e-3, 1e-9, 0, 0, 0,
										   0, 0, 1e6, 0, 0, 0,
										   0, 0, 0, 1e6, 0, 0,
										   0, 0, 0, 0, 1e6, 0,
										   0, 0, 0, 0, 0, 1e-9};

// Data structure for speed and position
// 速度、位置数据结构体
typedef struct __Vel_Pos_Data_
{
	float X;
	float Y;
	float Z;
} Vel_Pos_Data;

struct __attribute__((packed)) UpLinkCommand
{
	uint8_t header = 0x7b;
	float targetLinear;	 // Linear speed in m/s
	float targetAngular; // Angular speed in rad/s
	uint8_t checksum;
};

// The robot chassis class uses constructors to initialize data, publish topics, etc
// 机器人底盘类，使用构造函数初始化数据和发布话题等
class turn_on_robot : public rclcpp::Node
{
public:
	turn_on_robot();		   // Constructor //构造函数
	~turn_on_robot();		   // Destructor //析构函数
	void Control();			   // Loop control code //循环控制代码
	serial::Serial tankSerial; // Declare a serial object //声明串口对象
private:
	// ros::NodeHandle n;           //Create a ROS node handle //创建ROS节点句柄
	rclcpp::Time _Now, _Last_Time; // Time dependent, used for integration to find displacement (mileage) //时间相关，用于积分求位移(里程)
	float Sampling_Time;		   // Sampling time, used for integration to find displacement (mileage) //采样时间，用于积分求位移(里程)

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Cmd_Vel_Sub; // Initialize the topic subscriber //初始化话题订阅者

	// Initialize the topic publisher //初始化话题发布者
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
	// rclcpp::Publisher<wheeltec_robot_msg::msg::Supersonic>::SharedPtr distance_publisher;

	// The speed topic subscribes to the callback function
	// 速度话题订阅回调函数
	void Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux);

	void Publish_Odom();	  // Pub the speedometer topic //发布里程计话题
	void Publish_ImuSensor(); // Pub the IMU sensor topic //发布IMU传感器话题
	// 从串口(ttyUSB)读取运动底盘速度、IMU、电源电压数据
	// Read motion chassis speed, IMU, power supply voltage data from serial port (ttyUSB)
	bool Get_Sensor_Data();

	std::unique_ptr<UpLinkCommand> buildCommand(float linear, float angular);

	string usart_port_name,
		robot_frame_id, gyro_frame_id, odom_frame_id; // Define the related variables //定义相关变量
	int serial_baud_rate;							  // Serial communication baud rate //串口通信波特率
	Vel_Pos_Data Robot_Pos;							  // The position of the robot //机器人的位置
	Vel_Pos_Data Robot_Vel;							  // The speed of the robot //机器人的速度

	struct IMUData
	{
		float accelX;
		float accelY;
		float accelZ;
		float gyroX;
		float gyroY;
		float gyroZ;
	};
};
#endif
