#include <stdio.h> /* Standard input/output definitions */
#include <string.h> /* String function definitions */
#include <unistd.h> /* UNIX standard function definitions */
#include <fcntl.h> /* File control definitions */
#include <errno.h> /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <unistd.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include "packet.h"
#include "imu_data_decode.h"

#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
//#include<imu_driver/angular_vel.h>
//#include<imu_driver/eul.h>
//#include<imu_driver/linear_acc.h>
//#include<imu_driver/mag.h>
//#include<imu_driver/quaternion.h>
#include "tf/transform_datatypes.h"

#define BAUD         (115200)
#define GRA_ACC      (9.8)
#define DEG_TO_RAD   (0.01745329)



using std::string;
int imu_data_decode_init(void);
typedef void (*on_data_received_event)(packet_t *ptr);
void packet_decode_init(packet_t *pkt, on_data_received_event rx_handler);
uint32_t packet_decode(uint8_t);
void dump_data_packet(receive_imusol_packet_t *data);
static int frame_rate;

static uint8_t buf[2048];
void dump_data_packet(receive_imusol_packet_t *data);
void merge_data(receive_imusol_packet_t *data, sensor_msgs::Imu& imu_data);

int open_port(char *port_device)
{
	struct termios options;

	int fd = open(port_device, O_RDWR | O_NOCTTY);
	
	tcgetattr(fd, &options);

	if (fd == -1)
	{
		perror("open_port: Unable to open SerialPort");
		puts("Please check the usb port name!!!");
		puts("such as \" sudo ./main ttyUSB0 \"");
		exit(0);
	}

	if(fcntl(fd, F_SETFL, 0)<0)
	{
		printf("fcntl failed\n");
	}
	else
	{
		fcntl(fd, F_SETFL, 0);
	}
  
	if(isatty(STDIN_FILENO)==0)
	{
		printf("standard input is not a terminal device\n");
	}
	else 
	{
		printf("isatty success!\n");
	}

	bzero(&options,sizeof(options));

	options.c_cflag = B115200 | CS8 | CLOCAL |CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	options.c_cc[VTIME] = 0;
	options.c_cc[VMIN] = 0;
	tcflush(fd,TCIFLUSH);
	tcsetattr(fd, TCSANOW, &options);
	return (fd);
}

void timer(int sig)
{
	if(SIGALRM == sig)
	{
		frame_rate = frame_count;
		frame_count = 0;
		alarm(1);
	}
}


int main(int argc,  char **argv)
{
	ros::init(argc,  argv, "imu_node");
	ros::NodeHandle nh;
	std::string imu_port;
	nh.getParam("imu_port", imu_port);
	ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 1000);  //note-justin imu_raw imu_data
	//IMU消息发布处
	//ros::Publisher angular_vel_pub= nh.advertise<imu_driver::angular_vel>("angular_vel_data", 10);
	//ros::Publisher eul_pub= nh.advertise<imu_driver::eul>("eul_data", 10);
	//ros::Publisher linear_acc_pub= nh.advertise<imu_driver::linear_acc>("linear_acc_data", 10);
	//ros::Publisher mag_pub= nh.advertise<imu_driver::mag>("mag_data", 10);
	//ros::Publisher qua_pub= nh.advertise<imu_driver::quaternion>("qua_data", 10);
	int fd = 0;
	int i;
	ssize_t n = 0;
	serial::Serial sp;
	serial::Timeout to = serial::Timeout::simpleTimeout(100);
	sp.setPort(imu_port);
	sp.setBaudrate(BAUD);
	sp.setTimeout(to);
	
	imu_data_decode_init();
	signal(SIGALRM,timer);

	try
	{
		sp.open();
	}
	catch(serial::IOException& e)
	{
		std::cout << "imu_node.cpp::Unable to open port： "<< imu_port << std::endl;
		//comment return to test the case that port cannot be opened
		//return -1;
	}
    
	if(sp.isOpen())
	{
		std::cout << "Port is opened: "<< imu_port << std::endl;
	}
	else
	{
		//return -1;
	}
	
	alarm(1);
	ros::Rate loop_rate(100);
	sensor_msgs::Imu imu_data;
	while(ros::ok())
	{
		size_t num = sp.available();
		if(num!=0)
		{
			uint8_t buffer[1024];
			num = sp.read(buffer, num);
			if(num > 0)
			{
				for(int i = 0; i < num; i++)
					packet_decode(buffer[i]);

				if(receive_gwsol.tag != KItemGWSOL)
				{
					merge_data(&receive_imusol, imu_data);
					imu_pub.publish(imu_data);
				}
				else
				{
					for(int i = 0; i < receive_gwsol.n; i++)
					{
						merge_data(&receive_imusol, imu_data);
						imu_pub.publish(imu_data);
					}
				}
			}
		}
		else // if serial port cannot open, null data is published
		{
	
			/*  note-justin  IMU 不应该夹杂去发布数据
			imu_data.header.stamp = ros::Time::now();
			imu_data.header.frame_id = "imu_link";  //note-justin   使用IMU的坐标系
			imu_data.orientation =  tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0.0);
			
			imu_data.angular_velocity.x = 0.0;
			imu_data.angular_velocity.y = 0;
			imu_data.angular_velocity.z = 0;
			
			imu_data.linear_acceleration.x = 0;
			imu_data.linear_acceleration.y = 0;
			imu_data.linear_acceleration.z = 0;
			imu_pub.publish(imu_data);

			*/
		}
		loop_rate.sleep();
	}
	sp.close();
	return 0;
}

void merge_data(receive_imusol_packet_t *data, sensor_msgs::Imu& imu_data)
{

	imu_data.header.stamp = ros::Time::now();
	imu_data.header.frame_id = "imu_link";       //note-justin    使用IMU的坐标系
	imu_data.orientation =  tf::createQuaternionMsgFromRollPitchYaw( data->eul[0] * DEG_TO_RAD,  data->eul[1] * DEG_TO_RAD, data->eul[2] * DEG_TO_RAD);
	
	//或只通过y即绕z的旋转角度计算四元数，用于平面小车。返回四元数：
	//imu_data.orientation =  tf::createQuaternionMsgFromYaw( data->eul[0] * DEG_TO_RAD,  data->eul[1] * DEG_TO_RAD, data->eul[2] * DEG_TO_RAD);
	imu_data.angular_velocity.x = data->gyr[0] * DEG_TO_RAD;
	imu_data.angular_velocity.y = data->gyr[1] * DEG_TO_RAD;
	imu_data.angular_velocity.z = data->gyr[2] * DEG_TO_RAD;

	imu_data.linear_acceleration.x = data->acc[0] * GRA_ACC;
	imu_data.linear_acceleration.y = data->acc[1] * GRA_ACC;
	imu_data.linear_acceleration.z = data->acc[2] * GRA_ACC;
	//std::cout <<"imu data:" <<  imu_data << std::endl;
}
