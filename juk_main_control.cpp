#include <iostream>

#include <ros/ros.h>
#include <GeoMath.h>

#include <pthread.h>
#include <unistd.h>



#include <juk_msg/juk_dji_gps_msg.h>
#include <juk_msg/juk_dji_device_status_msg.h>
#include <juk_msg/juk_control_dji_msg.h>
#include <juk_msg/juk_set_target_data_msg.h>
#include <juk_msg/juk_position_data_msg.h>
#include <juk_msg/juk_dji_camera_control_msg.h>
#include <juk_msg/reach_msg.h>

#include <fstream>


using namespace std;
using namespace GeoMath;

class  ShareData 
{
	
public:
	pthread_mutex_t mutex;
	int a;
	int argc;
	char** argv;
	juk_msg::juk_position_data_msg pos_data;
	juk_msg::juk_dji_camera_control_msg cam_msg;
	juk_msg::juk_set_target_data_msg target_msg;
	v3geo last_photo_pos;
	v3geo current_photo_pos;
	double next_photo_dist = 0.5;
	bool enable_camera = true;
	v3 gimbal;
	unsigned long photo_id = 0;
	
	string log_name;
	ofstream log_file;
	
	ros::Publisher *pub_target;
	ros::Publisher *pub_camera;
	
	bool ok;
};

ShareData c;
pthread_t ctrl_thr;


void  pos_callback(const juk_msg::juk_position_data_msg::ConstPtr& in)
{	
	pthread_mutex_lock(&c.mutex);
	c.pos_data = *in;
	pthread_mutex_unlock(&c.mutex);
}

void precision_gps_callback(const juk_msg::reach_msg::ConstPtr& in)
{
	pthread_mutex_lock(&c.mutex);
	if (c.enable_camera)
	{
		c.current_photo_pos = v3geo (in->lat, in->lng, in->alt);
		
		
	}
	pthread_mutex_unlock(&c.mutex);
}

void * ctrl_run(void* context)
{
	ShareData* c=(ShareData*)context;
	
	
	juk_msg::juk_dji_camera_control_msg camera_msg;
	usleep(5000000);
	cout << "START" << endl;
	c->target_msg.data_x = 1;
	c->target_msg.data_y = 1;
	c->target_msg.data_z = 1;
	
	c->target_msg.speed = 1;
	c->target_msg.break_distance_mode = juk_msg::juk_set_target_data_msg::mode_allow_break_distance;
	c->target_msg.system = c->target_msg.system_home;
	c->target_msg.acc = 0.3;
	
	c->target_msg.course = 180;
	
	c->pub_target->publish(c->target_msg);
	
	
}




int main(int argc, char *argv[])
{
	
	if (argc < 2)
	{
		cout << "TO FEW ARGUMENTS" << endl;
		return -1;
	}
	
	c.log_file.open(argv[1]);
	c.log_file.precision(15);
	c.log_file.flags(std::ios::fixed);
	
	
	ros::init(c.argc, c.argv, "JUK_TARGET_CONTROL");
	ros::NodeHandle nh;
	
	ros::Subscriber sub_position_data =	nh.subscribe<juk_msg::juk_position_data_msg>("JUK/POSITION_DATA", 1, pos_callback);
	ros::Subscriber sub_precision_gps =	nh.subscribe<juk_msg::reach_msg>("REACH_EMLID_DATA", 1, precision_gps_callback);
	ros::Publisher pub_target = nh.advertise<juk_msg::juk_set_target_data_msg>("JUK/TARGET", 1);
	ros::Publisher pub_camera = nh.advertise<juk_msg::juk_dji_camera_control_msg>("JUK/DJI_GIMBAL", 1);
	
	c.pub_target = &pub_target;
	c.pub_camera = &pub_camera;
	
	pthread_create(&ctrl_thr, nullptr, ctrl_run, (void*)(&c));
	
	ros::Rate r(10);
	while (ros::ok())
	{	

		if ((c.current_photo_pos - c.last_photo_pos).length_xyz() >= c.next_photo_dist)
		{
			c.last_photo_pos = c.current_photo_pos;
			std::stringstream str;
			str.fill('0');
			str << std::setw(4) << setiosflags(ios::right);
			
			
			str << "DJI_" << c.photo_id << ".JPG";
			c.cam_msg.action = juk_msg::juk_dji_camera_control_msg::take_photo;
			c.log_file <<  str.str() << c.current_photo_pos.lat << ";" << c.current_photo_pos.lng << ";" << c.current_photo_pos.alt << endl;
			c.photo_id++;
			cout << str.str()<<endl;
		}
		
		c.cam_msg.roll = c.gimbal.x;
		c.cam_msg.pitch = 0;
		c.cam_msg.yaw = 0;
		
		c.pub_camera->publish(c.cam_msg);
		c.cam_msg.action = 0;
		
		
		ros::spinOnce();
		r.sleep();
	}	
	return 0;
}