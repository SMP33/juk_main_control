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
	double next_photo_dist = 1;
	bool enable_camera = false;
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

float calc_course(v3geo p1, v3geo p2)
{
	return (p2 - p1).rotateXY(-90*CONST.DEG2RAD).angle_xy(v3(1,0,0))*CONST.RAD2DEG;
}

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
	
	vector<v3geo > route(0);
	
	
	//route.push_back(v3geo(c->pos_data.lat, c->pos_data.lng, c->pos_data.alt+10));
	route.push_back(v3geo(55.8942871094, 37.2508087158, 6));
	route.push_back(v3geo(55.8941917419, 37.2505874634, 6));
	route.push_back(v3geo(55.8940658569, 37.2507438660, 6));
	route.push_back(v3geo(55.8941650391, 37.2509727478, 6));
	
//	for (size_t i = 0; i < route.size(); i++)
//	{
//		cout << "NEXT" << endl;
//		
//		c->target_msg.data_x = route[i].lat;
//		c->target_msg.data_y = route[i].lng;
//		c->target_msg.data_z = 7;
//		
//		c->target_msg.speed = 1;
//		c->target_msg.break_distance_mode = juk_msg::juk_set_target_data_msg::mode_allow_break_distance;
//		c->target_msg.system = juk_msg::juk_set_target_data_msg::system_absolut;
//		c->target_msg.acc = 1;
//		
//		c->target_msg.course = calc_course(v3geo(c->pos_data.lat, c->pos_data.lng, c->pos_data.alt),route[i]);
//	
//		c->pub_target->publish(c->target_msg);
//	
//		usleep(5000000);
//		
//		while (c->pos_data.stable_time < 1)
//			usleep(1000000);
//		
//		
//	}
	c->target_msg.data_x = 10;
	c->target_msg.data_y = 20;
	c->target_msg.data_z = 30;
		
	c->target_msg.speed = 5;
	c->target_msg.break_distance_mode = juk_msg::juk_set_target_data_msg::mode_allow_break_distance;
	c->target_msg.system = juk_msg::juk_set_target_data_msg::system_home;
	c->target_msg.acc = 1;
		
	c->target_msg.course = 0;
	
	cout << "Press ENTER to continue..." << endl;
	char a;
	cin >>  a;
	c->pub_target->publish(c->target_msg);
	cout << "END" << endl;	
}




int main(int argc, char *argv[])
{
	
	if (argc < 2)
	{
		c.log_file.open("last.camera.log");
	}
	else
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
	
	c.enable_camera = false;
	
	ros::Rate r(10);
	while (ros::ok())
	{	

		if ((c.current_photo_pos - c.last_photo_pos).length_xyz() >= c.next_photo_dist && c.enable_camera)
		{
			c.last_photo_pos = c.current_photo_pos;
			std::stringstream str;
			
			str.fill('0');			
			str.width(4);
			str.flags(ios::right);
			
			str << "DJI_" << c.photo_id << ".JPG";
			c.cam_msg.action = juk_msg::juk_dji_camera_control_msg::take_photo;
			c.log_file <<  str.str() << c.current_photo_pos.lat << ";" << c.current_photo_pos.lng << ";" << c.current_photo_pos.alt << endl;
			c.photo_id++;
			cout << str.str()<<endl;
		}
		
		c.cam_msg.roll = c.gimbal.x;
		c.cam_msg.pitch = -270;
		c.cam_msg.yaw = 0;
		
		c.pub_camera->publish(c.cam_msg);
		c.cam_msg.action = 0;
		
		
		ros::spinOnce();
		r.sleep();
	}	
	return 0;
}