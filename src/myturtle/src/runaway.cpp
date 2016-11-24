#include <ros/ros.h>
#include <turtlesim/Num.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
ros::Publisher velPublishers [5];
int controlNumber = 1;
turtlesim::Pose turtle_pos [5];
const double RADIUS_P2 = 10.0;
const double CORNER_DISTANCE = 2.0;
const double FRAME_SIZE = 11.0;
double distanceP2 (double x1, double y1, double x2, double y2)
{
	return (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1);
}
void runaway(int turtleNumber, turtlesim::Pose target)
{
	turtlesim::Pose source = turtle_pos[turtleNumber-1];

	geometry_msgs::Twist vel_msg;
	vel_msg.linear.x = 1.5;
	bool t = true;
	if(distanceP2(source.x, source.y, target.x, target.y) <= RADIUS_P2)
	{
		if(source.x <= CORNER_DISTANCE)
		{
			t = false;
			if(source.y <= CORNER_DISTANCE)
			{
				if(target.x > target.y)
					vel_msg.angular.z = 4.0 * (atan2(1, 0)  - source.theta);
				else
					vel_msg.angular.z = 4.0 * (atan2(0, 1)  - source.theta);
			}
			else if(source.y >= FRAME_SIZE - CORNER_DISTANCE)
			{
				if(target.x > FRAME_SIZE - target.y)
					vel_msg.angular.z = 4.0 * (atan2(-1, 0)  - source.theta);
				else
					vel_msg.angular.z = 4.0 * (atan2(0, 1)  - source.theta);
			}
		}	
		else if(source.x >= FRAME_SIZE - CORNER_DISTANCE)
		{
			t = false;
			if(source.y <= CORNER_DISTANCE)
			{
				if(FRAME_SIZE - target.x > target.y)
					vel_msg.angular.z = 4.0 * (atan2(1, 0)  - source.theta);
				else
					vel_msg.angular.z = 4.0 * (atan2(0, -1)  - source.theta);
			}
			else if(source.y >= FRAME_SIZE - CORNER_DISTANCE)
			{
				if(/*FRAME_SIZE*/ - target.x > /*FRAME_SIZE*/- target.y)
					vel_msg.angular.z = 4.0 * (atan2(-1, 0)  - source.theta);
				else
					vel_msg.angular.z = 4.0 * (atan2(0, -1)  - source.theta);
			}	
		}
	}
	if(t)	 
	{
    	vel_msg.angular.z = 4.0 * (atan2(source.y - target.y,
    	                            source.x - target.x)  - source.theta);
	}
    velPublishers[turtleNumber-1].publish(vel_msg);
}
void changeControlNumber(const turtlesim::Num::ConstPtr &msg)
{
	controlNumber = msg->num;
	if(controlNumber > 5 || controlNumber <= 0)
		controlNumber = 1;
}
void setPos_turtle1(const turtlesim::Pose::ConstPtr &vel)
{
	turtle_pos[0] = *vel;
	
}
void setPos_turtle2(const turtlesim::Pose::ConstPtr &vel)
{
	turtle_pos[1] = *vel;
}
void setPos_turtle3(const turtlesim::Pose::ConstPtr &vel)
{
	turtle_pos[2] = *vel;
}
void setPos_turtle4(const turtlesim::Pose::ConstPtr &vel)
{
	turtle_pos[3] = *vel;
}
void setPos_turtle5(const turtlesim::Pose::ConstPtr &vel)
{
	turtle_pos[4] = *vel;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "runaway");
	ros::NodeHandle n;

	velPublishers[0] = n.advertise<geometry_msgs::Twist>("myturtle1/cmd_vel", 10);
	velPublishers[1] = n.advertise<geometry_msgs::Twist>("myturtle2/cmd_vel", 10);
	velPublishers[2] = n.advertise<geometry_msgs::Twist>("myturtle3/cmd_vel", 10);
	velPublishers[3] = n.advertise<geometry_msgs::Twist>("myturtle4/cmd_vel", 10);
	velPublishers[4] = n.advertise<geometry_msgs::Twist>("myturtle5/cmd_vel", 10);

	ros::Subscriber sub1 = n.subscribe("myturtle1/pose", 10, setPos_turtle1);
	ros::Subscriber sub2 = n.subscribe("myturtle2/pose", 10, setPos_turtle2);
	ros::Subscriber sub3 = n.subscribe("myturtle3/pose", 10, setPos_turtle3);
	ros::Subscriber sub4 = n.subscribe("myturtle4/pose", 10, setPos_turtle4);
	ros::Subscriber sub5 = n.subscribe("myturtle5/pose", 10, setPos_turtle5);
	ros::Subscriber sub0 = n.subscribe("controlNumber" , 10, changeControlNumber);

	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		for(int i=1; i<=5; i++)
		{
			if(i!=controlNumber)
			{
				runaway(i, turtle_pos[controlNumber-1]);
			}
		}
		ros::spinOnce();

		loop_rate.sleep();
	}


	return 0;
}
