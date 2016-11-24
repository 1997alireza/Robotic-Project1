#include <ros/ros.h>
#include <turtlesim/Spawn.h>
#include <string>
int spawnedNum = 1;
void addTurtle(ros::ServiceClient add_turtle_service)
{
	turtlesim::Spawn srv;
	char* numChS;
	std::sprintf(numChS, "%d", spawnedNum);
	std::string numS(numChS);
	srv.request.name = "myturtle" + numS;
	ROS_INFO("myturtle%s spawned!", numS.c_str());
	float x,y,theta;
	switch(spawnedNum){
		case 1:
			x = 5.5;
			y = 5.5;
			theta = 92.5;
			break;
		case 2:
			x = 2.75;
			y = 2.75;
			theta = 183.5;
			break;
		case 3:
			x = 2.75;
			y = 7.75;
			theta = 270.0;
			break;
		case 4:
			x = 7.75;
			y = 2.75;
			theta = 3.7;
			break;
		case 5:
			x = 7.75;
			y = 7.75;
			theta = 45.7;
			break;
	}
	srv.request.x = x;
	srv.request.y = y;
	srv.request.theta = theta;
	add_turtle_service.call(srv);
	spawnedNum++;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "turtlesInit");
	ros::NodeHandle n;
	ros::service::waitForService("spawn");
	ros::ServiceClient add_turtle_service = n.serviceClient<turtlesim::Spawn>("spawn");

	while(spawnedNum < 6)
		addTurtle(add_turtle_service);

	ros::spin();

	
}
