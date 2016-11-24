#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Num.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <string>

#define KEYCODE_D 0x64 
#define KEYCODE_A 0x61
#define KEYCODE_W 0x77
#define KEYCODE_S 0x73
#define KEYCODE_TAB 0x09
int turtleNumber = 1;
class TeleopTurtle
{
public:
  TeleopTurtle();
  void keyLoop();

private:

  
  ros::NodeHandle nh_;
  double linear_, angular_, l_scale_, a_scale_;
  ros::Publisher twist_pub_s [5];
  ros::Publisher controlNumberPub;
  
};

TeleopTurtle::TeleopTurtle():
  linear_(0),
  angular_(0),
  l_scale_(2.0),
  a_scale_(2.0)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  twist_pub_s [0] = nh_.advertise<geometry_msgs::Twist>("myturtle1/cmd_vel", 1);
  twist_pub_s [1] = nh_.advertise<geometry_msgs::Twist>("myturtle2/cmd_vel", 1);
  twist_pub_s [2] = nh_.advertise<geometry_msgs::Twist>("myturtle3/cmd_vel", 1);
  twist_pub_s [3] = nh_.advertise<geometry_msgs::Twist>("myturtle4/cmd_vel", 1);
  twist_pub_s [4] = nh_.advertise<geometry_msgs::Twist>("myturtle5/cmd_vel", 1);
  controlNumberPub = nh_.advertise<turtlesim::Num>("controlNumber", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  signal(SIGINT,quit);

  teleop_turtle.keyLoop();
  
  return(0);
}


void TeleopTurtle::keyLoop()
{
  char c;
  bool dirty=false;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle.");


  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);

    bool isMove = true;
  
    switch(c)
    {
      case KEYCODE_A:
        ROS_DEBUG("LEFT");
        angular_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("RIGHT");
        angular_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_W:
        ROS_DEBUG("UP");
        linear_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_S:
        ROS_DEBUG("DOWN");
        linear_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_TAB:
        ROS_DEBUG("TAB");
        turtleNumber = (turtleNumber) % 5 + 1;

        char* numChS;
        std::sprintf(numChS, "%d", turtleNumber);
        std::string numS(numChS);
	      puts("tab => control changed !");

        isMove = false;

        turtlesim::Num num_msg;
        num_msg.num = turtleNumber;
        controlNumberPub.publish(num_msg);
        break;
    }
  
    if(isMove)
    {
     geometry_msgs::Twist twist;
     twist.angular.z = a_scale_*angular_;
     twist.linear.x = l_scale_*linear_;
      if(dirty ==true)
      {
       twist_pub_s [turtleNumber-1].publish(twist);    
       dirty=false;
      }
    }

  }


  return;
}



