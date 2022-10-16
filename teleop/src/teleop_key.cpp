#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <iostream>


static struct termios old, current;

/* Initialize new terminal i/o settings */
void initTermios(int echo) 
{
  tcgetattr(0, &old); /* grab old terminal i/o settings */
  current = old; /* make new settings same as old settings */
  current.c_lflag &= ~ICANON; /* disable buffered i/o */
  if (echo) {
      current.c_lflag |= ECHO; /* set echo mode */
  } else {
      current.c_lflag &= ~ECHO; /* set no echo mode */
  }
  tcsetattr(0, TCSANOW, &current); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios(void) 
{
  tcsetattr(0, TCSANOW, &old);
}

/* Read 1 character - echo defines echo mode */
char getch_(int echo) 
{
  char ch;
  initTermios(echo);
  ch = getchar();
  resetTermios();
  return ch;
}

/* Read 1 character without echo */
char getch(void) 
{
  return getch_(0);
}

/* Read 1 character with echo */
char getche(void) 
{
  return getch_(1);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_key");

  ros::NodeHandle n;

 
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  ros::Rate loop_rate(60);
  
  int count = 0;
  char key;
  std::string controls="Control Your roomba!\n---------------------------\nMoving around:\n     w\na    s    d\n     x\nw/x : increase/decrease linear velocity \na/d : increase/decrease angular velocity \nspace key, s : force stop\nCTRL-C and ENTER to quit)\n";

  std::cout<<controls;
  float x_linear=0.0;
  float z_angular=0.0;
  while (ros::ok())
  {
   
    geometry_msgs::Twist msg;
    key = getch();
    

    if(key=='w' && x_linear<0.2)
      x_linear+=0.01;
    else if(key=='x' && x_linear>-0.2)
      x_linear-=0.01;
    
    if(key=='a' && z_angular<2.0)
      z_angular+=0.1;
    else if(key=='d' && z_angular>-2.0)
      z_angular-=0.1;
    else if(key=='s')
    {
      x_linear=0.0;
      z_angular=0.0;
    }
    msg.linear.x=x_linear;
    msg.angular.z=z_angular;
  std::cout<<"Current velocity:     linear_velocity: "<<msg.linear.x<<"     Angular_velocity: "<< msg.angular.z<<'\n';

    //ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

