#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>

//testing comments

class PS4Controller{
public:
  PS4Controller();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  // axes
  int left_vert_axis_, right_vert_axis_;
  int pad_vert_axis, pad_hoz_axis;
  int l2_axis_, r2_axis_;
  // buttons
  int start_button_, share_button, options_button;
  int square_button_, x_button_, circle_button_, triangle_button_;
  int l1_button, r1_button_;
  double linear_scale_, angular_scale_;
  
  ros::Publisher vel_pub_;
  ros::Publisher autonomy_cmd_pub_;
  ros::Subscriber joy_sub_;
};


PS4Controller::PS4Controller(): //This is an initialization list of the indexes into the axes and buttons arrays
  linear_scale_(1.0),
  angular_scale_(1.0),
  left_vert_axis_(1),
  right_vert_axis_(5),
  pad_hoz_axis_(6),
  pad_vert_axis_(7),
  l2_axis_(3),
  r2_axis_(4),
  share_button_(8),
  options_button_(9),
  square_button_(0),
  x_button_button_(1),
  circle_button_(2),
  triangle_button_(3),
  l1_button_(4),
  r1_button_(5)
{

  //get parameters from the parameter server
  //try to get a parameter named arg1, save it in arg2, if that fails use value from arg3
  nh_.param("scale_angular", angular_scale_, angular_scale_);
  nh_.param("scale_linear", linear_scale_, linear_scale_);
  nh_.param("left_vert_axis", left_vert_axis_, left_vert_axis_);
  nh_.param("right_vert_axis", right_vert_axis_, right_vert_axis_);
  nh_.param("start_button", start_button_, start_button_);
  nh_.param("circle_button", circle_button_, circle_button_);
  nh_.param("x_button", x_button_, x_button_);
  nh_.param("square_button", square_button_, square_button_);
  nh_.param("triangle_button", triangle_button_, triangle_button_);
  nh_.param("l1_button", l1_button_, l1_button_);
  nh_.param("r1_button", r1_button_, r1_button_);
  nh_.param("pad_vert_axis", pad_vert_axis_, pad_vert_axis_);
  nh_.param("pad_hoz_axis", pad_hoz_axis_, pad_hoz_axis_);
  nh_.param("l2_axis", l2_axis_, l2_axis_);
  nh_.param("r2_axis", r2_axis_, r2_axis_);
  nh_.param("share_button", share_button_, share_button_);
  nh_.param("options_button", options_button_, options_button_);

  //advertise this node to ros
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  autonomy_cmd_pub_ = nh_.advertise<std_msgs::Empty>("/click_select_button", 10);

  // TODO: how to publish to roboclaw and pololu? Just individual for each of the things? probably

  //subscribe to the incoming joystick input
  //argument description: (name of topic, number of messages to queue, callback pointer, what object to call that callback on)
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &PS4Controller::joyCallback, this);

}

// Callback method called when this node gets a joy messge
void PS4Controller::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
  geometry_msgs::Twist twist;
  std_msgs::Empty empty;

  twist.linear.x = linear_scale_ * (joy->axes[left_vert_axis_] + joy->axes[right_vert_axis_]); // has a maximum value of 2
  twist.angular.z = angular_scale_ * (joy->axes[left_vert_axis_] - joy->axes[right_vert_axis_]); // has a maximum value of 2

  if(joy->buttons[start_button_]){
    autonomy_cmd_pub_.publish(empty);
  }
  
  vel_pub_.publish(twist);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "ps4_controller");
  PS4Controller ps4_controller;

  //wait for and incoming joy message to interpret
  ros::spin();
}
