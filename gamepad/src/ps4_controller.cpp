#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

//******NOTE FOR THE TESTING IN DECEMBER: Potential problem areas in the code are marked with the tag *FIXME?*.
//           These are areas I felt unsure about.
class PS4Controller{
public:
  PS4Controller();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int left_vert_axis_, right_horiz_axis_;
  int start_button_;

  double linear_scale_, angular_scale_;

  // keep track to handle joy driver bug
  bool l2_has_changed_;
  bool r2_has_changed_;
  

  ros::Publisher vel_pub_; // velocity publisher node
  ros::Publisher autonomy_cmd_pub_; // autonomy publisher node
  ros::Subscriber joy_sub_; // joy subscriber node
};


PS4Controller::PS4Controller(): // This is an initialization list of the indexes into the axes and buttons arrays
  linear_scale_(1),
  angular_scale_(1),
  left_vert_axis_(1),
  right_horiz_axis_(4), // FIXME?: if this doesn't work, try switching parameter to 3
  start_button_(12)
{


  // get parameters from the parameter server
  // try to get a parameter named arg1, save it in arg2, if that fails use value from arg3
  nh_.param("scale_angular", angular_scale_, angular_scale_);
  nh_.param("scale_linear", linear_scale_, linear_scale_);
  nh_.param("left_vert_axis", left_vert_axis_, left_vert_axis_);
  nh_.param("right_horiz_axis", right_horiz_axis_, right_horiz_axis_); // FIXME?: not sure if "right_horiz_axis" is named correctly
  nh_.param("start_button", start_button_, start_button_);

  // advertise this node to ros
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  extend_pub_ = nh_.advertise<std_msgs::Float32>("/extend_la/cmd", 1); // la = linear actuator
  insert_pub_ = nh_.advertise<std_msgs::Float32>("/insert_la/cmd", 1);
  dumper_pub_ = nh_.advertise<std_msgs::Float32>("/dumper_spin/cmd", 1);
  digger_pub_ = nh_.advertise<std_msgs::Float32>("/digger_spin/cmd", 1);

  // Misc button clicks
  click_start_pub_ = nh_.advertise<std_msgs::Empty>("/click_start_button", 1);
  click_share_pub_ = nh_.advertise<std_msgs::Empty>("/click_share_button", 1);
  click_options_pub_ = nh_.advertise<std_msgs::Empty>("/click_options_button", 1);
    
  // subscribe to the incoming joystick input
  // argument description: (name of topic, number of messages to queue, callback pointer, what object to call that callback on)
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &PS4Controller::joyCallback, this);

}

// Callback method called when this node gets a joy messge
void PS4Controller::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
  l2_has_changed_ = l2_has_changed_ | joy->axes[l2_axis_] != 0.0;
  r2_has_changed_ = r2_has_changed_ | joy->axes[r2_axis_] != 0.0;

  geometry_msgs::Twist twist;
  std_msgs::Empty empty;

  // The linear-x component of the twist is the product of the linear scale factor and the left vertical axis input on joy
  twist.linear.x = linear_scale_ * joy->axes[left_vert_axis_] // has a maximum value of 1

  // The angular-z component of the twist is the product of the angular scale factor and the right horizontal axis input on joy
  twist.angular.z = angular_scale_ * joy->axes[right_horiz_axis_]); // has a maximum value of 1


  // insert = tri forward, x backward
  std_msgs::Float32 insert_cmd;
  insert_cmd.data = (joy->buttons[triangle_button_] - joy->buttons[x_button_]);
  insert_pub_.publish(insert_cmd);
  // extend = r1 down, l1 up
  std_msgs::Float32 extend_cmd;
  extend_cmd.data = (joy->buttons[r1_button_] - joy->buttons[l1_button_]);
  extend_pub_.publish(extend_cmd);
  // dumper = dpad up, dpad down
  std_msgs::Float32 dumper_cmd;
  dumper_cmd.data = (joy->axes[dpad_vert_axis_]);
  dumper_pub_.publish(dumper_cmd);

  // spin digger = r2 normal digging (CCW), l2 reverse digging (CW)
  // check if we have seen anything but 0 from these 2 ports.  if not, then there value is +1
  std_msgs::Float32 digger_cmd;
  float r2 = -(-1 + joy->axes[r2_axis_]) / 2.0; // convert [1,-1] to (0,1)
  float l2 = -(-1 + joy->axes[l2_axis_]) / 2.0; // convert [1,-1] to (0,1)
  // if these haven't changed, the joy driver initializes them incorrectly, so we need to do this check
  if (!l2_has_changed_) {
	  l2 = 0.0;
  }
  if (!r2_has_changed_) {
	  r2 = 0.0;
  }

  digger_cmd.data = r2 - l2;
  digger_pub_.publish(digger_cmd);

  // misc button clicks
  if(joy->buttons[start_button_]){
    click_start_pub_.publish(empty);
  }
  if(joy->buttons[share_button_]){
    click_share_pub_.publish(empty);
  }
  if(joy->buttons[options_button_]){
    click_options_pub_.publish(empty);
  }
  
}


int main(int argc, char** argv){
  ros::init(argc, argv, "ps4_controller");
  PS4Controller ps4_controller;

  // wait for and incoming joy message to interpret
  ros::spin();
}
