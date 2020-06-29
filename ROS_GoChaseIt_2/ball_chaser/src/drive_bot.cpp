#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

class SubscribeAndPublish
{
  private:
  ros::NodeHandle n_; 
  ros::Publisher motor_command_pub;
  public:
    SubscribeAndPublish()
    {
      //Topic you want to publish
        motor_command_pub = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    }
    ros::NodeHandle n() { return n_; }


  // This callback function executes whenever a safe_move service is requested
  bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
      ball_chaser::DriveToTarget::Response& res)
  {

      ROS_INFO("Requested vel received - linear_x:%1.2f, angular_z:%1.2f", (float)req.linear_x, (float)req.angular_z);

      // Publish wheel linear and angular velocity
      geometry_msgs::Twist setVel;

      setVel.linear.x = req.linear_x;
      setVel.angular.z = req.angular_z;

      motor_command_pub.publish(setVel);

      // Wait 3 seconds 
      //ros::Duration(3).sleep();

      // Return a response message
      res.msg_feedback = "car velocity set - linear_x: " + std::to_string(req.linear_x) + " , angular_z: " + std::to_string(req.angular_z);
      ROS_INFO_STREAM(res.msg_feedback);

      return true;
  }

};//End of class SubscribeAndPublish

int main(int argc, char** argv)
{
    // Initialize the drive_bot node and create a handle to it
    ros::init(argc, argv, "drive_bot");
  
    //Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish SAPObject;

    // Define a safe_move service with a handle_safe_move_request callback function
    ros::ServiceServer service = SAPObject.n().advertiseService("/ball_chaser/command_robot", &SubscribeAndPublish::handle_drive_request, &SAPObject);
    ROS_INFO("Ready to send velocity commands");

    // Handle ROS communication events
    ros::spin();

    return 0;
}