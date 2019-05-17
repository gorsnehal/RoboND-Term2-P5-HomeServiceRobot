#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int32.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/* Function to send move_base actions */
bool movebasegoal(int goalID, float goalX, float goalY, float goalW)
{
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = goalX;
  goal.target_pose.pose.position.y = goalY;
  goal.target_pose.pose.orientation.w = goalW;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray, the base reached goal %d\n", goalID);
    return true;
  }
  else
  {
    ROS_INFO("The base failed to reach goal %d for some reason\n", goalID);
    return false;
  }
}

int main(int argc, char** argv)
{
  
  // Two Goals (PickUp location and Dropoff location)
  const int numOfGoals = 2;
  
  float goalArray[numOfGoals][3] = { {3.5f, 7.0f, 1.0f}, 
                           {-4.0f, 0.5f, 0.5f} };
  
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  
  ros::NodeHandle n;
  ros::Publisher action_pub = n.advertise<std_msgs::Int32>("goalaction", 1);
  
  ros::spinOnce();
  
  for (int i = 0; i < numOfGoals; i++) { 
    
    if(i == 0)
    {
      // Publish the marker
      while (action_pub.getNumSubscribers() < 1)
      {
        if (!ros::ok())
        {
          return 0;
        }
        ROS_WARN_ONCE("Please create a subscriber to the action publisher");
        sleep(1);
      };
      
	  // Sending action messages to Add Marker node, which displays / hides markers as per action
	  // sent by this (Pick Object) node
      std_msgs::Int32 msg;
      ROS_INFO("Sending pickup action\n");
      msg.data = 1;	// Add Action 1
      action_pub.publish(msg);
    }
    else
    {
      ROS_INFO("Sending dropoff\n");
    }
    
    bool retStatus = movebasegoal(i, goalArray[i][0], goalArray[i][1], goalArray[i][2]);
    
    std_msgs::Int32 msg;
    
    if(retStatus == true)
    {
      if(i == 0)
      {
        ROS_INFO("Reached pickup %d\n", i);
        ROS_INFO("Sending pickup action DELETE / HIDE");
        msg.data = 2;		// Delete Action
      }
      else
      {
        ROS_INFO("Reached dropoff %d\n", i);
        ROS_INFO("Sending dropoff action ADD");
        msg.data = 3;		// Add Action 2
      }
    }
    else
    {
      ROS_INFO("The base failed to reach goal\n");
      ROS_INFO("Sending goal action ERROR");
      msg.data = 0;		// Error
    }
    
    action_pub.publish(msg);
  
    //Wait for 5 seconds
	sleep(5);
  }

  return 0;
}