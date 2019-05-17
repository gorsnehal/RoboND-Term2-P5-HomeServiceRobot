#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int32.h>

ros::Publisher marker_pub;
int gaction;

/**
 * Simple receipt of messages over the ROS system.
 * Action Message received from 'Pick Object Node. As per message content Marker node displays / hides the Marker at defined location
 */
void actionCallback(const std_msgs::Int32::ConstPtr& msg)
{
  ROS_INFO("I heard: [%d]", msg->data);
  gaction = msg->data;
}

/* Populates Marker message and publishes the same */
bool markerObjects(int goalID, float posX, float posY, int action, int shape)
{
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = action;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = posX;
  marker.pose.position.y = posY;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  };

  marker_pub.publish(marker);
  
  return true;
}

int main( int argc, char** argv )
{
  // Two Goals (PickUp location and Dropoff location)
  const int numOfGoals = 2;
  
  float goalArray[numOfGoals][3] = { {3.5f, 7.0f, 1.0f}, 
                           {-4.0f, 0.5f, 0.5f} };
  
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(2);
  
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
  // Subscriber for goalaction message
  ros::Subscriber sub = n.subscribe("goalaction", 1, actionCallback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  uint32_t action = visualization_msgs::Marker::ADD;

  while(ros::ok())
  {
    // For callback
    ros::spinOnce();
    
    if(gaction == 1)		// 1st Add Marker Call (First PickUp)
    {
      ROS_INFO("Publishing ADD action, iteration: %d\n", 1);
      markerObjects(0, goalArray[0][0], goalArray[0][1], action, shape);
    }
    else if(gaction == 2)	// Delete Marker (First Location Reached)
    {
      action = visualization_msgs::Marker::DELETE;
      ROS_INFO("Publishing ADD action, iteration: %d\n", 1);
      markerObjects(0, goalArray[0][0], goalArray[0][1], action, shape);
    }
    else if(gaction == 3)		// 2nd Add Marker call at Dropoff upon reaching there
    {
      action = visualization_msgs::Marker::ADD;
      ROS_INFO("Publishing ADD action, iteration: %d\n", 2);
      markerObjects(1, goalArray[1][0], goalArray[1][1], action, shape);
    }
    else if(gaction == 0)	// Error - No Action now
    {
      //ROS_INFO("Pick_Objects reported Error\n");
    }

    r.sleep();
  };
}