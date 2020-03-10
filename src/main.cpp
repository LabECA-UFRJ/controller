#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"

using namespace std;

ros::Publisher publisherControlSignal;

float P_gain;

void controllerCallback(const std_msgs::Float64::ConstPtr& error)
{
    // Proportional control.
    float controlSignal = error->data * P_gain;

    geometry_msgs::Twist twistSignal;
    twistSignal.angular.z = controlSignal;

    publisherControlSignal.publish(twistSignal);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle nodeHandle;

    if (ros::param::get("~P_gain", P_gain) == false)
    {
        ROS_FATAL("Parameter P_gain not set.");
        return -1;
    }

    ros::Subscriber subscriberSetPoint = nodeHandle.subscribe("error", 1000, controllerCallback);

    publisherControlSignal = nodeHandle.advertise<geometry_msgs::Twist>("control_signal", 1000);

    ros::spin();

    return 0;
}
