#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"

using namespace std;

ros::Publisher publisherControlSignal;

float P_gain;
float I_gain;
float D_gain;

double lastError;
ros::Time lastErrorTime;
bool hasLastError;

void controllerCallback(const std_msgs::Float64::ConstPtr& error)
{
    double derivative = 0;
    if (hasLastError) {
        double de = error->data - lastError;
        double dt = (ros::Time::now() - lastErrorTime).toSec();
        derivative = D_gain * (de/dt);
    }

    double proportional = error->data * P_gain;
    double controlSignal = proportional + derivative;

    geometry_msgs::Twist twistSignal;
    twistSignal.angular.z = controlSignal;

    publisherControlSignal.publish(twistSignal);

    lastError = error->data;
    lastErrorTime = ros::Time::now();
    hasLastError = true;
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

    if (ros::param::get("~D_gain", D_gain) == false)
    {
        ROS_FATAL("Parameter D_gain not set.");
        return -1;
    }

    hasLastError = false;

    ros::Subscriber subscriberSetPoint = nodeHandle.subscribe("error", 1000, controllerCallback);

    publisherControlSignal = nodeHandle.advertise<geometry_msgs::Twist>("control_signal", 1000);

    ros::spin();

    return 0;
}
