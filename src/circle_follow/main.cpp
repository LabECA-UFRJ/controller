#include <string>

#include "ros/ros.h"

#include "controller_msgs/Circle.h"

#include "simulation_msgs/PoseRobotArray.h"

#include "std_msgs/Float64.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Twist.h"

#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"

class CircleFollowController
{
public:
    CircleFollowController() : _nh(""), _tfListener(_tfBuffer)
    {
        _twistSignalPub = _nh.advertise<geometry_msgs::Twist>("twist_signal", 10);
        _circleReferenceSub = _nh.subscribe("circle_reference", 1, &CircleFollowController::UpdateReference, this);

        _rho = _nh.advertise<std_msgs::Float64>("rho", 10);
        _alpha = _nh.advertise<std_msgs::Float64>("alpha", 10);
        _eta = _nh.advertise<std_msgs::Float64>("eta", 10);

        SetupParameters();
    }

    void UpdateReference(const controller_msgs::Circle::ConstPtr &circle)
    {
        _circle.x = circle->x;
        _circle.y = circle->y;
        _circle.r = circle->r;
    }

    void Spin()
    {
        WaitTransformAvailable();

        ros::Rate rate(60.0);
        while (_nh.ok())
        {
            auto transform = GetTransform();

            auto rho = ComputeRho(transform, _circle);
            auto alpha = ComputeAlpha(transform, _circle);
            auto eta = ComputeEta(transform, _circle);

            // Proportional controller
            auto v = k1 * rho;
            auto omega = k2 * ComputeSigmoid(rho) * alpha + k3 * eta;

            // Constant velocity on top of trajectory
            v += (1 - ComputeSigmoid(rho)) * U;

            geometry_msgs::Twist controllerSignal;
            controllerSignal.linear.x = v;
            controllerSignal.angular.z = omega;
            _twistSignalPub.publish(controllerSignal);

            std_msgs::Float64 foo;

            foo.data = rho;
            _rho.publish(foo);

            foo.data = alpha;
            _alpha.publish(foo);

            foo.data = eta;
            _eta.publish(foo);

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    double ComputeSigmoid(double x)
    {
        return 1.0 / (1.0 + exp(-c1 * (x - c2)));
    }

    double ComputeRho(const geometry_msgs::Transform &robot, const controller_msgs::Circle &circle)
    {
        // Robot position
        auto xr = robot.translation.x;
        auto yr = robot.translation.y;

        // Circle position
        auto xc = circle.x;
        auto yc = circle.y;

        // Delta
        auto dx = xr - xc;
        auto dy = yr - yc;

        auto angle = atan2(dy, dx);

        // Goal
        auto xg = xc + circle.r * cos(angle);
        auto yg = yc + circle.r * sin(angle);

        return sqrt(pow(xr - xg, 2) + pow(yr - yg, 2));
    }

    double ComputeAlpha(const geometry_msgs::Transform &robot, const controller_msgs::Circle &circle)
    {
        tf2::Quaternion q(
            robot.rotation.x,
            robot.rotation.y,
            robot.rotation.z,
            robot.rotation.w);

        tf2::Vector3 v(1.0f, 0, 0);

        v = quatRotate(q, v);

        double theta_r = atan2(v.y(), v.x());

        // Robot position
        auto xr = robot.translation.x;
        auto yr = robot.translation.y;

        // Circle position
        auto xc = circle.x;
        auto yc = circle.y;

        // Delta
        auto dx = xr - xc;
        auto dy = yr - yc;

        auto angle = atan2(dy, dx);

        // Goal
        auto xg = xc + circle.r * cos(angle);
        auto yg = yc + circle.r * sin(angle);

        auto rotation = atan2(yg - yr, xg - xr) - theta_r;

        if (rotation > M_PI)
        {
            rotation = -(2 * M_PI - rotation);
        }
        else if (rotation < -M_PI)
        {
            rotation = rotation + 2 * M_PI;
        }

        return rotation;
        //return atan2(yg - yr, xg - xr) - theta_r;
        //return theta_r - atan2(yr - yg, xr - xg);
    }

    double ComputeEta(const geometry_msgs::Transform &robot, const controller_msgs::Circle &circle)
    {
        // Robot position
        auto xr = robot.translation.x;
        auto yr = robot.translation.y;

        // Circle position
        auto xc = circle.x;
        auto yc = circle.y;

        // Delta
        auto dx = xr - xc;
        auto dy = yr - yc;

        auto theta_g = atan2(dy, dx) + M_PI_2;

        tf2::Quaternion q(
            robot.rotation.x,
            robot.rotation.y,
            robot.rotation.z,
            robot.rotation.w);

        tf2::Vector3 v(1.0f, 0, 0);

        v = quatRotate(q, v);

        double theta_r = atan2(v.y(), v.x());

        double rotation = theta_g - theta_r;
        if (rotation > M_PI)
        {
            rotation = -(2 * M_PI - rotation);
        }
        else if (rotation < -M_PI)
        {
            rotation = rotation + 2 * M_PI;
        }

        return rotation;
    }

    geometry_msgs::Transform GetTransform()
    {
        try
        {
            geometry_msgs::TransformStamped transformStamped;
            transformStamped = _tfBuffer.lookupTransform(ref_frame, child_frame, ros::Time(0));

            geometry_msgs::Transform transform;
            transform.translation = transformStamped.transform.translation;
            transform.rotation = transformStamped.transform.rotation;

            return transform;
        }
        catch (tf2::TransformException &e)
        {
            ROS_FATAL("%s", e.what());
            exit(-1);
        }
    }

    void WaitTransformAvailable()
    {
        while (!_tfBuffer.canTransform(child_frame, ref_frame, ros::Time(0), ros::Duration(1.0)))
        {
            ros::Duration(1.0).sleep();
            continue;
        }
    }

    void SetupParameters()
    {
        try
        {
            ref_frame = GetParam<std::string>("~ref_frame");
            child_frame = GetParam<std::string>("~child_frame");

            k1 = GetParam<double>("~k1");
            k2 = GetParam<double>("~k2");
            k3 = GetParam<double>("~k3");

            c1 = GetParam<double>("~c1");
            c2 = GetParam<double>("~c2");

            U = GetParam<double>("~U");
        }
        catch (std::runtime_error &e)
        {
            ROS_FATAL("%s", e.what());
            exit(-1);
        }
    }

    template <typename ParamType>
    ParamType GetParam(const char *name)
    {
        ParamType param;
        if (ros::param::get(name, param) == false)
        {
            std::stringstream ss;
            ss << "Parameter " << name << " not set";
            throw std::runtime_error(ss.str());
        }

        return param;
    }

    ros::NodeHandle _nh;

    ros::Publisher _twistSignalPub;
    ros::Publisher _rho;
    ros::Publisher _alpha;
    ros::Publisher _eta;
    ros::Subscriber _circleReferenceSub;

    std::string ref_frame, child_frame;

    double k1, k2, k3; // Controller gain
    double c1, c2;     // Sigmoid parameters
    double U;          // Constant speed when correctly on top of trajectory

    controller_msgs::Circle _circle;

    tf2_ros::Buffer _tfBuffer;
    tf2_ros::TransformListener _tfListener;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circle_follow_controller");

    CircleFollowController node;

    node.Spin();

    return 0;
}