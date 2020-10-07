#ifndef ARC_ROS_TOOL_H
#define ARC_ROS_TOOL_H

#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace arc_ros_tool
{

inline double clock_s()
{
    ros::Time time = ros::Time::now();
    return (double)(time.sec) + (double)time.nsec / 1000000000.0;
}

inline double clock_ms()
{
    return clock_s() * 1000.0;
}

inline int fps()
{
    // https://hk.saowen.com/a/db5be16ca1622e785f138d264295089d4c2b19d2ef63d5afc3820afcce2e9ee8
    static int fps = 0;
    static double lastTime = clock_s(); // s
    static int frameCount = 0;

    ++frameCount;

    double curTime = clock_s();
    if (curTime - lastTime > 1.0)
    {
        fps = frameCount;
        frameCount = 0;
        lastTime = curTime;
    }
    return fps;
}

class arc_transform_listener : public tf::TransformListener
{
public:
    arc_transform_listener()
    {

    }

    bool getPose(std::string iFrameID_Ref, std::string iFrameID, geometry_msgs::Pose &out)
    {
        tf::StampedTransform transform;
        if(this->getTransform(iFrameID_Ref, iFrameID, transform))
        {
            out.position.x = transform.getOrigin().getX();
            out.position.y = transform.getOrigin().getY();
            out.position.z = transform.getOrigin().getZ();

            out.orientation.x = transform.getRotation().getX();
            out.orientation.y = transform.getRotation().getY();
            out.orientation.z = transform.getRotation().getZ();
            out.orientation.w = transform.getRotation().getW();
        }
        else
        {
            return false;
        }
    }

    bool getPose(std::string iFrameID_Ref, geometry_msgs::PoseStamped iPose, geometry_msgs::Pose &out)
    {
        try
        {
            iPose.header.stamp = ros::Time(0);
            geometry_msgs::PoseStamped oPoseStamed;
            this->transformPose(iFrameID_Ref, iPose, oPoseStamed);
            out = oPoseStamed.pose;
            return true;
        }
        catch( tf::TransformException &ex)
        {
            ros::NodeHandle n("~");
            ROS_ERROR("[%s] ex.what() = %s",
                      n.getNamespace().c_str(),
                      ex.what());
        }
        return false;
    }    

    bool getPoseStamped(std::string iFrameID_Ref, geometry_msgs::PoseStamped iPose, double iTimeout, geometry_msgs::PoseStamped &out)
    {
        double check_time = clock_s();
        ros::NodeHandle n("~");
        while(true)
        {
            try
            {
                this->transformPose(iFrameID_Ref, iPose, out);
                //ROS_INFO("%.4f ms", clock_ms() - check_time * 1000.0 );
                return true;

            }
            catch( tf::TransformException &ex)
            {
                //ROS_INFO("[%s] waitting tf", n.getNamespace().c_str());                
                if(clock_s() - check_time > iTimeout)
                {
                    ROS_ERROR("[%s] wait tf (%s -> %s) timeout: (%f)s ex.what() = %s",
                              n.getNamespace().c_str(),
                              iPose.header.frame_id.c_str(),
                              iFrameID_Ref.c_str(),
                              iTimeout,
                              ex.what());
                    break;
                }
            }
            ros::Duration(0.001).sleep();
        }
        return false;
    }

    bool getPoseStamped(std::string iFrameID_Ref, std::string iFrameID, double iTimeout, geometry_msgs::PoseStamped &out )
    {
        double check_time = clock_s();
        ros::NodeHandle n("~");
        ros::Time time_now = ros::Time::now();
        while(true)
        {
            try
            {
                tf::StampedTransform transform;
                this->lookupTransform(iFrameID_Ref, iFrameID, time_now, transform );
                out.header.frame_id = iFrameID_Ref;
                out.header.stamp = time_now;

                out.pose.position.x = transform.getOrigin().getX();
                out.pose.position.y = transform.getOrigin().getY();
                out.pose.position.z = transform.getOrigin().getZ();

                out.pose.orientation.x = transform.getRotation().getX();
                out.pose.orientation.y = transform.getRotation().getY();
                out.pose.orientation.z = transform.getRotation().getZ();
                out.pose.orientation.w = transform.getRotation().getW();

                //ROS_INFO("%.4f ms", clock_ms() - check_time * 1000.0 );
                return true;

            }
            catch( tf::TransformException &ex)
            {
                //ROS_INFO("[%s] waitting tf", n.getNamespace().c_str());
                if(clock_s() - check_time > iTimeout)
                {
                    ROS_ERROR("[%s] wait tf (%s -> %s) timeout: (%f)s ex.what() = %s",
                              n.getNamespace().c_str(),
                              iFrameID.c_str(),
                              iFrameID_Ref.c_str(),
                              iTimeout,
                              ex.what());
                    break;
                }
            }
            ros::Duration(0.001).sleep();
        }
        return false;
    }

    bool getTransform(std::string iFrameID_Ref, std::string iFrameID, tf::StampedTransform &out)
    {
        try
        {
            this->lookupTransform(iFrameID_Ref, iFrameID, ros::Time(0), out );
            return true;

        }
        catch( tf::TransformException &ex)
        {
            ros::NodeHandle n("~");
            ROS_ERROR("[%s] ex.what() = %s",
                      n.getNamespace().c_str(),
                      ex.what());
        }
        return false;
    }

    bool getTransform(std::string iFrameID_Ref, std::string iFrameID, ros::Time iTime, double iTimeout, tf::StampedTransform &out )
    {
        double check_time = clock_s();
        ros::NodeHandle n("~");
        ros::Time time_now = iTime;
        while(true)
        {
            try
            {
                this->lookupTransform(iFrameID_Ref, iFrameID, time_now, out );
                return true;
            }
            catch( tf::TransformException &ex)
            {

                if(clock_s() - check_time > iTimeout)
                {
                    ROS_ERROR("[%s] wait tf (%s -> %s) timeout: (%f)s ex.what() = %s",
                              n.getNamespace().c_str(),
                              iFrameID.c_str(),
                              iFrameID_Ref.c_str(),
                              iTimeout,
                              ex.what());
                    break;
                }
            }
            ros::Duration(0.001).sleep();
        }
        return false;
    }

    bool getTransform(std::string iFrameID_Ref, std::string iFrameID, double iTimeout, tf::StampedTransform &out )
    {
        return this->getTransform(iFrameID_Ref,iFrameID,ros::Time::now(), iTimeout, out);
    }

};

// http://wiki.unity3d.com/index.php/Averaging_Quaternions_and_Vectors
class Vector4
{
public:
    Vector4();
    double x,y,z,w;
};
//Get an average (mean) from more then two quaternions (with two, slerp would be used).
//Note: this only works if all the quaternions are relatively close together.
//Usage:
//-Cumulative is an external Vector4 which holds all the added x y z and w components.
//-newRotation is the next rotation to be added to the average pool
//-firstRotation is the first quaternion of the array to be averaged
//-addAmount holds the total amount of quaternions which are currently added
//This function returns the current average quaternion
tf::Quaternion AverageQuaternion(Vector4 &cumulative, tf::Quaternion newRotation, tf::Quaternion firstRotation, int addAmount);

//Changes the sign of the quaternion components. This is not the same as the inverse.
tf::Quaternion InverseSignQuaternion(tf::Quaternion q);
//Returns true if the two input quaternions are close to each other. This can
//be used to check whether or not one of two quaternions which are supposed to
//be very similar but has its component signs reversed (q has the same rotation as
//-q)
bool AreQuaternionsClose(tf::Quaternion q1, tf::Quaternion q2);
// -------------------------------------------
// 四元數平均
tf::Quaternion AverageQuaternions(std::vector<tf::Quaternion> iRotations);
// 位置平均
tf::Vector3 AveragePositions(std::vector<tf::Vector3> iPositions);
// -------------------------------------------

}




#endif // ARC_ROS_TOOL_H
