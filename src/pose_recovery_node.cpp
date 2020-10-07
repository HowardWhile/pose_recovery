#include "pose_recovery.h"


int main(int argc, char **argv)
{    
    ros::init(argc, argv, "pose_recovery_node");
    ros::NodeHandle n("~");
    ros::Rate r(100);
    pose_recovery s;


    while ( ros::ok() )
    {
        //ROS_INFO_THROTTLE(1.0, "[%s] Hello World", n.getNamespace().c_str());

        s.Work();

        ros::spinOnce();
        r.sleep();
    }
}
