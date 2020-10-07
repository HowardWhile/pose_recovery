#include "pose_recovery.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>

// -------------------------------------------
Parameters::Parameter *para;
ShareInfomation *info;
// -------------------------------------------

ShareInfomation::ShareInfomation()
{


}

pose_recovery::pose_recovery()
{
    this->task = this->task_initial;
    this->initial();
}

double pose_recovery::clock_s()
{
    ros::Time time = ros::Time::now();
    return (double)(time.sec) + (double)time.nsec / 1000000000.0;
}

void pose_recovery::initial()
{
    para = new Parameters::Parameter();
    info = new ShareInfomation();

    ros::NodeHandle n("~");
    // -------------------------------------------
    // publisher
    // -------------------------------------------
    this->initialpose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);

    // -------------------------------------------
    // subscriber
    // -------------------------------------------
    this->command_sub = n.subscribe<std_msgs::String>("command", 10, &pose_recovery::command_Callback, this);

    // -------------------------------------------
    // tf
    // -------------------------------------------
    this->tf_listener = new arc_ros_tool::arc_transform_listener;
    this->tf_broadcaster = new tf::TransformBroadcaster();

    //

}

void pose_recovery::command_Callback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("[command_Callback] %s", msg->data.c_str());
}

void pose_recovery::p2p(geometry_msgs::PoseStamped iPose, nlohmann::json &oPose)
{
    oPose.clear();

    oPose["header"]["seq"] = iPose.header.seq;
    oPose["header"]["frame_id"] = iPose.header.frame_id;
    oPose["header"]["stamp"]["sec"] = iPose.header.stamp.sec;
    oPose["header"]["stamp"]["nsec"] = iPose.header.stamp.nsec;

    oPose["pose"]["position"]["x"] =  iPose.pose.position.x;
    oPose["pose"]["position"]["y"] =  iPose.pose.position.y;
    oPose["pose"]["position"]["z"] =  iPose.pose.position.z;

    oPose["pose"]["orientation"]["x"] =  iPose.pose.orientation.x;
    oPose["pose"]["orientation"]["y"] =  iPose.pose.orientation.y;
    oPose["pose"]["orientation"]["z"] =  iPose.pose.orientation.z;
    oPose["pose"]["orientation"]["w"] =  iPose.pose.orientation.w;
}

bool pose_recovery::p2p(nlohmann::json iPose, geometry_msgs::PoseStamped &oPose)
{
    try
    {
        oPose.header.seq = iPose["header"]["seq"];
        oPose.header.frame_id = iPose["header"]["frame_id"];
        oPose.header.stamp = ros::Time(iPose["header"]["stamp"]["sec"], iPose["header"]["stamp"]["nsec"]);

        oPose.pose.position.x = iPose["pose"]["position"]["x"];
        oPose.pose.position.y = iPose["pose"]["position"]["y"];
        oPose.pose.position.z = iPose["pose"]["position"]["z"];

        oPose.pose.orientation.x = iPose["pose"]["orientation"]["x"];
        oPose.pose.orientation.y = iPose["pose"]["orientation"]["y"];
        oPose.pose.orientation.z = iPose["pose"]["orientation"]["z"];
        oPose.pose.orientation.w = iPose["pose"]["orientation"]["w"];

        return true;
    }
    catch (...)
    {
        return false;
    }
}

void pose_recovery::Work()
{
    switch(this->task)
    {
    case task_initial:
    {
        para->OnInitial = false;

        geometry_msgs::PoseStamped oPose;
        if(this->tf_listener->getPoseStamped(para->map_frame_id, para->base_frame_id, 2.0, oPose))
        {
            // 收到TF了嘗試恢復位置訊息
            ROS_INFO("received tf msg: %s -> %s",para->map_frame_id.c_str(), para->base_frame_id.c_str() );
            ROS_INFO("try to recover position");
            this->task = task_recovery;
        }
        else
        {
            // 繼續等待tf直到定位系統更新tf
            ROS_INFO_THROTTLE(2.0, "waitting posture tf update.");
        }
    }
        break;

    case task_recovery:
    {
        //先讀檔
        // 先確認檔案存在 https://stackoverflow.com/questions/12774207/fastest-way-to-check-if-a-file-exist-using-standard-c-c11-c
        std::vector<std::string> filenames;
        filenames.resize(para->file_buffer);

        for(int i = 0 ; i < para->file_buffer ; i++)
        {
            filenames[i] = para->file_name + std::to_string(i) + ".json";
        }

        std::vector<geometry_msgs::PoseStamped> pose_list;

        for(int i = 0 ; i < para->file_buffer ; i++)
        {
            std::ifstream ifs(filenames[i]);
            if(ifs.good())
            {
                try
                {
                    // https://stackoverflow.com/questions/33628250/c-reading-a-json-object-from-file-with-nlohmann-json
                    nlohmann::json json_file = nlohmann::json::parse(ifs);
                    geometry_msgs::PoseStamped oPose;
                    if(this->p2p(json_file, oPose) == true)
                    {
                        pose_list.push_back(oPose);
                    }
                }
                catch(...)
                {
                    ROS_ERROR("can not deserialization parameter file: %s", filenames[i].c_str());
                }
            }
        }

        //ROS_INFO_THROTTLE(1.0, "[debug] size() = %d", pose_list.size());
        if(pose_list.size() > 0)
        {
            ROS_INFO("detected recode file.");

            // 找出最新的位置
            ros::Time temp_time;
            int temp_idx = -1;
            for(int i = 0 ; i < (int)pose_list.size() ; i++)
            {
                if(pose_list[i].header.stamp > temp_time)
                {
                    temp_time = pose_list[i].header.stamp;
                    temp_idx = i;
                }
            }

            if(temp_idx >= 0)
            {
                geometry_msgs::PoseWithCovarianceStamped pub_msg;
                geometry_msgs::PoseStamped record_pose = pose_list[temp_idx];
                pub_msg.header = record_pose.header;
                pub_msg.header.stamp = ros::Time::now();

                pub_msg.pose.pose = record_pose.pose;
                pub_msg.pose.covariance[0] = para->covariance_x;
                pub_msg.pose.covariance[7] = para->covariance_y;
                pub_msg.pose.covariance[35] = para->covariance_theta /180.0* M_PI;

                this->initialpose_pub.publish(pub_msg);
                ROS_INFO_STREAM("initial poseture publish: " << pub_msg);
                ros::Duration(2.0).sleep();
            }
        }
        else
        {
            ROS_WARN("There are no recode file, cancel recovery record");

        }
        this->task = task_record;

    }
        break;

    case task_record:
    {
        static double last_record_time = 0.0;//this->clock_s();

        geometry_msgs::PoseStamped now_pose;
        if(this->tf_listener->getPoseStamped(para->map_frame_id, para->base_frame_id, 2.0, now_pose))
        {
            if((this->clock_s() - last_record_time) > para->period)
            {
                //ROS_INFO_STREAM_THROTTLE(1.0,"now_pose:"<<now_pose);

                nlohmann::json j_psoe;
                this->p2p(now_pose, j_psoe);
                //ROS_INFO_STREAM_THROTTLE(1.0,"j_psoe:"<<j_psoe);

                static int count = 0;
                std::string filename = para->file_name + std::to_string(count%para->file_buffer) + ".json";
                count++;

                std::ofstream file;
                file.open(filename);
                std::string json_str = j_psoe.dump(4); // 用排版好的json
                file << json_str;
                file.close();

                last_record_time = this->clock_s();
            }
        }
        else
        {
            // 繼續等待tf直到定位系統更新tf
            //ROS_INFO_THROTTLE(2.0, "waitting posture tf update.");
            this->task = task_initial;

        }

        if(para->OnInitial)
        {

            this->task = task_initial;
        }
    }
        break;

    default:
        ROS_ERROR_THROTTLE(1.0,"unknow task");
        break;
    }
}



