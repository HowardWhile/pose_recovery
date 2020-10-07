#ifndef POSE_RECOVERY_H
#define POSE_RECOVERY_H

#include "arc_ros_tool.hpp"

#include "json.hpp"

// generate by cfg file
#include <pose_recovery_pkg/pose_recovery_parametersConfig.h>

// ros toppic
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>

// ros api
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

// opencv api
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// c++ boost
#include <boost/filesystem.hpp>

// c++ STL
#include <string>
#include <fstream>


#define PARAMETERS_CONFIG_CLASS pose_recovery_pkg::pose_recovery_parametersConfig

namespace Parameters
{

class ParameterConfig : public PARAMETERS_CONFIG_CLASS
{
public:
    ParameterConfig()
    {
        this->ini_parameter_table();
    }
    /* -------------------------------------- */
    // getValue
    // 用來取得 dynamic_reconfigure/Config.msg 的數值
    nlohmann::json getValue(dynamic_reconfigure::Config iConfig, std::string iParamName)
    {
        nlohmann::json rValue;
        rValue.clear();

        std::string oValue_str;
        int oValue_int;
        double oValue_double;
        bool oValue_bool;

        if(this->getValue(iConfig, iParamName, oValue_str))
        {
            rValue = oValue_str;
            return rValue;
        }
        if(this->getValue(iConfig, iParamName, oValue_int))
        {
            rValue = oValue_int;
            return rValue;
        }
        if(this->getValue(iConfig, iParamName, oValue_double))
        {
            rValue = oValue_double;
            return rValue;
        }
        if(this->getValue(iConfig, iParamName, oValue_bool))
        {
            rValue = oValue_bool;
            return rValue;
        }
        return rValue;
    }
    // 使用變數名稱 來取得PARAMETERS_CONFIG_CLASS 的數值
    nlohmann::json getValue(std::string iParamName)
    {
        nlohmann::json rValue;
        rValue.clear();

        int id = this->parameters_table[iParamName]["id"]; // 取得這個變數的index是多少

        std::vector<PARAMETERS_CONFIG_CLASS::AbstractParamDescriptionConstPtr> parameters_des = this->__getParamDescriptions__();

        boost::any oValue;
        parameters_des[id]->getValue(*this, oValue);
        std::string value_type = parameters_des[id]->type;

        if(value_type == "str") rValue = boost::any_cast<std::string>(oValue);
        if(value_type == "int") rValue = boost::any_cast<int>(oValue);
        if(value_type == "double") rValue = boost::any_cast<double>(oValue);
        if(value_type == "bool") rValue = boost::any_cast<bool>(oValue);

        return rValue;
    }
    // setValue
    int setValue(std::string iParamName, nlohmann::json iValue)
    {
        if(this->parameters_table.count(iParamName)) // 具有這個參數
        {
            dynamic_reconfigure::Config cfg;

            dynamic_reconfigure::BoolParameter value_bool;
            dynamic_reconfigure::IntParameter value_int;
            dynamic_reconfigure::StrParameter value_str;
            dynamic_reconfigure::DoubleParameter value_double;

            if(iValue.is_boolean())
            {
                value_bool.name = iParamName;
                value_bool.value = iValue;
                cfg.bools.push_back(value_bool);
            }

            if(iValue.is_number_integer())
            {
                value_int.name = iParamName;
                value_int.value = iValue;
                cfg.ints.push_back(value_int);
            }

            if(iValue.is_string())
            {
                value_str.name = iParamName;
                value_str.value = iValue;
                cfg.strs.push_back(value_str);
            }

            if(iValue.is_number_float())
            {
                value_double.name = iParamName;
                value_double.value = iValue;
                cfg.doubles.push_back(value_double);
            }

            return this->__fromMessage__(cfg);
        }

        return 0; // 寫參數失敗
    }
    /* -------------------------------------- */
    // 參數的列表
    std::vector<std::string> parameters_name;
    nlohmann::json parameters_table;


private:
    // create table for parameter name -> type, default value
    void ini_parameter_table(void)
    {
        this->parameters_name.clear();
        this->parameters_table.clear();

        // http://docs.ros.org/jade/api/dynamic_reconfigure/html/msg/ConfigDescription.html
        dynamic_reconfigure::ConfigDescription cfg_des = this->__getDescriptionMessage__(); // 描述

        int groups_size = cfg_des.groups.size();
        //ROS_INFO("[cfg param info] groub number: %d", groups_size);
        if( groups_size == 1)
        {
            dynamic_reconfigure::Group cfg_group = cfg_des.groups[0];
            std::vector<dynamic_reconfigure::ParamDescription> parameters_des = cfg_group.parameters;

            int param_num = parameters_des.size();
            //ROS_INFO("[cfg param info] parameter number: %d", param_num);

            for (int idx_param = 0; idx_param < param_num; ++idx_param)
            {
                dynamic_reconfigure::ParamDescription cfg_param_info = parameters_des[idx_param];
                std::string param_name = cfg_param_info.name;
                std::string param_type = cfg_param_info.type;

                /*ROS_INFO("[cfg param info] %d: parameter [name: %s] [type: %s]",
                     idx_param,
                     param_name.c_str(),
                     param_type.c_str());//*/

                // 產生參數名稱的列表
                this->parameters_name.push_back(param_name);

                // 建立以參數名稱索引的描述列表
                this->parameters_table[param_name]["type"] = param_type;

                // 建立以參數名稱索引的預設值
                nlohmann::json default_value = this->getValue(cfg_des.dflt, param_name);

                if( default_value.size() > 0 )
                {
                    this->parameters_table[param_name]["default"] = default_value;
                }
                else
                {
                    ROS_WARN("can not find default value. parameter name:(%s)", param_name.c_str());
                }

                // 參數的編號
                this->parameters_table[param_name]["id"] = idx_param;

            }
            //ROS_INFO_STREAM("[debug]" << std::endl << parameters_table.dump(4));
        }
        else
        {
            ROS_WARN("cfg_des.groups.size() is %d.", groups_size);
        }
    }

    int getValue(dynamic_reconfigure::Config iConfig, std::string iParamName, bool &oValue)
    {
        int objs_size = iConfig.bools.size();
        if(objs_size > 0)
        {
            dynamic_reconfigure::BoolParameter* objs = &iConfig.bools[0];
            for (int i = 0; i < objs_size; ++i)
            {
                if(objs[i].name == iParamName)
                {
                    oValue = objs[i].value;
                    return 1;
                }
            }
        }
        return 0;
    }
    int getValue(dynamic_reconfigure::Config iConfig, std::string iParamName, int &oValue)
    {
        int objs_size = iConfig.ints.size();
        if(objs_size > 0)
        {
            dynamic_reconfigure::IntParameter* objs = &iConfig.ints[0];
            for (int i = 0; i < objs_size; ++i)
            {
                if(objs[i].name == iParamName)
                {
                    oValue = objs[i].value;
                    return 1;
                }
            }
        }
        return 0;
    }
    int getValue(dynamic_reconfigure::Config iConfig, std::string iParamName, double &oValue)
    {
        int objs_size = iConfig.doubles.size();
        if(objs_size > 0)
        {
            dynamic_reconfigure::DoubleParameter* objs = &iConfig.doubles[0];
            for (int i = 0; i < objs_size; ++i)
            {
                if(objs[i].name == iParamName)
                {
                    oValue = objs[i].value;
                    return 1;
                }
            }
        }
        return 0;
    }
    int getValue(dynamic_reconfigure::Config iConfig, std::string iParamName, std::string &oValue)
    {
        int objs_size = iConfig.strs.size();
        if(objs_size > 0)
        {
            dynamic_reconfigure::StrParameter* objs = &iConfig.strs[0];
            for (int i = 0; i < objs_size; ++i)
            {
                if(objs[i].name == iParamName)
                {
                    oValue = objs[i].value;
                    return 1;
                }
            }
        }
        return 0;
    }
};

class Parameter : public ParameterConfig
{
public:
    Parameter()
    {
        ros::NodeHandle n("~");

        // 參數檔位置
        n.param<std::string>("param_file_path", this->param_file_path, "pose_recovery_node.json");

        if(this->load_parameter_file() == 0)
        {
            ROS_WARN("can not load parameter file (%s). Use the default initialization parameter values", this->param_file_path.c_str() );
            this->load_default();
        }

        this->update_to_server();
        this->ini_parameter = *this; // 保存一下初始值
        this->is_first_cfg = true;

        this->reconfig_server = new dynamic_reconfigure::Server<PARAMETERS_CONFIG_CLASS>(ros::NodeHandle("~"));
        dynamic_reconfigure::Server<PARAMETERS_CONFIG_CLASS>::CallbackType cb = boost::bind(&Parameters::Parameter::reconfig_callback, this, _1, _2);
        this->reconfig_server->setCallback(cb);


        /*ROS_INFO("[%s][parameters_table]", n.getNamespace().c_str());
        // foreach json object, https://github.com/nlohmann/json/blob/develop/doc/examples/iterator_wrapper.cpp
        for (auto& x : this->parameters_table.items())
        {
            ROS_INFO_STREAM( "key: " << x.key() << ", value: " << x.value());
        } //*/
    }

    std::string param_file_path;

private:
    // 將參數數值還原成cfg裡定義的預設值
    void load_default()
    {
        ros::NodeHandle n("~");

        for (int idx_param = 0; idx_param < this->parameters_name.size(); ++idx_param)
        {
            std::string name = this->parameters_name[idx_param];
            std::string type = this->parameters_table[name]["type"];
            nlohmann::json value = this->parameters_table[name]["default"];

            ROS_INFO_STREAM( n.getNamespace() << " [load default] " << name << ": " << value );

            if(type == "str") this->setValue(name, (std::string)value );
            if(type == "int") this->setValue(name, (int)value );
            if(type == "double") this->setValue(name, (double)value );
            if(type == "bool") this->setValue(name, (bool)value );
        }
    }
    // 將參數更新到parameter serevr
    void update_to_server()
    {
        ros::NodeHandle n("~");
        for (int idx_param = 0; idx_param < this->parameters_name.size(); ++idx_param)
        {
            std::string name = this->parameters_name[idx_param];
            std::string type = this->parameters_table[name]["type"];
            nlohmann::json data = this->getValue(name);

            // ROS_INFO_STREAM( n.getNamespace() << " [update parameter to server] " << name << ": " << data );

            if(type == "str") n.setParam(name, (std::string)data );
            if(type == "int") n.setParam(name, (int)data) ;
            if(type == "double") n.setParam(name, (double)data );
            if(type == "bool") n.setParam(name, (bool)data );
        }
    }
    // 將參數數值保存到檔案
    void save_parameter_file()
    {
        ROS_INFO_STREAM("save parameter file to " << this->param_file_path);

        nlohmann::json parameters;
        for (int idx_param = 0; idx_param < this->parameters_name.size(); ++idx_param)
        {
            std::string name = this->parameters_name[idx_param];
            nlohmann::json data = this->getValue(name);
            parameters[name] = data;
        }        

        try
        {

            // 在建立檔案前先確認路徑存在
            // https://stackoverflow.com/questions/18682148/how-to-create-directories-automatically-using-ofstream
            // https://stackoverflow.com/questions/675039/how-can-i-create-directory-tree-in-c-linux
            // https://stackoverflow.com/questions/3071665/getting-a-directory-name-from-a-filename
            boost::filesystem::path dir(this->param_file_path);

            if(this->param_file_path != dir.filename())
            {
                // 檢查檔案路徑是路徑 + 檔名，還是只有檔名
                if(!(boost::filesystem::exists(dir.parent_path())))
                {
                    ROS_WARN("file path (%s) does not exist.", dir.parent_path().c_str());
                    if (boost::filesystem::create_directory(dir.parent_path()))
                        ROS_WARN("....Successfully Created !");
                }
            }



            std::ofstream file;
            file.open(this->param_file_path);
            std::string json_str = parameters.dump(4); // 用排版好的json
            file << json_str;
            file.close();
        }
        catch (std::exception ex)
        {
            ROS_ERROR("can not save file to %s [%s]",
                      this->param_file_path.c_str(),
                      ex.what());
        }
    }
    // 從檔案讀取參數
    int load_parameter_file()
    {
        // 先確認檔案存在 https://stackoverflow.com/questions/12774207/fastest-way-to-check-if-a-file-exist-using-standard-c-c11-c
        std::ifstream ifs(this->param_file_path);
        if(ifs.good())
        {
            try
            {
                // https://stackoverflow.com/questions/33628250/c-reading-a-json-object-from-file-with-nlohmann-json
                nlohmann::json json_file = nlohmann::json::parse(ifs);

                ros::NodeHandle n("~");
                for (int idx_param = 0; idx_param < this->parameters_name.size(); ++idx_param)
                {
                    std::string name = this->parameters_name[idx_param];
                    std::string type = this->parameters_table[name]["type"];

                    nlohmann::json value = json_file[name];

                    ROS_INFO_STREAM( n.getNamespace() << " [load parameter file] " << name << ": " << value );

                    if(type == "str") this->setValue(name, (std::string)value );
                    if(type == "int") this->setValue(name, (int)value );
                    if(type == "double") this->setValue(name, (double)value );
                    if(type == "bool") this->setValue(name, (bool)value );
                }

                return 1;
            }
            catch(...)
            {
                ROS_ERROR("can not deserialization parameter file: %s", this->param_file_path.c_str());

                // https://stackoverflow.com/questions/2602013/read-whole-ascii-file-into-c-stdstring
                // 顯示參數檔的內容
                std::string file_data;
                ifs.seekg(0, std::ios::end);
                file_data.reserve(ifs.tellg());
                ifs.seekg(0, std::ios::beg);
                file_data.assign((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

                ROS_ERROR_STREAM(file_data);
            }
        }
        return 0;
    }
    /* -------------------------------------------- */
    // reconfig 會用到的成員
    /* -------------------------------------------- */
    ParameterConfig ini_parameter; // 保存系統初始化時的參數 reset會讀回來
    bool is_first_cfg;
    dynamic_reconfigure::Server<PARAMETERS_CONFIG_CLASS>* reconfig_server; // server
    // callback
    void reconfig_callback(PARAMETERS_CONFIG_CLASS &config, uint32_t level)
    {
        if(this->is_first_cfg)
        {
            this->is_first_cfg = false;
            config = *this; // 第一次 reconfig callback 就傳遞當前的參數出去即可
        }
        else
        {
            if(config.initial)
            {
                OnInitial = true;
                config.initial = false;
            }

            // 必須要有 save 和 reset 這兩個是基礎功能
            if(config.reset) // reset
            {
                (*(PARAMETERS_CONFIG_CLASS*)this) = this->ini_parameter; // 將啟動時的參數讀回
                config = *this; // 回報給reconfig
                config.reset = false;
            }

            if(config.save) // save
            {
                config.save = false;
                (*(PARAMETERS_CONFIG_CLASS*)this)  = config;
                this->save_parameter_file();
            }

            (*(PARAMETERS_CONFIG_CLASS*)this)  = config; // 不是reset 和save 就直接將數值更新就好

            // ...
        }
    }
    /* -------------------------------------------- */
public:
    bool OnInitial = false;
};
}

class ShareInfomation
{
public:
    ShareInfomation();


};


class pose_recovery
{
public:
    pose_recovery();

    void Work();
    double clock_s();
    void initial();

    typedef enum
    {
        task_initial = 0,
        task_recovery,
        task_record

    }te_task;
private:
    // -------------------------------------------
    // publisher
    // -------------------------------------------
    //ros::Publisher visualize_pub;
    ros::Publisher initialpose_pub;
    // -------------------------------------------
    // subscriber
    // -------------------------------------------
    ros::Subscriber command_sub;
    void command_Callback(const std_msgs::String::ConstPtr &msg);
    // -------------------------------------------
    // tf
    // -------------------------------------------
    arc_ros_tool::arc_transform_listener* tf_listener;
    tf::TransformBroadcaster* tf_broadcaster;
    // -------------------------------------------

    void p2p(geometry_msgs::PoseStamped iPose, nlohmann::json &oPose);
    bool p2p(nlohmann::json iPose, geometry_msgs::PoseStamped &oPose);

    te_task task;
};

#endif // POSE_RECOVERY_H
