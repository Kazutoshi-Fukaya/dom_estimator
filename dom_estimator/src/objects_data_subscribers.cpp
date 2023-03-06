#include "objects_data_subscribers/objects_data_subscribers.h"

using namespace dom_estimator;

ObjectsDataSubscribers::ObjectsDataSubscribers() {}

ObjectsDataSubscribers::ObjectsDataSubscribers(ros::NodeHandle nh,ros::NodeHandle private_nh,Database* database) :
    nh_(nh), private_nh_(private_nh) { init(database); }

void ObjectsDataSubscribers::init(Database* database)
{
    this->clear();
    std::string robot_element_list_name;
    private_nh_.param("ROBOT_ELEMENT_LIST",robot_element_list_name,{std::string("robot_element_list")});
    XmlRpc::XmlRpcValue robot_element_list;
    if(!private_nh_.getParam(robot_element_list_name.c_str(),robot_element_list)){
        ROS_ERROR("Cloud not load %s", robot_element_list_name.c_str());
        return;
    }

    ROS_ASSERT(robot_element_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    this->resize(robot_element_list.size());
    for(int i = 0; i < (int)robot_element_list.size(); i++){
        if(!robot_element_list[i]["robot_name"].valid() ||
           !robot_element_list[i]["color"].valid()){
            ROS_ERROR("%s is valid", robot_element_list_name.c_str());
            return;
        }
        if(robot_element_list[i]["robot_name"].getType() == XmlRpc::XmlRpcValue::TypeString &&
           robot_element_list[i]["color"].getType() == XmlRpc::XmlRpcValue::TypeString){
            std::string robot_name = static_cast<std::string>(robot_element_list[i]["robot_name"]);
            std::string color = static_cast<std::string>(robot_element_list[i]["color"]);
            this->at(i) = new ObjectsDataSubscriber(nh_,robot_name,database);
        }
    }
}
