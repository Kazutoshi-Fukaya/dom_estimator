#include "objects_data_subscribers/objects_data_subscriber.h"

using namespace dom_estimator;

ObjectsDataSubscriber::ObjectsDataSubscriber(){}

ObjectsDataSubscriber::ObjectsDataSubscriber(ros::NodeHandle nh,std::string robot_name,Database* database) :
    nh_(nh), database_(database)
{
    std::string topic_name = robot_name + "/objects_data";
    data_sub_ = nh_.subscribe(topic_name,1,&ObjectsDataSubscriber::data_callback,this);
}

multi_localizer_msgs::ObjectsData ObjectsDataSubscriber::get_data() { return data_; }

void ObjectsDataSubscriber::data_callback(const multi_localizer_msgs::ObjectsDataConstPtr& msg)
{
    data_= *msg;
    for(const auto & data : msg->data){
        database_->add_object(data.name,data.x,data.y,data.time,data.credibility);
    }
}