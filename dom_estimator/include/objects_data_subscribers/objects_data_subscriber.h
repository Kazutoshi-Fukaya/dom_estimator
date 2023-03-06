#ifndef OBJECTS_DATA_SUBSCRIBER_H_
#define OBJECTS_DATA_SUBSCRIBER_H_

#include <ros/ros.h>

#include "utils/database/database.h"
#include "multi_localizer_msgs/ObjectsData.h"

namespace dom_estimator
{
class ObjectsDataSubscriber
{
public:
    ObjectsDataSubscriber();
    ObjectsDataSubscriber(ros::NodeHandle nh,std::string robot_name,Database* database);

    multi_localizer_msgs::ObjectsData get_data();

private:
    void data_callback(const multi_localizer_msgs::ObjectsDataConstPtr& msg);

    // node handler
    ros::NodeHandle nh_;

    // subscriber
    ros::Subscriber data_sub_;

    // database
    Database* database_;

    // buffer
    multi_localizer_msgs::ObjectsData data_;
};
} // namespace dom_estimator

#endif  // OBJECTS_DATA_SUBSCRIBER_H_
