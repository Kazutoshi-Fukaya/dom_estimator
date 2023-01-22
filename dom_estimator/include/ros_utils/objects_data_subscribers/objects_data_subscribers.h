#ifndef OBJECTS_DATA_SUBSCRIBERS_H_
#define OBJECTS_DATA_SUBSCRIBERS_H_

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

class ObjectsDataSubscribers : public std::vector<ObjectsDataSubscriber*>
{
public:
    ObjectsDataSubscribers();
    ObjectsDataSubscribers(ros::NodeHandle nh,ros::NodeHandle private_nh,Database* database);

private:
    void init(Database* database);

    // node handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
};
} // namespace dom_estimator

#endif  // OBJECTS_DATA_SUBSCRIBERS_H_
