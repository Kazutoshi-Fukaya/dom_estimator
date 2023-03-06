#ifndef OBJECTS_DATA_SUBSCRIBERS_H_
#define OBJECTS_DATA_SUBSCRIBERS_H_

#include "objects_data_subscribers/objects_data_subscriber.h"

namespace dom_estimator
{
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
