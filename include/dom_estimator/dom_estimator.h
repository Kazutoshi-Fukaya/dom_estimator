#ifndef DOM_ESTIMATOR_H_
#define DOM_ESTIMATOR_H_

#include <ros/ros.h>

namespace dom_estimator
{
class DomEstimator
{
public:
    DomEstimator();
    void process();

private:
    // node handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;


};
} // namespace dom_estimator

#endif  // DOM_ESTIMATOR_H_
