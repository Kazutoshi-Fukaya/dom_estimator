#ifndef SIMPLE_OBJECT_H_
#define SIMPLE_OBJECT_H_

#include <algorithm>
#include <cmath>
#include <iostream>

namespace dom_estimator
{
class SimpleObject
{
public:
    SimpleObject();
    SimpleObject(int _id,double _start_time,double _init_dom,double _x,double _y);
    SimpleObject(bool _has_observed,int _id,double _time,double _start_time,double _dom,double _x,double _y);

    // update
    void update_object(double update_time,double update_x,double update_y);
    void update_dom(double update_time,double update_x,double update_y);

    // for debug
    void print_object();

    // params
    bool has_observed;          // Observed or not
    int id;                     // Object ID
    double time;                // Last Observation Time (sec)
    double start_time;          // First Observation Time (sec)
    double dom;                 // DOM
    double x;                   // Last Observation x-coordinate
    double y;                   // Last Observation y-coordinate

private:
};
} // namespace dom_estimator

#endif  // SIMPLE_OBJECT_H_