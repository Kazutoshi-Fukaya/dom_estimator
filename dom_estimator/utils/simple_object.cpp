#include "utils/database/simple_object.h"

using namespace dom_estimator;

SimpleObject::SimpleObject() :
    has_observed(false),id(-1),time(0.0),start_time(0.0),dom(0.0),x(0.0),y(0.0) {}

SimpleObject::SimpleObject(int _id,double _start_time,double _init_dom,double _x,double _y) :
    has_observed(false),id(_id),time(0.0),start_time(_start_time),dom(_init_dom),x(_x),y(_y) {}

SimpleObject::SimpleObject(bool _has_observed,int _id,double _time,double _start_time,double _dom,double _x,double _y) :
    has_observed(_has_observed),id(_id),time(_time),start_time(_start_time),dom(_dom),x(_x),y(_y) {}

void SimpleObject::update_object(double update_time,double update_x,double update_y)
{
    update_dom(update_time,update_x,update_y);
    time = update_time;
    x = update_x;
    y = update_y;
}

void SimpleObject::update_dom(double update_time,double update_x,double update_y)
{
    double diff_x = x - update_x;
    double diff_y = y - update_y;
    double distance = std::sqrt(diff_x*diff_x + diff_y*diff_y);
    double time_diff = update_time - time;
    double total_time = time - start_time;

    dom = (total_time*dom + distance)/(total_time + time_diff);
}

void SimpleObject::print_object()
{
    std::cout << "ID: " << id << std::endl;
    std::cout << "Time: " << time << std::endl;
    std::cout << "Start Time: " << start_time << std::endl;
    std::cout << "DOM: " << dom << std::endl;
    std::cout << "X: " << x << std::endl;
    std::cout << "Y: " << y << std::endl;
}