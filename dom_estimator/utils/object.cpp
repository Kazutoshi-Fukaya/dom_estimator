#include "utils/database/object.h"

using namespace dom_estimator;

Object::Object() :
    has_observed(false),
    time(0.0), credibility(0.0), x(0.0), y(0.0) {}

void Object::add_init_object(double _x,double _y)
{
    // time = 0.0, credibility = 1.0
    time = 0.0;
    credibility = 1.0;
    x = _x;
    y = _y;
    this->emplace_back(Element(time,credibility,x,y));
}

void Object::add_object(double _x,double _y,double _time,double _credibility)
{
    time = _time;
    x = _x;
    y = _y;
    this->emplace_back(Element(_time,_credibility,_x,_y));
}

void Object::add_element(double _x,double _y,double _time,double _credibility)
{
    time = _time;
    has_observed = true;    // observe object
    this->emplace_back(Element(_time,_credibility,_x,_y));
}

double Object::get_distance(double _x,double _y)
{
    double diff_x = x - _x;
    double diff_y = y - _y;
    return std::sqrt(diff_x*diff_x + diff_y*diff_y);
}

void Object::print_elements()
{
    std::cout << "(X,Y,Time,Credibility): ("
              << x << "," << y << "," << time << "," << credibility << ")" << std::endl;
    for(auto it = this->begin(); it != this->end(); it++){
        it->print_element();
    }
}
