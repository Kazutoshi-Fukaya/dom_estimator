#include "utils/database/object_with_id.h"

using namespace dom_estimator;

ObjectWithID::ObjectWithID() :
    id(-1), name(std::string("")), condition(std::string("")), is_static(NULL), init_dom(0.0), distance_th(0.0),
    observations_count(0), appearance_count(0), disappearance_count(0), dom(0.0) {}

ObjectWithID::ObjectWithID(int _id,std::string _name,std::string _condition,double _init_dom,double _distance_th) :
    id(_id), name(_name), condition(_condition), is_static(str_to_bool(condition)), init_dom(_init_dom), distance_th(_distance_th),
    observations_count(0), appearance_count(0), disappearance_count(0), dom(init_dom) {}

void ObjectWithID::add_init_object(double _x,double _y)
{
    // time = 0.0, credibility = 1.0
    time = 0.0;
    credibility = 1.0;
    x = _x;
    y = _y;
    this->emplace_back(Element(time,credibility,x,y));
}

void ObjectWithID::add_object(double _x, double _y, double _time, double _credibility)
{
    time = _time;
    x = _x;
    y = _y;
    this->emplace_back(Element(_time,_credibility,_x,_y));
}

void ObjectWithID::add_element(double _x, double _y, double _time, double _credibility)
{
    time = _time;
    has_observed = true;
    this->emplace_back(Element(_time,_credibility,_x,_y));
}

void ObjectWithID::update_obejct()
{
    // TODO
}

void ObjectWithID::time_update()
{
    // TODO
}

// dom evaluation equation
void ObjectWithID::update_dom(double time)
{
    size_t object_size = this->size();

    // Movement frequency evaluation value
    double m_value = (double)(appearance_count + disappearance_count)/(time/60.0)/(double)object_size;   

    // Observation count evaluation value
    double o_value = (double)(observations_count)/(time/60.0)/(double)object_size;

    // static object (for dom)
    if(is_static){
        // dom = m_value;

        dom = 1.0/(1.0 + std::exp(-2.0*o_value));   // test
    }
    // semi-dynamic object (for dom)
    else{
        // dom = m_value;
        dom = 1.0/(1.0 + std::exp(-(o_value/20.0 - m_value)));    // test

    }
}

double ObjectWithID::get_distance(double _x,double _y)
{
    double diff_x = x - _x;
    double diff_y = y - _y;
    return std::sqrt(diff_x*diff_x + diff_y*diff_y);
}

void ObjectWithID::print_elements()
{
    std::cout << "(X,Y,Time,Credibility): ("
              << x << "," << y << "," << time << "," << credibility << ")" << std::endl;
    for(auto it = this->begin(); it != this->end(); it++){
        it->print_element();
    }
}

bool ObjectWithID::str_to_bool(std::string _condition)
{
    if(_condition == std::string("Static")) return true;
    return false;
}
