#include "utils/database/object_with_id.h"

using namespace dom_estimator;

ObjectWithID::ObjectWithID() :
    has_observed(false), total_distance_traveled(0.0),
    id(-1), name(std::string("")), condition(std::string("")), is_static(NULL), init_dom(0.0), distance_th(0.0),
    observations_count(0), appearance_count(0), disappearance_count(0), dom(0.0),last_buffer_time(0.0) {}

ObjectWithID::ObjectWithID(int _id,std::string _name,std::string _condition,double _init_dom,double _distance_th) :
    has_observed(false), total_distance_traveled(0.0),
    id(_id), name(_name), condition(_condition), is_static(str_to_bool(condition)), init_dom(_init_dom), distance_th(_distance_th),
    observations_count(0), appearance_count(0), disappearance_count(0), dom(init_dom),last_buffer_time(0.0) {}

void ObjectWithID::add_init_object(double _x,double _y)
{
    // time = 0.0, credibility = 1.0
    time = 0.0;
    credibility = 1.0;
    x = _x;
    y = _y;
    this->emplace_back(Element(time,credibility,x,y));
}

void ObjectWithID::add_observed_object(double _x, double _y, double _time, double _credibility, double _dom)
{
    time = _time;
    has_observed = true;
    x = _x;
    y = _y;
    credibility = _credibility;
    dom = _dom;
    this->emplace_back(Element(_time,_credibility,_x,_y));
}

// void ObjectWithID::add_object(double _x, double _y, double _time, double _credibility)
// {
//     time = _time;
//     x = _x;
//     y = _y;
//     this->emplace_back(Element(_time,_credibility,_x,_y));
// }

void ObjectWithID::add_object(double _x, double _y, double _time, double _credibility, double _error)
{
    int buffer_max_size = 5;
    double buffer_time = 30.0;

    if(_time - last_buffer_time > buffer_time){
        buffer_elements.clear();
    }
    last_buffer_time = _time;

    buffer_elements.emplace_back(Element(_time,_credibility,_x,_y));
    if(buffer_elements.size() >= buffer_max_size){
        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_credibility = 0.0;
        for(auto it = buffer_elements.begin(); it != buffer_elements.end(); it++){
            sum_x += it->x;
            sum_y += it->y;
            sum_credibility += it->credibility;
        }
        double smoothed_x = sum_x/buffer_elements.size();
        double smoothed_y = sum_y/buffer_elements.size();
        double smoothed_credibility = sum_credibility/buffer_elements.size();
        buffer_elements.clear();

        add_smoothed_object(smoothed_x,smoothed_y,_time,smoothed_credibility,_error);
    } 
}

void ObjectWithID::add_smoothed_object(double _x, double _y, double _time, double _credibility, double _error)
{
    time = _time;
    double diff = get_distance(_x,_y);
    if(!has_observed){
        has_observed = true;
        x = _x;
        y = _y;
        credibility = _credibility;
    }  
    else if(diff > _error){
        x = _x;
        y = _y;
        credibility = _credibility;
        total_distance_traveled += diff;
    }
    else{
        x = (x*credibility + _x*_credibility)/(credibility + _credibility);
        y = (y*credibility + _y*_credibility)/(credibility + _credibility);
        credibility = (credibility*credibility + _credibility*_credibility)/(credibility + _credibility);
    }
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

void ObjectWithID::time_update(double _time)
{
}

// dom evaluation equation
// void ObjectWithID::update_dom(double time)
// {
//     size_t object_size = this->size();

//     // Movement frequency evaluation value
//     double m_value = (double)(appearance_count + disappearance_count)/(time/60.0)/(double)object_size;   

//     // Observation count evaluation value
//     double o_value = (double)(observations_count)/(time/60.0)/(double)object_size;

//     // static object (for dom)
//     if(is_static){
//         // dom = m_value;

//         dom = 1.0/(1.0 + std::exp(-2.0*o_value));   // test
//     }
//     // semi-dynamic object (for dom)
//     else{
//         // dom = m_value;
//         dom = 1.0/(1.0 + std::exp(-(o_value/20.0 - m_value)));    // test

//     }
// }

// void ObjectWithID::update_dom(double _time)
// {
//     // _time: total time(sec)
//     double coef = 1.0;
//     double s = total_distance_traveled/(_time/60.0); // speed
//     dom = 1.0 - std::exp(-s*coef);
// }

void ObjectWithID::update_dom()
{
    double coef = 1.0;
    double s = total_distance_traveled/(time/60.0); // speed
    dom = 1.0 - std::exp(-s*coef);
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
