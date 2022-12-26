#include "utils/database/objects.h"

using namespace dom_estimator;

Objects::Objects() :
    name(std::string("")), condition(std::string("")), is_static(NULL), init_dom(0.0), distance_th(0.0),
    observations_count(0), spawn_count(0), dom(0.0) {}

Objects::Objects(std::string _name,std::string _condition,double _init_dom,double _distance_th) :
    name(_name), condition(_condition), is_static(str_to_bool(condition)), init_dom(_init_dom), distance_th(_distance_th),
    observations_count(0), spawn_count(0), dom(init_dom) {}

void Objects::add_init_object(double x,double y)
{
    // time = 0.0, credibility = 1.0
    Object object;
    object.add_init_object(x,y);
    this->emplace_back(object);
}

void Objects::print_obejcts()
{
    std::cout << "Name: " << name << std::endl;
    std::cout << "Condition: " << condition << std::endl;
    std::cout << "Difficulty of Moving: " << dom << std::endl;
    for(auto it = this->begin(); it != this->end(); it++){
        it->print_elements();
    }
    std::cout << std::endl;
}

bool Objects::str_to_bool(std::string _condition)
{
    if(_condition == std::string("Static")) return true;
    return false;
}
