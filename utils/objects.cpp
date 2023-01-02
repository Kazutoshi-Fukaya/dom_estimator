#include "utils/database/objects.h"

using namespace dom_estimator;

Objects::Objects() :
    buffer_object_(new BufferObject),
    name(std::string("")), condition(std::string("")), is_static(NULL), init_dom(0.0), distance_th(0.0),
    observations_count(0), appearance_count(0), disappearance_count(0), dom(0.0) {}

Objects::Objects(std::string _name,std::string _condition,double _init_dom,double _distance_th) :
    buffer_object_(new BufferObject),
    name(_name), condition(_condition), is_static(str_to_bool(condition)), init_dom(_init_dom), distance_th(_distance_th),
    observations_count(0), appearance_count(0), disappearance_count(0), dom(init_dom) {}

void Objects::add_init_object(double x,double y)
{
    // time = 0.0, credibility = 1.0
    Object object;
    object.add_init_object(x,y);
    this->emplace_back(object);
}

void Objects::add_object(double x,double y,double time,double credibility)
{
    // print_buffer_objects(); // for debug

    if(this->empty()){
        buffer_object_->add_buffer(time,credibility,x,y);
        return;
    }

    std::vector<double> dist_list;
    dist_list.resize(this->size());
    for(size_t i = 0; i < this->size(); i++){
        dist_list.at(i) = this->at(i).get_distance(x,y);
    }
    size_t min_index = std::distance(dist_list.begin(),std::min_element(dist_list.begin(),dist_list.end()));

    // static object
    if(is_static){
        // Add if within threshold
        if(dist_list.at(min_index) < distance_th){
            this->at(min_index).add_element(x,y,time,credibility);
            dom += 0.05;
        }
    }
    // semi-dynamic object
    else{
        if(dist_list.at(min_index) < distance_th){
            this->at(min_index).add_element(x,y,time,credibility);
        }
        else{
            buffer_object_->add_buffer(time,credibility,x,y);
        }
    }
}

void Objects::update_obejct()
{
    std::vector<Element> elements;
    if(buffer_object_->are_elements_to_add(elements)){
        for(auto &elem : elements){
            appearance_count++;
            Object object;
            object.has_observed = true;
            object.credibility = elem.credibility;
            object.add_object(elem.x,elem.y,elem.time,elem.credibility);
            this->emplace_back(object);
        }
    }
}

void Objects::time_update()
{
    for(auto it = this->begin(); it != this->end();){
        //  no observation and semi-dynamic
        if(!it->has_observed && !is_static){
            disappearance_count++;
            it = this->erase(it);
        }
        else{
            it->has_observed = false;
            it++;
        }
    }
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

void Objects::print_buffer_objects()
{
    std::cout << std::endl
              << "==================== BUFFER DEBUG ====================" << std::endl;
    std::cout << "Name: " << name << std::endl;
    buffer_object_->print_bufffer_elements();
}

bool Objects::str_to_bool(std::string _condition)
{
    if(_condition == std::string("Static")) return true;
    return false;
}
