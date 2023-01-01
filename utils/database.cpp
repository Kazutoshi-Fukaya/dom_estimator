#include "utils/database/database.h"

using namespace dom_estimator;

Database::Database() {}

void Database::add_init_object(std::string name,double x,double y)
{
    for(auto it = this->begin(); it != this->end(); it++){
        if(it->first->name == name){
            it->second->add_init_object(x,y);
        }
    }
}

void Database::add_object(std::string name,double x,double y,double time,double credibility)
{
    for(auto it = this->begin(); it != this->end(); it++){
        if(it->first->name == name){
            it->second->add_object(x,y,time,credibility);
        }
    }
}

void Database::update_objects()
{
    for(auto it = this->begin(); it != this->end(); it++){
        it->second->update_obejct();
    }
}

void Database::time_update()
{
    for(auto it = this->begin(); it != this->end(); it++){
        it->second->time_update();   
    }
}

// for debug
void Database::print_contents()
{
    for(auto it = this->begin(); it != this->end(); it++){
        it->second->print_obejcts();
    }
}
