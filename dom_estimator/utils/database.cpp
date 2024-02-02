#include "utils/database/database.h"

using namespace dom_estimator;

Database::Database() {}

// void Database::add_init_object(std::string name,double x,double y)
// {
//     for(auto it = this->begin(); it != this->end(); it++){
//         if(it->first->name == name){
//             it->second->add_init_object(x,y);
//         }
//     }
// }

// void Database::add_object(std::string name,double x,double y,double time,double credibility)
// {
//     for(auto it = this->begin(); it != this->end(); it++){
//         if(it->first->name == name){
//             it->second->add_object(x,y,time,credibility);
//         }
//     }
// }

void Database::add_init_object(int id,double x,double y)
{
    for(auto it = this->begin(); it != this->end(); it++){
        if(it->second->id == id){
            it->second->add_init_object(x,y);
        }
    }
}

void Database::add_observed_object(int id,double x,double y,double time,double credibility,double dom)
{
    for(auto it = this->begin(); it != this->end(); it++){
        if(it->second->id == id){
            it->second->add_observed_object(x,y,time,credibility,dom);
        }
    }
}

// void Database::add_object(int id,double x,double y,double time,double credibility)
// {
//     for(auto it = this->begin(); it != this->end(); it++){
//         if(it->second->id == id){
//             it->second->add_object(x,y,time,credibility);
//         }
//     }
// }

void Database::add_object(int id,double x,double y,double time,double credibility,double error)
{
    for(auto it = this->begin(); it != this->end(); it++){
        if(it->second->id == id){
            it->second->add_object(x,y,time,credibility,error);
        }
    }
}

void Database::update_objects()
{
    for(auto it = this->begin(); it != this->end(); it++){
        it->second->update_obejct();
    }
}

// void Database::time_update()
// {
//     for(auto it = this->begin(); it != this->end(); it++){
//         it->second->time_update();   
//     }
// }

void Database::time_update(double time)
{
    for(auto it = this->begin(); it != this->end(); it++){
        it->second->time_update(time);
    }
}

void Database::update_dom(double time)
{
    for(auto it = this->begin(); it != this->end(); it++){
        it->second->update_dom(time);
    }
}

// for debug
void Database::print_contents()
{
    for(auto it = this->begin(); it != this->end(); it++){
        // it->second->print_obejcts();
        it->second->print_elements();
    }
}
