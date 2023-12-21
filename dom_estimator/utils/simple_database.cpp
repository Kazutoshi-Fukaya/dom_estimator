#include "utils/database/simple_database.h"

using namespace dom_estimator;

SimpleDatabase::SimpleDatabase() {}

void SimpleDatabase::add_init_object(int _id,double _start_time,double _init_dom,double _x,double _y)
{
    SimpleObject* object = new SimpleObject(_id,_start_time,_init_dom,_x,_y);
    this->insert(std::make_pair(_id,object));
}

void SimpleDatabase::add_new_object(int _id,double _time,double _x,double _y)
{
    SimpleObject* object = new SimpleObject(true,_id,_time,_time,0.0,_x,_y);
    this->insert(std::make_pair(_id,object));
}

void SimpleDatabase::update_object(int _id,double _time,double _x,double _y)
{
    auto it = this->find(_id);
    if(it != this->end()){
        it->second->update_object(_time,_x,_y);
    } else {
        add_new_object(_id,_time,_x,_y);
    }
}

void SimpleDatabase::print_contents()
{
    for(auto it = this->begin(); it != this->end(); it++){
        it->second->print_object();
    }
}