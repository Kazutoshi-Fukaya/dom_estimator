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

// for debug
void Database::print_contents()
{
	for(auto it = this->begin(); it != this->end(); it++){
		it->second->print_obejcts();
	}
}