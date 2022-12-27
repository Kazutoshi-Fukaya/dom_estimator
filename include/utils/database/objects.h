#ifndef OBJECTS_H_
#define OBJECTS_H_

#include "utils/database/object.h"

namespace dom_estimator
{
class Objects : public std::vector<Object>
{
public:
	Objects();
	Objects(std::string name,std::string _condition,double _init_dom,double _distance_th);

	// add
	void add_init_object(double x,double y);
	void add_object(double x,double y,double time,double credibility);

	// for debug
	void print_obejcts();

	// init param
	std::string name;			// Object Name
	std::string condition;		// Static or Dynamic 
    bool is_static; 			// Static or Dynamic
	double init_dom;			// Init Difficulty of Moving
    double distance_th;	    	// if greater than or equal to this value, do not add	

	// param
	int observations_count;	    // Number of observations
	int spawn_count;			// Number of object spawn (kari)
    double dom; 				// Difficulty of Moving

private:
	bool str_to_bool(std::string _condition);

};
} // namespace dom_estimator

#endif  // OBJECTS_H_