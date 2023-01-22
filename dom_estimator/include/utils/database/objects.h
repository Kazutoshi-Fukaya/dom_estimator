#ifndef OBJECTS_H_
#define OBJECTS_H_

#include "utils/database/object.h"
#include "utils/database/buffer_object.h"

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

    // update
    void update_obejct();
    void time_update();
    void update_dom(double time);

    // for debug
    void print_obejcts();
    void print_buffer_objects();

    // buffer objects
    BufferObject* buffer_object_;

    // init param
    std::string name;           // Object Name
    std::string condition;      // Static or Dynamic
    bool is_static;             // Static or Dynamic
    double init_dom;            // Init Difficulty of Moving
    double distance_th;         // if greater than or equal to this value, do not add

    // param
    int observations_count;     // Number of observations
    int appearance_count;       // Number of appearance (kari)
    int disappearance_count;    // Number of disappearance (kari)
    double dom;                 // Difficulty of Moving

private:
    bool str_to_bool(std::string _condition);

};
} // namespace dom_estimator

#endif  // OBJECTS_H_
