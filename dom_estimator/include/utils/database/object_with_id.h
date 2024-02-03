#ifndef OBJECT_WITH_ID_H_
#define OBJECT_WITH_ID_H_

#include <algorithm>
#include <cmath>
#include <vector>

#include "utils/database/element.h"
#include "utils/database/buffer_object.h"

namespace dom_estimator
{
class ObjectWithID : public std::vector<Element>
{
public:
    ObjectWithID();
    ObjectWithID(int _id,std::string _name,std::string _condition,double _init_dom,double _distance_th);

    // add
    void add_init_object(double _x,double _y);
    void add_observed_object(double _x,double _y,double _time,double _credibility,double _dom);
    void add_object(double _x,double _y,double _time,double _credibility,double _error);
    void add_element(double _x,double _y,double _time,double _credibility);

    // update
    void update_obejct();
    void time_update(double _time);
    // void update_dom(double _time);
    void update_dom();

    double get_distance(double _x,double _y);

    // for debug
    // void print_obejcts();
    // void print_buffer_objects();
    void print_elements();

    // buffer objects
    // BufferObject* buffer_object_;

    // init param
    int id;                     // Object ID
    std::string name;           // Object Name (unused?)
    std::string condition;      // Static or Dynamic    (unused)
    bool is_static;             // Static or Dynamic    (unused)
    double init_dom;            // Init Difficulty of Moving
    double distance_th;         // if greater than or equal to this value, do not add   (unused)

    // param
    int observations_count;     // Number of observations   (unused)
    int appearance_count;       // Number of appearance (kari)  (unused)
    int disappearance_count;    // Number of disappearance (kari)   (unused)
    double total_distance_traveled; // Total distance traveled (m)
    double dom;                 // Difficulty of Moving

    bool has_observed;          // Observed or not
    double time;                // Time the element was last added
    double credibility;         // Credibility estimated from accumulated elements
    double x;                   // x-coordinate estimated from accumulated elements
    double y;                   // y-coordinate estimated from accumulated elements

private:
    bool str_to_bool(std::string _condition);

};
} // namespace dom_estimator

#endif  // OBJECT_WITH_ID_H_
