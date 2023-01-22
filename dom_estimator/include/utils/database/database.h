#ifndef DATABASE_H_
#define DATABASE_H_

#include <map>

#include "utils/object_param/object_param.h"
#include "utils/database/objects.h"

namespace dom_estimator
{
class Database : public std::map<ObjectParam*,Objects*>
{
public:
    Database();

    // add
    void add_init_object(std::string name,double x,double y);
    void add_object(std::string name,double x,double y,double time,double credibility);

    // update
    void update_objects();
    void time_update();
    void update_dom(double time);

    // for debug
    void print_contents();

private:
};
} // namespace dom_estimator

#endif  // DATABASE_H_
