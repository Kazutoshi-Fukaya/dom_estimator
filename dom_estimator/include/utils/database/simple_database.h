#ifndef SIMPLE_DATABASE_H_
#define SIMPLE_DATABASE_H_

#include <map>

#include "utils/database/simple_object.h"

namespace dom_estimator
{
class SimpleDatabase : public std::map<int,SimpleObject*>
{
public:
    SimpleDatabase();

    // add
    void add_init_object(int _id,double _start_time,double _init_dom,double _x,double _y);
    void add_new_object(int _id,double _time,double _x,double _y);
    void update_object(int _id,double _time,double _x,double _y);

    // for debug
    void print_contents();

private:
};
} // namespace dom_estimator

#endif  // SIMPLE_DATABASE_H_