#ifndef BUFFER_OBJECT_H_
#define BUFFER_OBJECT_H_

#include <algorithm>
#include <cmath>
#include <vector>

#include "utils/database/element.h"

namespace dom_estimator
{
class BufferElements : public std::vector<Element>
{
public:
    BufferElements();

    void add_element(double _time,double _credibility,double _x,double _y);
    void calc_center();
    size_t get_buffer_size();
    double get_distance(double _x,double _y);

    // for debug
    void print_elements();

    double time;
    double credibility;
    double x;
    double y;
private:
};

class BufferObject : public std::vector<BufferElements>
{
public:
    BufferObject();

    void add_buffer(double _time,double _credibility,double _x,double _y);
    bool are_elements_to_add(std::vector<Element>& elements);

    // for debug
    void print_bufffer_elements();

private:
    const double distance_th = 0.3;
    const int size_th = 10;
};
} // namespace dom_estimator

#endif  // BUFFER_OBJECT_H_
