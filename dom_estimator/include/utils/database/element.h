#ifndef ELEMENT_H_
#define ELEMENT_H_

#include <iostream>

namespace dom_estimator
{
class Element
{
public:
    Element() :
        time(0.0), credibility(0.0), x(0.0), y(0.0) {}

    Element(double _time,double _credibility,double _x,double _y) :
        time(_time), credibility(_credibility), x(_x), y(_y) {}

    // for debug
    void print_element()
    {
        std::cout << "    -[Time,Credibility,X,Y]: ["
                  << time << ","
                  << credibility << ","
                  << x << ","
                  << y << "]" << std::endl;
    }

    double time;            // Time added to database
    double credibility;     // Absolute weight of the acquired robot
    double x;               // x-coordinate
    double y;               // y-coordinate

private:
};
} // namespace dom_estimator

#endif  // ELEMENT_H_
