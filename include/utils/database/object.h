#ifndef OBJECT_H_
#define OBJECT_H_

#include <vector>

#include "utils/database/element.h"

namespace dom_estimator
{
class Object : public std::vector<Element>
{
public:
	Object();

	// add
	void add_init_object(double _x,double _y);
	void add_object(double _x,double _y,double _time,double _credibility);

	// for debug
	void print_elements();

	double time;				// Time the element was last added
    double credibility;			// Credibility estimated from accumulated elements
    double x;					// x-coordinate estimated from accumulated elements
    double y;					// y-coordinate estimated from accumulated elements

private:

};
} // namespace dom_estimator


#endif  // OBJECT_H_