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


	// for debug
	void print_contents();

private:
};
} // namespace dom_estimator

#endif  // DATABASE_H_