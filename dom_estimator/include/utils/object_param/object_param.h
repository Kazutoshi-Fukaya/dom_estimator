#ifndef OBJECT_PARAM_H_
#define OBJECT_PARAM_H_

#include <iostream>

#include "utils/object_param/color.h"

namespace dom_estimator
{
class ObjectParam
{
public:
    ObjectParam() :
        name(std::string("")), is_static(NULL), color(Color(0.0,0.0,0.0)) {}

    ObjectParam(std::string _name,bool _is_static,Color _color) :
        name(_name), is_static(_is_static), color(_color) {}

    ObjectParam(std::string _name,std::string _condition,Color _color) :
        name(_name), is_static(str_to_bool(_condition)), color(_color) {}

    ~ObjectParam() {}

    // for debug
    void print_param()
    {
        std::cout << "Name: " << name << std::endl;
        std::cout << "Condition: " << bool_to_str(is_static) << std::endl;
        std::cout << "Color(R,G,B): (" << color.r << ","
                                       << color.g << ","
                                       << color.b << ")" << std::endl << std::endl;
    }

    std::string name;
    bool is_static;
    Color color;

private:
    bool str_to_bool(std::string _condition)
    {
        if(_condition == std::string("Static")) return true;
        return false;
    }

    std::string bool_to_str(bool _is_static)
    {
        if(_is_static) return std::string("Static");
        else return std::string("Semi-Dynamic");
    }
};
} // namespace dom_estimator

#endif  //OBJECT_PARAM_H_
