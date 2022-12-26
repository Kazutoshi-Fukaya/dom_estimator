#ifndef COLOR_H_
#define COLOR_H_

namespace dom_estimator
{
class Color
{
public:
    Color() :
        r(0.0), g(0.0), b(0.0) {}
    Color(double _r,double _g,double _b) :
        r(_r), g(_g), b(_b) {}

    double r;
    double g;
    double b;

private:
};
} // namespace dom_estimator

#endif  //COLOR_H_
