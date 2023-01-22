#include "dom_estimator/dom_estimator.h"

using namespace dom_estimator;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"dom_estimator");
    DomEstimator dom_estimator;
    dom_estimator.process();
    return 0;
}
