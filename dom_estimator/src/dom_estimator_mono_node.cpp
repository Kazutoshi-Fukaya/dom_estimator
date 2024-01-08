#include "dom_estimator_mono/dom_estimator_mono.h"

using namespace dom_estimator;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"dom_estimator_mono");
    DomEstimatorMono dom_estimator_mono;
    dom_estimator_mono.process();
    return 0;
}
