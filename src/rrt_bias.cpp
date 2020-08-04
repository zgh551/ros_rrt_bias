#include "../include/rrt_bias/rrt_bias.h"
#include <ompl/base/StateValidityChecker.h>

ompl::geometric::RRT_Bias::RRT_Bias(const base::SpaceInformationPtr &si)
    : base::Planner(si, "RRT Bias")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

}

ompl::geometric::RRT_Bias::~RRT_Bias()
