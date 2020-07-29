#ifndef _OMPL_RRT_BIAS_H_
#define _OMPL_RRT_BIAS_H_

#include "ompl/base/Planner.h"

#include "ompl/util/RandomNumbers.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/PlannerIncludes.h"

namespace ompl
{
    namespace geometric
    {
        
        class RRT_Bias : public base::Planner
        {
            public:
                /* @brief Construct */
                RRT_Bias(const base::SpaceInformationPtr &si);

                ~RRT_Bias() override;
                
                /* setup  */
                void setup(); override;
            protected:
                /* */
                class Motion
                {
                
                };
        };
    }
}

#endif
