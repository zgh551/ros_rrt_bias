#ifndef _OMPL_RRT_BIAS_H_
#define _OMPL_RRT_BIAS_H_

#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/goals/GoalSampleableRegion.h"


//#include <memory>
//#include <ompl/base/PlannerStatus.h>
//#include <ompl/base/PlannerTerminationCondition.h>
//#include <ompl/base/State.h>
//#include <ompl/base/StateSampler.h>
//#include <ompl/base/StateSpaceTypes.h>
//#include <ompl/base/StateValidityChecker.h>
//#include "ompl/util/RandomNumbers.h"


namespace ompl
{
    namespace geometric
    {
        
        class RRT_Bias : public base::Planner
        {
            public:
                /* 
                 * @brief: Construct
                 */
                RRT_Bias(const base::SpaceInformationPtr &si);

                ~RRT_Bias() override;
                
                /*
                 * @brief: The only once setup
                 * @param: None
                 * @return None
                 */
                void setup() override;

                /* @brief: Free the 
                 * @param: None
                 * @return None
                 */
                void clear() override;

                /*
                 * @brief: The solution function
                 */
                base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            protected:
                /*
                 * @brief: Representation of a motion
                 * @brief: This only contains pointers to parent motions as we
                 * only need to go backwards in the tree
                 */
                class Motion
                {
                    public:
                        Motion() = default;

                        /*
                         * @brief Constructor that allocates memory for the
                         * state
                         */
                        Motion(const base::SpaceInformationPtr &si) 
                            : state(si->allocState())
                        {
                        }

                        ~Motion() = default;

                        /*
                         * @brief The state contained by the motion
                         */
                        base::State *state{nullptr};

                        /*
                         * @brief: The parent motion in the exploration tree
                         */
                        Motion *parent{nullptr};
                };

                /*
                 * @brief Freedom the memory space that allocate in solve
                 * processing 
                 * @param None
                 * @retun None
                 */
                void freeMemory();

                /*
                 * @brief: Compute distance between motions (actually distance
                 * between contained states)
                 * @param a: A motion state
                 * @param b: Another motion state
                 * @retun The distance between motions
                 */
                double distanceFunction(const Motion *a, const Motion *b) const
                {
                    return si_->distance(a->state, b->state);
                }

                /*
                 * @brief: The state sampler
                 */
                base::StateSamplerPtr sampler_;

                /*
                 * @brief: A nearest-neighbors datastructure containing the
                 * tree of motions
                 */
                std::shared_ptr<NearestNeighbors<Motion *>> nn_;

                /*
                 * @brief: The random number generator
                 */
                RNG rng_;

                /*
                 * @brief: The most recent goal motion. Used for PlannnerData
                 * Computation
                 */
                Motion *lastGoalMotion_{nullptr};

                /*
                 * @brief: The maximum lenght of a motion to be added to a tree
                 */
                double maxDistance_{0.};

                /*
                 * @brief: The fraction of time the goal is picked as the state
                 * to expand towards (if such a state is available)
                 */
                double goalBias_{.05};

        };
    }
}

#endif
