#include "../include/rrt_bias/rrt_bias.h"
#include <limits>
#include <memory>
#include <ompl/base/GoalTypes.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/State.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/util/Console.h>
#include <vector>

ompl::geometric::RRT_Bias::RRT_Bias(const base::SpaceInformationPtr &si)
    : base::Planner(si, "RRT Bias")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

}

ompl::geometric::RRT_Bias::~RRT_Bias()
{

}

void ompl::geometric::RRT_Bias::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if(!nn_)
    {
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    }
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b);});

}

void ompl::geometric::RRT_Bias::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
    {
        nn_->clear();
    }
    lastGoalMotion_ = nullptr;
}

void ompl::geometric::RRT_Bias::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if(motion->state != nullptr)
            {
                si_->freeState(motion->state);
            }
            delete motion;
        }
    }
}

ompl::base::PlannerStatus ompl::geometric::RRT_Bias::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();

    // base on the problem define,get the goal handle
    base::Goal *goal = pdef_->getGoal().get();
    // the goal sampler region
    // auto *goal_smp = dynamic_cast<base::GoalSampleableRegion *>(goal);

    // get the input state 
    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    // check nearest neighbor whether update
    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // check the sampler whether init,if not ,alloc a default state sampler
    if (!sampler_)
    {
        sampler_ = si_->allocStateSampler();
    }

    OMPL_INFORM("%s: Strating planning with %u states already in datastructure", getName().c_str(), nn_->size());

    // relation variable init
    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rnd_motion = new Motion(si_);
    base::State *rnd_state = rnd_motion->state;
    base::State *x_state = si_->allocState();

    while (!ptc)
    {
        // sample random state
        sampler_->sampleUniform(rnd_state);

        // find closest state in the tree
        Motion *nn_motion = nn_->nearest(rnd_motion);

        // update the distance state
        base::State *d_state = rnd_state;

        // Compute the distance between nearest state and the random state
        double d = si_->distance(nn_motion->state, rnd_state);
        if ( d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nn_motion->state, rnd_state, maxDistance_ / d, x_state);
            d_state = x_state;
        }

        // avoidance checker
        if (si_->checkMotion(nn_motion->state, d_state))
        {
            /*
             * @brief Step1: Add new motion to the nearest neighbors tree
             */
            // allocate a new motion temporary variable
            auto *new_motion = new Motion(si_);
            // clone the d_state to the new motion temporary variable
            si_->copyState(new_motion->state, d_state);
            // link the new motion to parent motion
            new_motion->parent = nn_motion;
            // add the new motion variable to the nearest neighbors tree
            nn_->add(new_motion);

            // update the nearest neighbors motion
            nn_motion = new_motion;

            /*
             * @brief Step2: Check whether arrive goal point 
             */
            double dist = .0;
            // judge the new state whether satisfied goal condition
            // and calculate the distance to the goal
            bool sat = goal->isSatisfied(nn_motion->state, &dist);
            // if the satisfied break out while and store the distance
            if (sat)
            {
                approxdif = dist;
                solution = nn_motion;
                break;
            }
            // if the current distance lower the approximate distanc,then
            // update and the approximate solution update
            if (dist  < approxdif)
            {
                approxdif = dist;
                approxsol = nn_motion;
            }
        } // end if check motion
    }// end while(!ptc)

    /*
     * @brief The solution process
     */
    // declare the return status variable and initialize them 
    bool solved = false;
    bool approximate = false;
    // if the planner terminal condition satisfied, but not the goal region
    // arrived.then the 'solution' variable is null.at this time, the solution update
    // with the approximate solution and set the 'appriximate' variable to be ture.
    if (solution == nullptr)
    {
        solution  = approxsol;
        approximate = true;
    }

    //no matter what the solution whether is approximate, only if the 'solution' is
    //not null pointer, then update the 'lastGoalMotion',base on the solution
    //tree,backward get the path and add the path to the 'pef_',set the 'solved' to be true
    if (solution != nullptr)
    {
        // update the 'lastGoalMotion'
        lastGoalMotion_ = solution;

        /*
         * @brief construct the solution path
         */

        // decelare the temporary motion path variable
        std::vector<Motion *> motion_path;
        // backwards get the solution path
        while (solution != nullptr)
        {
            motion_path.push_back(solution);
            solution = solution->parent;
        }

        /*
         * @brief set the solution path
         */
        // declare the temporary 'path' variable
        auto path(std::make_shared<PathGeometric>(si_));
        // add the solution path state to the 'path'
        for (int i = motion_path.size() - 1; i >= 0; --i)
        {
            path->append(motion_path[i]->state);
        }
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    /*
     * @brief free the memory
     */
    // the 'x_state' is allocate by the si_->allocate() function,so need to use
    // the si_->freestate()function to free the memory
    si_->freeState(x_state);
    if (rnd_motion->state != nullptr)
    {
        si_->freeState(rnd_motion->state);
    }
    delete rnd_motion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, approximate};
}


