/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#include "ompl/geometric/planners/rrt/RRTConnectGugu.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/String.h"

ompl::geometric::RRTConnectGugu::RRTConnectGugu(const base::SpaceInformationPtr &si, bool addIntermediateStates)
  : base::Planner(si, addIntermediateStates ? "RRTConnectGuguIntermediate" : "RRTConnectGugu")
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &RRTConnectGugu::setRange, &RRTConnectGugu::getRange, "0.:1.:10000.");
    Planner::declareParam<bool>("intermediate_states", this, &RRTConnectGugu::setIntermediateStates,
                                &RRTConnectGugu::getIntermediateStates, "0,1");

    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
    addIntermediateStates_ = addIntermediateStates;
}

ompl::geometric::RRTConnectGugu::~RRTConnectGugu()
{
    freeMemory();
}

void ompl::geometric::RRTConnectGugu::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    //tStart_和tGoal_通过智能指针的reset(p)方法指向了新对象NearestNeighbors<Motion *> p
    if (!tStart_)
        tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!tGoal_)
        tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    tStart_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    tGoal_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::geometric::RRTConnectGugu::freeMemory()
{
    std::vector<Motion *> motions;

    if (tStart_)
    {
        tStart_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }

    if (tGoal_)
    {
        tGoal_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

void ompl::geometric::RRTConnectGugu::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (tStart_)
        tStart_->clear();
    if (tGoal_)
        tGoal_->clear();
    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
}

ompl::geometric::RRTConnectGugu::GrowState ompl::geometric::RRTConnectGugu::growTree(TreeData &tree, Motion *&potentialtreemotion,
                                                                             Motion *rmotion)
{
    /* find closest state in the tree */
    Motion *nmotion = tree->nearest(rmotion);

    /* assume we can reach the state we go towards */
    bool reach = true;

    /* tree新增motion的state指针，函数内部用，初始化为指向采样点state，若距离tree过远，则后续会更新为指向插值得到的state */
    base::State *dstate = rmotion->state;

    double d = si_->distance(nmotion->state, rmotion->state);
    if (d > maxDistance_)
    {
        base::State *tempstate = si_->allocState(); //临时记录插值后的state
        si_->getStateSpace()->interpolate(nmotion->state, rmotion->state, maxDistance_ / d, tempstate);

        /* Check if we have moved at all. Due to some stranger state spaces (e.g., the constrained state spaces),
         * interpolate can fail and no progress is made. Without this check, the algorithm gets stuck in a loop as it
         * thinks it is making progress, when none is actually occurring. */
        if (si_->equalStates(nmotion->state, tempstate))
            return TRAPPED;

        dstate = tempstate; //tree新增motion的state指针更新为指向插值得到的state
        reach = false;
    }


    bool validMotion = si_->checkMotion(nmotion->state, dstate);

    if (!validMotion)
        return TRAPPED;
    else
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, dstate);
        motion->parent = nmotion;
        motion->root = nmotion->root;
        tree->add(motion);

        potentialtreemotion = motion; //更新potentialtreemotion指向树新增motion
        return reach ? REACHED : ADVANCED;
    }
}

void ompl::geometric::RRTConnectGugu::constructSolPath(Motion *startMotion, Motion *goalMotion, std::shared_ptr<ompl::geometric::PathGeometric> path)
{
    /* construct the solution path */
    Motion *solution = startMotion;
    std::vector<Motion *> mpath1;
    while (solution != nullptr)
    {
        mpath1.push_back(solution);
        solution = solution->parent;
    }

    solution = goalMotion;
    std::vector<Motion *> mpath2;
    while (solution != nullptr)
    {
        mpath2.push_back(solution);
        solution = solution->parent;
    }

    path->getStates().reserve(mpath1.size() + mpath2.size());
    for (int i = mpath1.size() - 1; i >= 0; --i)
        path->append(mpath1[i]->state);
    for (auto &i : mpath2)
        path->append(i->state);
}

ompl::base::PlannerStatus ompl::geometric::RRTConnectGugu::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    
    // 初始化goal
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());
    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }
    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    // 初始化start：将start加入到startTree中，并设置其为根节点
    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        tStart_->add(motion);
    }
    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }
    OMPL_INFORM("%s: Starting planning with %d states already in start tree datastructure", getName().c_str(),
                tStart_->size());
    OMPL_INFORM("%s: Starting planning with %d states already in goal tree datastructure", getName().c_str(),
                tGoal_->size());

    //初始化sampler            
    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    // 初始化startTree和goalTree newest motion
    Motion *startMotion; //这里仅声明，具体分配内存在growTree()函数内部实现
    Motion *goalMotion;

    //初始化随机采样点
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;

    // 初始化求解成功标志字及求解结果状态
    bool solved = false;
    base::PlannerStatus::StatusType status = base::PlannerStatus::TIMEOUT;

    while (!ptc)
    {     
        //将goal加入到goalTree中去，并设置其为根节点
        if (tGoal_->size() == 0)
        {
            const base::State *st = pis_.nextGoal(ptc);
            if (st != nullptr)
            {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, st);
                motion->root = motion->state;
                tGoal_->add(motion);
            }

            if (tGoal_->size() == 0)
            {
                OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
                status = base::PlannerStatus::INVALID_GOAL;
                break;
            }
        }
        
        if (startTree_) //生长startTree
        {
            /* sample random state */
            sampler_->sampleUniform(rstate); 
            
            //将startTree向rmotion生长，startmotion指向startTree最终生长motion
            GrowState gs = growTree(tStart_, startMotion, rmotion);

            //若tree生长成功，则进一步尝试将新增生长点朝另一个棵树的最近点贪婪生长
            if (gs != TRAPPED)
            {
                //将goalTree向startmotion贪婪生长，goalmotion指向goalTree最终生长motion
                GrowState gsc = growTree(tGoal_, goalMotion, startMotion);
                while (gsc == ADVANCED)
                    gsc = growTree(tGoal_, goalMotion, startMotion);
                
                /* if we connected the trees in a valid way (start and goal pair is valid)*/
                if (gsc == REACHED && goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
                {
                    // it must be the case that either the start tree or the goal tree has made some progress
                    // so one of the parents is not nullptr. We go one step 'back' to avoid having a duplicate state
                    // on the solution path
                    if (startMotion->parent != nullptr)
                        startMotion = startMotion->parent;
                    else
                        goalMotion = goalMotion->parent;
                    
                    /* construct the solution path */
                    auto path(std::make_shared<PathGeometric>(si_));
                    constructSolPath(startMotion, goalMotion, path);

                    pdef_->addSolutionPath(path, false, 0.0, getName());
                    solved = true;
                    break;
                }     
            }
   
            startTree_ = false;
        }
        else //生长goalTree
        {
            /* sample random state */
            sampler_->sampleUniform(rstate); 
            
            //将goalTree向rmotion生长，goalMotion指向goalTree最终生长motion
            GrowState gs = growTree(tGoal_, goalMotion, rmotion);

            //若tree生长成功，则进一步尝试将新增生长点朝另一个棵树的最近点贪婪生长
            if (gs != TRAPPED)
            {             
                //将startTree向goalmotion贪婪生长，startMotion指向startTree最终生长motion
                GrowState gsc = growTree(tStart_, startMotion, goalMotion);
                while (gsc == ADVANCED)
                    gsc = growTree(tStart_, startMotion, goalMotion);
                
                /* if we connected the trees in a valid way (start and goal pair is valid)*/
                if (gsc == REACHED && goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
                {
                    // it must be the case that either the start tree or the goal tree has made some progress
                    // so one of the parents is not nullptr. We go one step 'back' to avoid having a duplicate state
                    // on the solution path
                    if (startMotion->parent != nullptr)
                        startMotion = startMotion->parent;
                    else
                        goalMotion = goalMotion->parent;
                        
                    /* construct the solution path */
                    auto path(std::make_shared<PathGeometric>(si_));
                    constructSolPath(startMotion, goalMotion, path);

                    pdef_->addSolutionPath(path, false, 0.0, getName());
                    solved = true;
                    break;
                }     
            }

            startTree_ = true;
        }
    }

    si_->freeState(rstate);
    delete rmotion;

    /*这里不能delete，否则会崩溃，不知道原因，但参考RRTConnect中也没有删除tgi.xmotion，
    可能是因为这里存的是tree，solve()函数外部还需要用到该量，故不应该在solve内把他delete.
    同理可参考，指向起始点和目标点的motion指针也没有delete，应该也是外部函数还要用，
    实际在该planner的析构函数内会通过freeMemory去free掉tStart_和tGoal_中的每一个motion*/
    // delete startMotion; 
    // delete goalMotion; 

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(),
                tStart_->size(), tGoal_->size());

    return solved ? base::PlannerStatus::EXACT_SOLUTION : status;
}

void ompl::geometric::RRTConnectGugu::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (tStart_)
        tStart_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state, 1));
        else
        {
            data.addEdge(base::PlannerDataVertex(motion->parent->state, 1), base::PlannerDataVertex(motion->state, 1));
        }
    }

    motions.clear();
    if (tGoal_)
        tGoal_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addGoalVertex(base::PlannerDataVertex(motion->state, 2));
        else
        {
            // The edges in the goal tree are reversed to be consistent with start tree
            data.addEdge(base::PlannerDataVertex(motion->state, 2), base::PlannerDataVertex(motion->parent->state, 2));
        }
    }

    // Add the edge connecting the two trees
    data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));

    // Add some info.
    data.properties["approx goal distance REAL"] = ompl::toString(distanceBetweenTrees_);
}
