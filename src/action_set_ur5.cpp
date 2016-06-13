////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#include "action_set_ur5.h"

// standard includes
#include <math.h>

// system includes
#include <angles/angles.h>
#include <sbpl_arm_planner/manip_lattice.h>

namespace sbpl_interface {

ActionSetUR5::ActionSetUR5() :
    sbpl::manip::ActionSet(),
    m_start_set(false),
    m_goal_set(false),
    m_all_ik_solutions()
{
}

void ActionSetUR5::updateStart(const sbpl::manip::RobotState& start)
{
    ActionSet::updateStart(start);

    m_start_set = true;
    updateSolutions();
}

void ActionSetUR5::updateGoal(const sbpl::manip::GoalConstraint& goal)
{
    ActionSet::updateGoal(goal);

    m_goal_set = true;
    updateSolutions();
}

bool ActionSetUR5::getAction(
    const sbpl::manip::RobotState& parent,
    double goal_dist,
    double start_dist,
    const sbpl::manip::MotionPrimitive& mp,
    std::vector<sbpl::manip::Action>& actions)
{
    if (mp.type == sbpl::manip::MotionPrimitive::SNAP_TO_XYZ_RPY) {
        if (!mprimActive(start_dist, goal_dist, mp.type)) {
            return false;
        }

        // return actions interpolating towards all ik solutions
        if (env_->getGoalConstraints().type !=
                sbpl::manip::GoalType::JOINT_STATE_GOAL)
        {
            size_t old_size = actions.size();
            actions.resize(actions.size() + m_all_ik_solutions.size());
            for (size_t i = 0; i < actions.size() - old_size; ++i) {
                actions[old_size + i].resize(1);
                actions[old_size + i][0] = m_all_ik_solutions[i];
            }
        }
        else {
            actions.resize(1);
            actions[0].resize(1);
            actions[0][0] = env_->getGoalConfiguration();
        }

        return true;
    }
    else {
        return ActionSet::getAction(parent, goal_dist, start_dist, mp, actions);
    }
}

void ActionSetUR5::updateSolutions()
{
    if (!m_goal_set || !m_start_set) {
        return;
    }

    ROS_INFO("Updating IK solutions");

    sbpl::manip::RobotModel* rm = env_->getRobotModel();

    m_all_ik_solutions.clear();

    // compute ik to the goal using the start state as the seed state
    std::vector<double> ik_sol;
    if (rm->computeIK(env_->getGoal(), env_->getStartConfiguration(), ik_sol)) {
        m_all_ik_solutions.push_back(ik_sol);
    }

    std::minstd_rand rng;
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    const int max_iters = 1000;
    for (int i = 0; i < max_iters; ++i) {
        // compute a random state
        sbpl::manip::RobotState rand_state(rm->getPlanningJoints().size(), 0.0);
        for (size_t jidx = 0; jidx < rm->getPlanningJoints().size(); ++jidx) {
            const double r = dist(rng);
            if (rm->hasPosLimit(jidx)) {
                const double span = rm->maxPosLimit(jidx) - rm->minPosLimit(jidx);
                const double rjv = rm->minPosLimit(jidx) + r * span;
                rand_state[jidx] = rjv;
            }
            else {
                const double rjv = 2.0 * M_PI * r;
                rand_state[jidx] = rjv;
            }
        }

        // compute ik to the goal using the random state as the seed state
        std::vector<double> ik_sol;
        if (rm->computeIK(env_->getGoal(), rand_state, ik_sol)) {
            // check closeness to any other solution
            if (std::all_of(m_all_ik_solutions.begin(), m_all_ik_solutions.end(),
                    [&](const sbpl::manip::RobotState& state) {
                        return !almostEqual(state, ik_sol);
                    }))
            {
                ROS_INFO("Found a novel ik solution");
                m_all_ik_solutions.push_back(ik_sol);
            }
        }
    }

    ROS_INFO("Computed %zu ik solutions", m_all_ik_solutions.size());
}

bool ActionSetUR5::almostEqual(
    const sbpl::manip::RobotState& s1,
    const sbpl::manip::RobotState& s2) const
{
    sbpl::manip::RobotModel* rm = env_->getRobotModel();
    const double err = 0.01;
    for (size_t jidx = 0; jidx < rm->getPlanningJoints().size(); ++jidx) {
        double dist;
        if (rm->hasPosLimit(jidx)) {
            dist = fabs(s2[jidx] - s1[jidx]);
        }
        else {
            dist = fabs(angles::shortest_angular_distance(s2[jidx], s1[jidx]));
        }

        if (dist > err) {
            return false;
        }
    }

    return true;
}

} // namespace sbpl_interface
