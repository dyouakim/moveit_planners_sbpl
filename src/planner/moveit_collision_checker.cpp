////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2015, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
// may be used to endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#include "moveit_collision_checker.h"

// standard includes
#include <limits>

// system includes
#include <leatherman/print.h>
#include <leatherman/viz.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <smpl/angles.h>
#include <smpl/debug/marker_conversions.h>

#include <moveit_planners_sbpl/planner/moveit_robot_model.h>

namespace sbpl_interface {

namespace smpl = sbpl::motion;

MoveItCollisionChecker::MoveItCollisionChecker() :
    Base(),
    m_robot_model(nullptr),
    m_scene(),
    m_ref_state()
{
    ros::NodeHandle nh;
    m_vpub = nh.advertise<visualization_msgs::MarkerArray>("visualization_markers", 10);
    collision_model = nh.advertise<visualization_msgs::MarkerArray>("/tra_collision", 10);
    obstaclesInfoPub_ = nh.advertise<moveit_msgs::CollisionCheckResponse>("/moveit_collision_checker/obstacles_info",5);
    counter = 0;
    lastExpansionStep_ = -1;
}

MoveItCollisionChecker::~MoveItCollisionChecker()
{
}

bool MoveItCollisionChecker::init(
    MoveItRobotModel* robot_model,
    const moveit::core::RobotState& ref_state,
    const planning_scene::PlanningSceneConstPtr& scene)
{
    ROS_DEBUG("Initializing MoveIt! Collision Checker");

    if (!robot_model->initialized()) {
        ROS_ERROR("Failed to initialize MoveIt Collision Checker: "
                "MoveIt Robot Model must be initialized");
        return false;
    }

    if (!scene) {
        ROS_ERROR("Failed to initialize MoveIt Collision Checker: "
                "Planning Scene is null");
        return false;
    }

    if (robot_model->moveitRobotModel()->getName() !=
        scene->getRobotModel()->getName())
    {
        ROS_ERROR("Failed to initialize MoveIt Collision Checker: "
                "model is not the same between SBPL Robot Model and Planning Scene");
        return false;
    }

    if (robot_model->moveitRobotModel()->getName() !=
        ref_state.getRobotModel()->getName())
    {
        ROS_ERROR("Failed to initialize MoveIt Collision Checker: "
                "model is not the same between SBPL Robot Model and reference state");
        return false;
    }

    m_robot_model = robot_model;
    m_var_incs.reserve(m_robot_model->getPlanningJoints().size());
    for (const std::string& joint_name : m_robot_model->getPlanningJoints()) {
        m_var_incs.push_back(sbpl::angles::to_radians(2.0));
    }
    ROS_INFO("Increments: %s", to_string(m_var_incs).c_str());

    m_ref_state.reset(new moveit::core::RobotState(scene->getRobotModel()));
    *m_ref_state = ref_state;

    m_scene = scene;

    m_zero_state.resize(m_robot_model->activeVariableCount(), 0.0);

    ros::NodeHandle ph("~");
    ph.param("enable_ccd", m_enabled_ccd, false);
    ROS_INFO("enable_ccd: %s", m_enabled_ccd ? "true" : "false");

    return true;
}

bool MoveItCollisionChecker::initialized() const
{
    return (bool)m_robot_model;
}

smpl::Extension* MoveItCollisionChecker::getExtension(size_t class_code)
{
    if (class_code == smpl::GetClassCode<smpl::CollisionChecker>()) {
        return this;
    }
    return nullptr;
}

void MoveItCollisionChecker::markGridForExpandedState(const sbpl::motion::RobotState& state,const sbpl::motion::RobotState& parent_state,int step)
{

    planning_scene::PlanningScene* nonconst_scene = const_cast<planning_scene::PlanningScene*> (m_scene.get());
    
    collision_detection::GridWorld* grid_world = dynamic_cast<  collision_detection::GridWorld*>(nonconst_scene->getWorldNonConst().get());
    
    double res = grid_world->grid()->resolution();
    int gx,gy,gz;
    Eigen::Affine3d link_pose;
    ros::Time t1 = ros::Time::now();

    std::vector< const robot_model::LinkModel*> links = m_robot_model->getLinksModels();
    

    int waypoint_count = interpolatePathFast(parent_state, state, m_waypoint_path);
    if (waypoint_count < 0) {
        return ;
    }

    for (int widx = 0; widx < waypoint_count; ++widx) 
    {
        ros::Time t1internal = ros::Time::now();
        const smpl::RobotState& p = m_waypoint_path[widx];
        setRobotStateFromState(*m_ref_state, p);
        for (std::size_t i = 0; i < links.size(); ++i)
        {   
            //ROS_WARN_STREAM("Link "<<i<<" name is "<<links[i]->getName());
            link_pose = m_ref_state->getGlobalLinkTransform(links[i]->getName());//links[i]->getLinkIndex());
            Eigen::Vector3d dim = links[i]->getShapeExtentsAtOrigin ();
            
            Eigen::Quaterniond q(link_pose.rotation());
            int xnumCells = ceil(dim[0]/res);
            int ynumCells = ceil(dim[1]/res);
            int znumCells = ceil(dim[2]/res);
            //ROS_WARN_STREAM("new call!");
            /*ROS_INFO_STREAM("x,y,z Number of cells for link "<<links[i]->getName()<<" is "<<xnumCells<<","<<ynumCells<<","<<znumCells);
            ROS_INFO_STREAM("and pose "<<link_pose.translation().x()<<","<<link_pose.translation().y()<<","<<link_pose.translation().z());
            ROS_INFO_STREAM("and dimensions "<<dim[0]<<","<<dim[1]<<","<<dim[2]);*/
                
            if(links[i]->getName()=="arm_base_link_yaw")
            {
                //ROS_INFO_STREAM("link pose "<<link_pose.translation().x()<<","<<link_pose.translation().y()<<","<<link_pose.translation().z());
                int xSign = 1,ySign = 1, zSign = 1;
                for(int j=0;j<xnumCells;j++)
                for(int k=0;k<ynumCells;k++)
                    for(int l=0;l<znumCells;l++)
                    {
                        if(j>xnumCells/2)
                            xSign = -(j-std::ceil(xnumCells/2));
                        else
                            xSign = j;
                        if(k>ynumCells/2)
                            ySign = -(k-std::ceil(ynumCells/2));
                        else
                            ySign = k;
                        if(l>znumCells/2)
                            zSign = -(l-std::ceil(znumCells/2));
                        else
                            zSign = l; 

                        grid_world->grid()->markCellExpansionStep (
                            link_pose.translation().x()+((xSign)*res), 
                            link_pose.translation().y()+((ySign)*res),
                            link_pose.translation().z()+((zSign)*res),step);
                        /*ROS_INFO_STREAM("Collision checker marking cell "<<link_pose.translation().x()+((xSign)*res)<<","<<
                    link_pose.translation().y()+((ySign)*res)<<","<<link_pose.translation().z()+((zSign)*res));*/
                    }

            }
            for(int j=0;j<xnumCells;j++)
                for(int k=0;k<ynumCells;k++)
                    for(int l=0;l<znumCells;l++)
                    {
                        //ROS_WARN_STREAM("hereeee "<<lastExpansionStep_);
                grid_world->grid()->markCellExpansionStep (link_pose.translation().x()+(j*res),link_pose.translation().y()+(k*
            res),link_pose.translation().z()+(l*res),step);
                /*ROS_WARN_STREAM("Collision checker marking cell "<<link_pose.translation().x()+(j*res)<<","<<
                    link_pose.translation().y()+(k*res)<<","<<link_pose.translation().z()+(l*res));*/
            }

            /*visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time();
            
            marker.ns = links[i]->getName();
            marker.id = counter;
            counter++;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.orientation.x = q.x();
            marker.pose.orientation.y = q.y();
            marker.pose.orientation.z = q.z();
            marker.pose.orientation.w = q.w();

            
            marker.scale.x = dim[0];
            marker.scale.y = dim[1];
            marker.scale.z = dim[2];

            marker.pose.position.x = link_pose.translation().x();
            marker.pose.position.y = link_pose.translation().y();
            marker.pose.position.z = link_pose.translation().z();
            marker.color.a = 1.0;
            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.0;
            
            vis_array.markers.push_back(marker);
            collision_model.publish(vis_array);*/
           
        }
        ros::Time t2internal = ros::Time::now();
        //ROS_INFO_STREAM("Cell Marking duration of point "<<widx<<" is "<<t2internal.toSec()<<","<<t1internal.toSec()<<","<<t2internal.toSec()-t1internal.toSec());
   
    }

    ros::Time t2 = ros::Time::now();
    ROS_INFO_STREAM("Cell Marking duration "<<t2.toSec()<<","<<t1.toSec()<<","<<t2.toSec()-t1.toSec()<<" for expansion step "<<step);
   
}

bool MoveItCollisionChecker::isStateValid(const smpl::RobotState& state, double& distToObst, bool verbose)
{

    if (!initialized()) {
        ROS_ERROR("MoveItCollisionChecker is not initialized");
        return false;
    }


    setRobotStateFromState(*m_ref_state, state);

    // TODO: need to propagate path_constraints and trajectory_constraints down
    // to this level from the planning context. Once those are propagated, this
    // call will need to be paired with an additional call to isStateConstrained

    // NOTE: since m_ref_state is not const, this call to isStateColliding will
    // go ahead and update the link transforms underneath before checking for
    // collisions. Source:
    //
    // http://docs.ros.org/indigo/api/moveit_core/html/classplanning__scene_1_1PlanningScene.html
    //
    /*collision_detection::CollisionRequest req;
    req.verbose = verbose;
    req.group_name = m_robot_model->planningGroupName();
    req.contacts = true;
    req.distance = true;
    collision_detection::CollisionResult res;
    m_scene->checkCollision(req,res,*m_ref_state);
    moveit_msgs::CollisionCheckResponse response;
    std::vector<moveit_msgs::CollisionContact> contacts;

    if(verbose)
    {
        if(res.collision)
        {
            for (collision_detection::CollisionResult::ContactMap::const_iterator it = res.contacts.begin(); it != res.contacts.end();
           ++it)
                for (std::size_t j = 0; j < it->second.size(); ++j)
                {
                    moveit_msgs::CollisionContact contact;
                    contact.source = it->second[j].body_name_1.c_str(); 
                    contact.dest = it->second[j].body_name_2.c_str();
                    contacts.push_back(contact);
                }
        }
        response.collision = res.collision;
        response.distance = res.distance;
        response.contacts = contacts;
        obstaclesInfoPub_.publish(response);
    }
    distToObst = res.distance;
    
    return !res.collision;*/
    return !m_scene->isStateColliding(
            *m_ref_state, m_robot_model->planningGroupName(), distToObst, verbose);
    
}



bool MoveItCollisionChecker::isStateValid(
    const smpl::RobotState& state,
    bool verbose)
{
    if (!initialized()) {
        ROS_ERROR("MoveItCollisionChecker is not initialized");
        return false;
    }

    setRobotStateFromState(*m_ref_state, state);

   
    // TODO: need to propagate path_constraints and trajectory_constraints down
    // to this level from the planning context. Once those are propagated, this
    // call will need to be paired with an additional call to isStateConstrained

    // NOTE: since m_ref_state is not const, this call to isStateColliding will
    // go ahead and update the link transforms underneath before checking for
    // collisions. Source:
    //
    // http://docs.ros.org/indigo/api/moveit_core/html/classplanning__scene_1_1PlanningScene.html
    //

    /*collision_detection::CollisionRequest req;
    req.verbose = verbose;
    req.group_name = m_robot_model->planningGroupName();
    req.contacts = true;
    req.distance = true;
    collision_detection::CollisionResult res;
    m_scene->checkCollision(req,res,*m_ref_state);
    moveit_msgs::CollisionCheckResponse response;
    std::vector<moveit_msgs::CollisionContact> contacts;
    if(verbose)
    {
        if(res.collision)
        {
            for (collision_detection::CollisionResult::ContactMap::const_iterator it = res.contacts.begin(); it != res.contacts.end();
           ++it)
                for (std::size_t j = 0; j < it->second.size(); ++j)
                {
                    moveit_msgs::CollisionContact contact;
                    contact.source = it->second[j].body_name_1.c_str(); 
                    contact.dest = it->second[j].body_name_2.c_str();
                    contacts.push_back(contact);
                }
        }
        response.collision = res.collision;
        response.distance = -1;
        response.contacts = contacts;
        obstaclesInfoPub_.publish(response);
    }
    
    return !res.collision;*/
    return !m_scene->isStateColliding(
            *m_ref_state, m_robot_model->planningGroupName(), verbose);
}

bool MoveItCollisionChecker::isStateToStateValid(
    const smpl::RobotState& start,
    const smpl::RobotState& finish,
    bool verbose)
{
    if (m_enabled_ccd) {
        return checkContinuousCollision(start, finish);
    } else {
        return checkInterpolatedPathCollision(start, finish);
    }
}

bool MoveItCollisionChecker::isStateToStateValid(
    const smpl::RobotState& start,
    const smpl::RobotState& finish, double& distToObst, int& distToObstCells,
    bool verbose)
{
    if (m_enabled_ccd) {
        return checkContinuousCollision(start, finish, distToObst, distToObstCells);
    } else {
        return checkInterpolatedPathCollision(start, finish, distToObst, distToObstCells);
    }
}

bool MoveItCollisionChecker::interpolatePath(
    const smpl::RobotState& start,
    const smpl::RobotState& finish,
    std::vector<smpl::RobotState>& opath)
{
    opath.clear();
    return interpolatePathFast(start, finish, opath) >= 0;
}

auto MoveItCollisionChecker::checkContinuousCollision(
    const smpl::RobotState& start,
    const smpl::RobotState& finish)
    -> bool
{
    collision_detection::CollisionRequest req;
    req.verbose = false;
    req.group_name = m_robot_model->planningGroupName();
    collision_detection::CollisionResult res;

    auto cw = m_scene->getCollisionWorld();
    moveit::core::RobotState state1(*m_ref_state);
    moveit::core::RobotState state2(*m_ref_state);
    setRobotStateFromState(state1, start);
    setRobotStateFromState(state2, finish);

    planning_scene::PlanningScene* scene_nonconst = const_cast <planning_scene::PlanningScene*> (m_scene.get());
    cw->checkRobotCollision(
            req,
            res,
            *m_scene->getCollisionRobot(),
            state1,
            state2,
            m_scene->getAllowedCollisionMatrix());
    if (!res.collision || (req.contacts && res.contacts.size() < req.max_contacts)) {
        auto cr = scene_nonconst-> getCollisionRobotNonConst();
        cr->checkSelfCollision(
                req, res, state1, state2, m_scene->getAllowedCollisionMatrix());
    }

    return !res.collision;
}

auto MoveItCollisionChecker::checkInterpolatedPathCollision(
    const sbpl::motion::RobotState& start,
    const sbpl::motion::RobotState& finish)
    -> bool
{
    int waypoint_count = interpolatePathFast(start, finish, m_waypoint_path);
    if (waypoint_count < 0) {
        return false;
    }

    for (int widx = 0; widx < waypoint_count; ++widx) {
        const smpl::RobotState& p = m_waypoint_path[widx];
        if (!isStateValid(p, false)) {
            return false;
        }
    }

    return true;
}

auto MoveItCollisionChecker::checkContinuousCollision(
        const sbpl::motion::RobotState& start,
        const sbpl::motion::RobotState& finish,
        double& distToObst, int& distToObstCells)
        -> bool
{
    collision_detection::CollisionRequest req;
    req.verbose = false;
    req.group_name = m_robot_model->planningGroupName();
    req.distance;
    collision_detection::CollisionResult res;

    auto cw = m_scene->getCollisionWorld();
    moveit::core::RobotState state1(*m_ref_state);
    moveit::core::RobotState state2(*m_ref_state);
    setRobotStateFromState(state1, start);
    setRobotStateFromState(state2, finish);
    planning_scene::PlanningScene* scene_nonconst = const_cast <planning_scene::PlanningScene*> (m_scene.get());
    cw->checkRobotCollision(
            req,
            res,
            *m_scene->getCollisionRobot(),
            state1,
            state2,
            m_scene->getAllowedCollisionMatrix());
    if (!res.collision || (req.contacts && res.contacts.size() < req.max_contacts)) {
        auto cr = scene_nonconst-> getCollisionRobotNonConst();
        cr->checkSelfCollision(
                req, res, state1, state2, m_scene->getAllowedCollisionMatrix());
    }
    distToObst = res.distance;
    collision_detection::GridWorld* grid_world = dynamic_cast< collision_detection::GridWorld*>(scene_nonconst->getWorldNonConst().get());
     if(grid_world && distToObst!=-1 && distToObst<clearance_threshold_)
    {
        distToObstCells = ceil(distToObst/grid_world->grid()->resolution())*100;
        distToObstCells = 1000 - distToObstCells;
    }
    return !res.collision;
}

auto MoveItCollisionChecker::checkInterpolatedPathCollision(
    const sbpl::motion::RobotState& start,
    const sbpl::motion::RobotState& finish,
    double& distToObst, int& distToObstCells)
    -> bool
    {
    int waypoint_count = interpolatePathFast(start, finish, m_waypoint_path);
    if (waypoint_count < 0) {
        return false;
    }
    
    for (int widx = 0; widx < waypoint_count; ++widx) {
        const smpl::RobotState& p = m_waypoint_path[widx];
        
        if(widx==waypoint_count-1)
        {
            bool result = isStateValid(p, distToObst,true);
            planning_scene::PlanningScene* scene_nonconst = const_cast <planning_scene::PlanningScene*> (m_scene.get());
            collision_detection::GridWorld* grid_world = dynamic_cast< collision_detection::GridWorld*>(scene_nonconst->getWorldNonConst().get());
            if(grid_world && distToObst!=-1 && distToObst<clearance_threshold_)
            {
                distToObstCells = ceil(distToObst/grid_world->grid()->resolution())*100;
                distToObstCells = 1000 - distToObstCells;
                //ROS_ERROR_STREAM("  Dist to obst "<<distToObst<<" and cells "<<distToObstCells);
            }

            if (!result) {
                 return false;
            }
        }
        else
        {
            if (!isStateValid(p, false)) {
            return false;
            }
        }
        
    }
    

    return true;
}

void MoveItCollisionChecker::setRobotStateFromState(
    moveit::core::RobotState& robot_state,
    const smpl::RobotState& state) const
{
    assert(state.size() == m_robot_model->activeVariableIndices().size());
    for (size_t vidx = 0; vidx < state.size(); ++vidx) {
        int avidx = m_robot_model->activeVariableIndices()[vidx];
        robot_state.setVariablePosition(avidx, state[vidx]);
    }
}

int MoveItCollisionChecker::interpolatePathFast(
    const smpl::RobotState& start,
    const smpl::RobotState& finish,
    std::vector<smpl::RobotState>& opath)
{
    assert(start.size() == m_robot_model->activeVariableCount() &&
            finish.size() == m_robot_model->activeVariableCount());

    // check joint limits on the start and finish points
    if (!(m_robot_model->checkJointLimits(start) &&
            m_robot_model->checkJointLimits(finish)))
    {
        ROS_ERROR("Joint limits violated");
        return -1;
    }

    // compute distance traveled by each joint
    m_diffs.resize(m_robot_model->activeVariableCount(), 0.0);
    for (size_t vidx = 0; vidx < m_robot_model->activeVariableCount(); ++vidx) {
        if (m_robot_model->variableContinuous()[vidx]) {
            m_diffs[vidx] = sbpl::angles::shortest_angle_diff(finish[vidx], start[vidx]);
        }
        else {
            m_diffs[vidx] = finish[vidx] - start[vidx];
        }
    }

    // compute the number of intermediate waypoints including start and end
    int waypoint_count = 0;
    for (size_t vidx = 0; vidx < m_robot_model->activeVariableCount(); vidx++) {
        int angle_waypoints = (int)(std::fabs(m_diffs[vidx]) / m_var_incs[vidx]) + 1;
        waypoint_count = std::max(waypoint_count, angle_waypoints);
    }
    waypoint_count = std::max(waypoint_count, 2);

    // fill intermediate waypoints
    const int prev_size = (int)opath.size();
    if (waypoint_count > prev_size) {
        opath.resize(waypoint_count, m_zero_state);
    }
    for (size_t widx = 0; widx < waypoint_count; ++widx) {
        for (size_t vidx = 0; vidx < m_robot_model->activeVariableCount(); ++vidx) {
            double alpha = (double)widx / (double)(waypoint_count - 1);
            double pos = start[vidx] + alpha * m_diffs[vidx];
            opath[widx][vidx] = pos;
        }
    }

    // normalize output continuous variables
    for (size_t vidx = 0; vidx < m_robot_model->activeVariableCount(); ++vidx) {
        if (m_robot_model->variableContinuous()[vidx]) {
            for (size_t widx = 0; widx < waypoint_count; ++widx) {
                opath[widx][vidx] = sbpl::angles::normalize_angle(opath[widx][vidx]);
            }
        }
    }

    return waypoint_count;
}

auto MoveItCollisionChecker::getCollisionModelVisualization(
    const smpl::RobotState& state)
    -> std::vector<sbpl::visual::Marker>
{
    moveit::core::RobotState robot_state(*m_ref_state);

    setRobotStateFromState(robot_state, state);

    visualization_msgs::MarkerArray marker_arr;
    std_msgs::ColorRGBA color;
    color.r = 0.0;
    color.g = 0.0;
    color.b = 0.8;
    color.a = 0.8;
    robot_state.getRobotMarkers(
            marker_arr,
            m_robot_model->moveitRobotModel()->getLinkModelNames(),
            color,
            "",
            ros::Duration(0),
            true);

    auto* tip_link = m_robot_model->planningTipLink();
    if (tip_link) {
        auto& T_model_tip = robot_state.getGlobalLinkTransform(tip_link->getName());
        auto frame_markers = viz::getFrameMarkerArray(
                T_model_tip, m_robot_model->moveitRobotModel()->getModelFrame(), "", marker_arr.markers.size());
        marker_arr.markers.insert(marker_arr.markers.end(), frame_markers.markers.begin(), frame_markers.markers.end());
    }

    std::vector<sbpl::visual::Marker> markers;
    markers.reserve(marker_arr.markers.size());
    for (auto& mm : marker_arr.markers) {
        sbpl::visual::Marker m;
        sbpl::visual::ConvertMarkerMsgToMarker(mm, m);
        markers.push_back(std::move(m));
    }

    return markers;
}

} // namespace sbpl_interface
