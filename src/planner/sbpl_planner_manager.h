#ifndef sbpl_interface_sbpl_planner_manager_h
#define sbpl_interface_sbpl_planner_manager_h

// system includes
#include <XmlRpcValue.h>
#include <moveit/macros/class_forward.h>
#include <moveit/planning_interface/planning_interface.h>
#include <smpl/debug/visualizer_ros.h>
#include "sbpl_planning_context.h"
// project includes
#include <moveit_planners_sbpl/planner/moveit_robot_model.h>

namespace sbpl_interface {

class SBPLPlannerManager : public planning_interface::PlannerManager
{
public:

    static const std::string DefaultPlanningAlgorithm;

    typedef planning_interface::PlannerManager Base;

    SBPLPlannerManager();
    virtual ~SBPLPlannerManager();

    /// \name planning_interface::PlannerManager API Requirements
    ///@{

    /// \sa planning_interface::PlannerManager::initialize()
    virtual bool initialize(
        const robot_model::RobotModelConstPtr& model,
        const std::string& ns) override;

    /// \sa planning_interface::PlannerManger::getDescription()
    virtual std::string getDescription() const;

    /// \sa planning_interface::PlannerManager:::getPlanningAlgorithms()
    virtual void getPlanningAlgorithms(
        std::vector<std::string>& algs) const override;

    /// \sa planning_interface::PlannerManager::getPlanningContext()
    virtual planning_interface::PlanningContextPtr getPlanningContext(
        const planning_scene::PlanningSceneConstPtr& planning_scene,
        const planning_interface::MotionPlanRequest& req,
        moveit_msgs::MoveItErrorCodes& error_code) const override;

    /// \sa planning_interface::PlannerManager::canServiceRequest()
    virtual bool canServiceRequest(
        const planning_interface::MotionPlanRequest& req) const override;

    /// \sa planning_interface::PlannerManager::setPlannerConfigurations()
    virtual void setPlannerConfigurations(
        const planning_interface::PlannerConfigurationMap& pcs) override;

    ///@}

private:

    std::map<std::string, SBPLPlanningContextPtr> planning_contexts_;

    moveit::core::RobotModelConstPtr m_robot_model;

    // per-group sbpl robot model
    std::map<std::string, std::shared_ptr<MoveItRobotModel>> m_sbpl_models;

    sbpl::VisualizerROS m_viz;

    planning_interface::PlannerConfigurationMap map;

    std::map<int, sbpl_interface::SBPLPlanningContext*> m_sbpl_context;

    void logPlanningScene(const planning_scene::PlanningScene& scene) const;
    void logMotionPlanRequest(
        const planning_interface::MotionPlanRequest& req) const;

    /// \name Parameter Loading
    ///@{

    bool loadPlannerConfigurationMapping(
        const ros::NodeHandle& nh,
        const moveit::core::RobotModel& model);

    // key/value pairs for parameters to pass down to planner
    typedef std::map<std::string, std::string> PlannerSettings;

    // config name -> (string -> string)
    typedef std::map<std::string, PlannerSettings> PlannerSettingsMap;

    bool loadSettingsMap(
        const ros::NodeHandle& nh,
        const std::string& param_name,
        PlannerSettingsMap& settings);
    ///@}

    // retrive an already-initialized model for a given group
    MoveItRobotModel* getModelForGroup(const std::string& group_name);

    std::string selectPlanningLink(
        const planning_interface::MotionPlanRequest& req) const;

    bool xmlToString(XmlRpc::XmlRpcValue& value, std::string& out) const;
};

MOVEIT_CLASS_FORWARD(SBPLPlannerManager);

} // namespace sbpl_interface

#endif
