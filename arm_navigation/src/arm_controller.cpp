//TODO: Ensure IK solution is being generated correctly
//TODO: Ensure Trajopt is generating solution correctly
//TODO: Ensure trajectory is being converted correctly


//Note: this set of files can cause the compiler to segfault if using a multithreaded catkin build
//#include "arm_controller.h"

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <jsoncpp/json/json.h>
#include <ros/ros.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <tesseract_ros/kdl/kdl_chain_kin.h>
#include <tesseract_ros/kdl/kdl_env.h>
#include <tesseract_ros/ros_basic_plotting.h>
#include <trajopt/plot_callback.hpp>
#include <trajopt/file_write_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <tf/transform_broadcaster.h>
#include "control_msgs/FollowJointTrajectoryActionGoal.h"

using namespace trajopt;
using namespace tesseract;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic"; /**< Default ROS parameter for robot
                                                                          description */
const std::string TRAJOPT_DESCRIPTION_PARAM =
        "trajopt_description"; /**< Default ROS parameter for trajopt description */

static bool plotting_ = false;
static bool write_to_file_ = false;
static int steps_ = 10;
static std::string method_ = "cpp";
static urdf::ModelInterfaceSharedPtr urdf_model_; /**< URDF Model */
static srdf::ModelSharedPtr srdf_model_;          /**< SRDF Model */
static tesseract_ros::KDLEnvPtr env_;             /**< Trajopt Basic Environment */
//static tf::TransformBroadcaster broadcaster;



TrajOptProbPtr cppMethod()
{
    //Setup pci
    ProblemConstructionInfo pci(env_);
    ROS_INFO("hmm");
    // Populate Basic Info
    pci.basic_info.n_steps = steps_;
    pci.basic_info.manip = "arm_with_torso";
    pci.basic_info.start_fixed = false;
    pci.basic_info.use_time = false;
    pci.opt_info.cnt_tolerance = .0001;
    pci.opt_info.min_trust_box_size = .0001;
    ROS_INFO("hmm1");



    // Create Kinematic Object
    pci.kin = pci.env->getManipulator(pci.basic_info.manip);
    ROS_INFO_STREAM(pci.kin);
    ROS_INFO("hmm2");

    // Populate Init Info
    Eigen::VectorXd start_pos = pci.env->getCurrentJointValues(pci.kin->getName());
    ROS_INFO_STREAM(start_pos);
    ROS_INFO("start pos acquired");
    Eigen::VectorXd end_pos;

    end_pos.resize(pci.kin->numJoints());
    ROS_INFO("Start pos resized");
    //getPose here
    geometry_msgs::PoseStamped end_pose;
    end_pose.pose.position.x = 0.5;
    end_pose.pose.position.y = 0.5;
    end_pose.pose.position.z = 0.8;
    end_pose.pose.orientation.x = 0.0;
    end_pose.pose.orientation.y = 0.0;
    end_pose.pose.orientation.z = 0.0;
    end_pose.pose.orientation.w = 1.0;

    tf::Transform t;
    tf::poseMsgToTF(end_pose.pose, t);
    tf::TransformBroadcaster broadcaster;
    broadcaster.sendTransform(tf::StampedTransform(t, ros::Time::now(), "base_link", "end_pose"));

    moveit::planning_interface::MoveGroupInterface move_group("arm_with_torso");
   // move_group.Kinematics
    move_group.setEndEffectorLink("gripper_link");
    /*
    moveit::core::RobotState robotState(move_group.getRobotModel());
    const moveit::core::JointModelGroup *jmp = robotState.getJointModelGroup("arm_with_torso");
    moveit::core::GroupStateValidityCallbackFn constraint = moveit::core::GroupStateValidityCallbackFn();
    moveit::core::RobotState::setFromIK(jmp,end_pose,10.0,constraint);
    */
    ROS_INFO("Value test");
    ROS_INFO_STREAM(*move_group.getJointValueTarget().getJointPositions("wrist_flex_joint"));

    //use IK to get joint values for the position of the pose
    move_group.setApproximateJointValueTarget(end_pose,"gripper_link");
    /*
    move_group.setApproximateJointValueTarget(end_pose,"shoulder_pan_joint");
    move_group.setApproximateJointValueTarget(end_pose,"shoulder_lift_joint");
    move_group.setApproximateJointValueTarget(end_pose,"upperarm_roll_joint");
    move_group.setApproximateJointValueTarget(end_pose,"elbow_flex_joint");
    move_group.setApproximateJointValueTarget(end_pose,"forearm_roll_joint");
    move_group.setApproximateJointValueTarget(end_pose,"wrist_flex_joint");
    move_group.setApproximateJointValueTarget(end_pose,"torso_lift_joint");

*/
    ROS_INFO("ik set");
    const double* torsoValue = move_group.getJointValueTarget().getJointPositions("torso_lift_joint");
    const double* shoulderValue = move_group.getJointValueTarget().getJointPositions("shoulder_pan_joint");
    const double* shoulderLiftValue = move_group.getJointValueTarget().getJointPositions("shoulder_lift_joint");
    const double* upperarmRollValue = move_group.getJointValueTarget().getJointPositions("upperarm_roll_joint");
    const double* elbowFlexValue = move_group.getJointValueTarget().getJointPositions("elbow_flex_joint");
    const double* forearmRollValue = move_group.getJointValueTarget().getJointPositions("forearm_roll_joint");
    const double* wristFlexValue = move_group.getJointValueTarget().getJointPositions("wrist_flex_joint");
    const double* wristRollValue = move_group.getJointValueTarget().getJointPositions("wrist_roll_joint");
    //this might be the order it expects
    //end_pos << *shoulderValue, *shoulderLiftValue, *upperarmRollValue, *elbowFlexValue, *forearmRollValue, *wristFlexValue, *wristRollValue;

   end_pos << *torsoValue, *shoulderValue, *shoulderLiftValue, *upperarmRollValue, *elbowFlexValue, *forearmRollValue, *wristFlexValue, *wristRollValue;
    //end_pos << *elbowFlexValue, *forearmRollValue, *shoulderLiftValue, *shoulderValue, *torsoValue, *upperarmRollValue, *wristFlexValue, *wristRollValue;
    ROS_INFO_STREAM(end_pos);


    //end_pos = epos;
    //ROS_INFO_STREAM(*wristFlexValue);

    //end_pos << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;


    ROS_INFO("hmm3");

    pci.init_info.type = InitInfo::GIVEN_TRAJ;
    pci.init_info.data = TrajArray(steps_, pci.kin->numJoints());
    for (unsigned idof = 0; idof < pci.kin->numJoints(); ++idof)
    {
        pci.init_info.data.col(idof) = Eigen::VectorXd::LinSpaced(steps_, start_pos[idof], end_pos[idof]);
        ROS_INFO_STREAM(Eigen::VectorXd::LinSpaced(steps_, start_pos[idof], end_pos[idof]));
    }
    ROS_INFO("hmm4");


    // Populate Cost Info
    std::shared_ptr<JointVelTermInfo> jv = std::shared_ptr<JointVelTermInfo>(new JointVelTermInfo);
    jv->coeffs = std::vector<double>(8, 1.0);
    jv->targets = std::vector<double>(8, 0.0);
    jv->first_step = 0;
    // n_steps-1 ??
    jv->last_step = pci.basic_info.n_steps;
    jv->name = "joint_vel";
    jv->term_type = TT_COST;
    pci.cost_infos.push_back(jv);
    ROS_INFO("hmm5");


    //pci.basic_info.use_time = true;
/*
    std::shared_ptr<trajopt::JointVelTermInfo> jvt(new trajopt::JointVelTermInfo);

    // Taken from documentation (radians/s) and scaled by 0.8, torso in m/s
    double scaleFactor = .9;
    std::vector<double> vel_lower_lim{ .1, 1.25 * -scaleFactor, 1.45 * -scaleFactor, 1.57 * -scaleFactor, 1.52 * -scaleFactor, 1.57 * -scaleFactor, 2.26 * -scaleFactor, 2.26 * -scaleFactor };
    std::vector<double> vel_upper_lim{ .1, 1.25 * scaleFactor, 1.45 * scaleFactor, 1.57 * scaleFactor, 1.52 * scaleFactor, 1.57 * scaleFactor, 2.26 * scaleFactor, 2.26 * scaleFactor};

    jvt->targets = std::vector<double>(8.0, 0.0);
    jvt->coeffs = std::vector<double>(8.0, 10.0);
    jvt->lower_tols = vel_lower_lim;
    jvt->upper_tols = vel_upper_lim;
    jvt->term_type = (trajopt::TT_COST);
    jvt->first_step = 0;
    jvt->last_step = pci.basic_info.n_steps ;
    jvt->name = "joint_velocity_cnt_hmm";
    pci.cost_infos.push_back(jvt);
*/
    ROS_INFO("hmm 5.5");
    /*
    std::shared_ptr<CollisionTermInfo> collision = std::shared_ptr<CollisionTermInfo>(new CollisionTermInfo);
    collision->name = "collision";
    collision->term_type = TT_COST;
    collision->continuous = false;
    collision->first_step = 0;
    collision->last_step = pci.basic_info.n_steps - 1;
    collision->gap = 1;
    collision->info = createSafetyMarginDataVector(pci.basic_info.n_steps, 0.025, 20);

    for (auto& info : collision->info)
    {
        info->SetPairSafetyMarginData("                                    ", "wrist_roll_link", 0.05, 10);
        //info->SetPairSafetyMarginData("link_3", "link_5", 0.01, 10);
        //info->SetPairSafetyMarginData("link_3", "link_6", 0.01, 10);
    }

    pci.cost_infos.push_back(collision);
    */

    ROS_INFO("hmm6");
    geometry_msgs::PoseStamped start_pose = move_group.getCurrentPose();
    tf::Transform tt;
    tf::poseMsgToTF(start_pose.pose, tt);
    broadcaster.sendTransform(tf::StampedTransform(tt, ros::Time::now(), "base_link", "start_pose"));

    // Populate Constraints
    for (auto i = 0; i < pci.basic_info.n_steps; ++i)
    {
        std::shared_ptr<CartPoseTermInfo> pose = std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
        pose->term_type = TT_CNT;
        pose->name = "waypoint_cart_" + std::to_string(i);
        pose->link = "gripper_link";
        pose->timestep = i;
        double x = ((end_pose.pose.position.x - start_pose.pose.position.x)/pci.basic_info.n_steps) * (i+1) + start_pose.pose.position.x;
        double y = ((end_pose.pose.position.y - start_pose.pose.position.y)/pci.basic_info.n_steps) * (i+1) + start_pose.pose.position.y;
        double z = ((end_pose.pose.position.z - start_pose.pose.position.z)/pci.basic_info.n_steps) * (i+1) + start_pose.pose.position.z;
        geometry_msgs::PoseStamped waypoint_pose;
        waypoint_pose.pose.position.x = x;
        waypoint_pose.pose.position.y = y;
        waypoint_pose.pose.position.z = z;
        waypoint_pose.pose.orientation.w = ((end_pose.pose.orientation.w - start_pose.pose.orientation.w)/pci.basic_info.n_steps) * (i+1) + start_pose.pose.orientation.w;
        waypoint_pose.pose.orientation.x = ((end_pose.pose.orientation.x - start_pose.pose.orientation.x)/pci.basic_info.n_steps) * (i+1) + start_pose.pose.orientation.x;
        waypoint_pose.pose.orientation.y = ((end_pose.pose.orientation.y - start_pose.pose.orientation.y)/pci.basic_info.n_steps) * (i+1) + start_pose.pose.orientation.y;
        waypoint_pose.pose.orientation.z = ((end_pose.pose.orientation.z - start_pose.pose.orientation.z)/pci.basic_info.n_steps) * (i+1) + start_pose.pose.orientation.z;

        tf::Transform ttt;
        tf::poseMsgToTF(waypoint_pose.pose, ttt);
        broadcaster.sendTransform(tf::StampedTransform(ttt, ros::Time::now(), "base_link", "waypoint_pose "+i));

       // ROS_INFO_STREAM(x << y << z);

        pose->xyz = Eigen::Vector3d(x, y, z);
        //pose->xyz = Eigen::Vector3d(0.0, 0.0, 0.0);

        pose->wxyz = Eigen::Vector4d(start_pose.pose.orientation.w, start_pose.pose.orientation.x, start_pose.pose.orientation.y, start_pose.pose.orientation.z);
        /*
        if (i == (pci.basic_info.n_steps - 1) || i == 0)
        {
            pose->pos_coeffs = Eigen::Vector3d(10, 10, 10);
            pose->rot_coeffs = Eigen::Vector3d(10, 10, 10);
        }
        else
        {
            pose->pos_coeffs = Eigen::Vector3d(0, 0, 0);
            pose->rot_coeffs = Eigen::Vector3d(10, 10, 0);
        }*/

        pci.cnt_infos.push_back(pose);

    }


    ROS_INFO("hmm7");
    ROS_INFO_STREAM(pci.init_info.data);

    return ConstructProblem(pci);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "glass_up_right_plan");
    ros::NodeHandle pnh("~");
    ros::NodeHandle nh;

    ROS_INFO("launched node");

    tf::TransformBroadcaster broadcaster;

    // Initial setup
    std::string urdf_xml_string, srdf_xml_string;
    nh.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
    ROS_INFO("urdf acquired");

    // !important rosparam set robot_description_semantic -t /home/jdenyse/catkin_ws/src/fetch_ros/fetch_moveit_config/config/fetch.srdf
    nh.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);
    ROS_INFO("srdf acquired");

    urdf_model_ = urdf::parseURDF(urdf_xml_string);

    ROS_INFO("urdf parsed");
    srdf_model_ = srdf::ModelSharedPtr(new srdf::Model);
    ROS_INFO("srdf MSP init");
    srdf_model_->initString(*urdf_model_, srdf_xml_string);
    ROS_INFO("srdf initString");
    env_ = tesseract_ros::KDLEnvPtr(new tesseract_ros::KDLEnv);
    assert(urdf_model_ != nullptr);
    assert(env_ != nullptr);
    ROS_INFO("prepped for env init");

    bool success = env_->init(urdf_model_, srdf_model_);
    assert(success);
    ROS_INFO("env init");

    // Create Plotting tool
    tesseract_ros::ROSBasicPlottingPtr plotter(new tesseract_ros::ROSBasicPlotting(env_));

    ROS_INFO("plotter created");
    // Add sphere
    AttachableObjectPtr obj(new AttachableObject());
    std::shared_ptr<shapes::Sphere> sphere(new shapes::Sphere());
    Eigen::Isometry3d sphere_pose;

    sphere->radius = 0.15;
    sphere_pose.setIdentity();
    sphere_pose.translation() = Eigen::Vector3d(0.5, 0, 0.55);

    ROS_INFO("pose translated");
    /*
    obj->name = "sphere_attached";
    obj->visual.shapes.push_back(sphere);
    obj->visual.shape_poses.push_back(sphere_pose);
    obj->collision.shapes.push_back(sphere);
    obj->collision.shape_poses.push_back(sphere_pose);
    obj->collision.collision_object_types.push_back(CollisionObjectType::UseShapeType);

    env_->addAttachableObject(obj);

    AttachedBodyInfo attached_body;
    attached_body.object_name = "sphere_attached";
    attached_body.parent_link_name = "base_link";
    attached_body.transform.setIdentity();
    //  attached_body.touch_links = {}; // This element enables the attached body
    //  to collide with other links

    env_->attachBody(attached_body);
*/
    //!important rosparam load kinematics.yaml
    // Get ROS Parameters
    pnh.param("plotting", plotting_, plotting_);
    pnh.param("write_to_file", write_to_file_, write_to_file_);
    pnh.param<std::string>("method", method_, method_);
    pnh.param<int>("steps", steps_, steps_);

    ROS_INFO("params acquired");
    // Set the robot initial state

    moveit::planning_interface::MoveGroupInterface move_group("arm_with_torso");
    ROS_INFO("HEY");
    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ROS_INFO("move_group created");
    moveit::core::RobotState robotState(move_group.getRobotModel());

    ROS_INFO("robot state acquired");
    //robotState.buildLinkModel();
    //auto hmm = *robotState.getJointPositions("torso_lift_joint");
    //hmm = round( hmm * 1000.0 ) / 1000.0;
    //ROS_INFO_STREAM(hmm);
    ros::AsyncSpinner spinner(1); spinner.start();
    std::vector<double> jointValues = move_group.getCurrentJointValues();

    std::unordered_map<std::string, double> ipos;


    ipos["torso_lift_joint"] = jointValues.at(0);
    ipos["shoulder_pan_joint"] = jointValues.at(1);
    ipos["shoulder_lift_joint"] = jointValues.at(2);
    ipos["upperarm_roll_joint"] = jointValues.at(3);
    ipos["elbow_flex_joint"] = jointValues.at(4);
    ipos["forearm_roll_joint"] = jointValues.at(5);
    ipos["wrist_flex_joint"] = jointValues.at(6);
    ipos["wrist_roll_joint"] = jointValues.at(7);


    //create a vector with the names of the joints for later use
    std::vector<std::string> joint_names;
    joint_names.push_back("torso_lift_joint");
    joint_names.push_back("shoulder_pan_joint");
    joint_names.push_back("shoulder_lift_joint");
    joint_names.push_back("upperarm_roll_joint");
    joint_names.push_back("elbow_flex_joint");
    joint_names.push_back("forearm_roll_joint");
    joint_names.push_back("wrist_flex_joint");
    joint_names.push_back("wrist_roll_joint");
    //joint_names.push_back("dummy_joint");

    //populate initial positions of joints from robot state
    /*
    for(int i = 0; i > -1; i--){
        ipos[joint_names.at(i)] = jointValues.at(i);
    }
*/
    env_->setState(ipos);

    plotter->plotScene();
    ROS_INFO("scene plotted");
    // Set Log Level
    util::gLogLevel = util::LevelInfo;
    ROS_INFO("log level set");
    // Setup Problem
    TrajOptProbPtr prob;
    ROS_INFO("about to call cppMethod");
    prob = cppMethod();
    ROS_INFO("problem setup");
    // Solve Trajectory
    ROS_INFO("glass upright plan example");
/*
    std::vector<tesseract::ContactResultMap> collisions;
    ContinuousContactManagerBasePtr manager = prob->GetEnv()->getContinuousContactManager();
    manager->setActiveCollisionObjects(prob->GetKin()->getLinkNames());
    manager->setContactDistanceThreshold(0);

    bool found = tesseract::continuousCollisionCheckTrajectory(
            *manager, *prob->GetEnv(), *prob->GetKin(), prob->GetInitTraj(), collisions);

    ROS_INFO((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free"));
*/
    sco::BasicTrustRegionSQP opt(prob);
    if (plotting_)
    {
        opt.addCallback(PlotCallback(*prob, plotter));
    }
    ROS_INFO("BTR setup");
    std::shared_ptr<std::ofstream> stream_ptr;
    if (write_to_file_)
    {
        // Create file write callback discarding any of the file's current contents
        stream_ptr.reset(new std::ofstream);
        std::string path = ros::package::getPath("trajopt") + "/scripts/glass_up_right_plan.csv";
        stream_ptr->open(path, std::ofstream::out | std::ofstream::trunc);
        opt.addCallback(trajopt::WriteCallback(stream_ptr, prob));
        ROS_INFO("wrote to file");
    }

    opt.initialize(trajToDblVec(prob->GetInitTraj()));
    ros::Time tStart = ros::Time::now();
    ROS_INFO("ready to optimize");
    opt.optimize();
    ROS_ERROR("planning time: %.3f", (ros::Time::now() - tStart).toSec());
    auto x = (opt.x());
    double d = 0;
    TrajArray traj = getTraj(opt.x(), prob->GetVars());
    for (unsigned i = 1; i < traj.rows(); ++i)
    {
        for (unsigned j = 0; j < traj.cols(); ++j)
        {
            d += std::abs(traj(i, j) - traj(i - 1, j));
            ros::spinOnce();
        }
    }

    /*
    for(int i = 0; i < 9; i++){
        ROS_INFO_STREAM(traj(i,

        ));
    }
    */
    //Convert TrajArray trajectory into trajectory msgs robot trajectory
    trajectory_msgs::JointTrajectory moveitTrajectory;
    //tesseract::tesseract_ros::tesseractTrajectoryToJointTrajectoryMsg()
    //Eigen::Ref<TrajArray>& trajEigen = new Eigen::Ref<TrajArray traj>;
    const Eigen::Ref<const TrajArray> eigenRef = traj;
    ROS_INFO_STREAM(eigenRef.cols());
    tesseract::tesseract_ros::tesseractTrajectoryToJointTrajectoryMsg(moveitTrajectory, joint_names, eigenRef);

    moveit_msgs::RobotTrajectory robotTrajectory;

    //this init point, try to merge into earlier stuff?

    trajectory_msgs::JointTrajectoryPoint initPoint;
    for(int i = 0; i < jointValues.size(); i++){
        initPoint.positions.push_back(jointValues.at(i));
    }
    robotTrajectory.joint_trajectory.points.push_back(initPoint);

    for(int i = 0; i < moveitTrajectory.points.size(); i++){
        ++moveitTrajectory.points.at(i).time_from_start.sec;
        robotTrajectory.joint_trajectory.points.push_back(moveitTrajectory.points.at(i));
    }
    /*
    trajectory_msgs::JointTrajectoryPoint finalPoint;
    for(int i = 0; i < jointValues.size(); i++){
        finalPoint.positions.push_back(endPosGetter(move_group)(i));
        finalPoint.time_from_start.sec = jointValues.size() + 2.0;
    }

    robotTrajectory.joint_trajectory.points.push_back(finalPoint);
*/
    robotTrajectory.joint_trajectory.joint_names = joint_names;

    //comment this
    //robotTrajectory.joint_trajectory = moveitTrajectory;

    moveit::planning_interface::MoveGroupInterface::Plan thePlan;

    //Convert moveit::core::RobotState to moveit_msgs::RobotState

    move_group.setGoalJointTolerance(1.0);

    moveit_msgs::RobotState msgsRobotState;
    moveit::core::robotStateToRobotStateMsg(robotState, msgsRobotState, true);

    thePlan.start_state_ = msgsRobotState;
    thePlan.trajectory_ = robotTrajectory;

    //WIP attempt to directly execute trajectory
    //ros::Publisher goal_pub = nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/arm_controller/follow_joint_trajectory/goal", 1000);
    //control_msgs::FollowJointTrajectoryActionGoal ourGoal;
    //ourGoal.goal.trajectory = moveitTrajectory;
    //ros::spin();

    ROS_ERROR("trajectory norm: %.3f", d);

    if (plotting_)
    {
        plotter->clear();
    }
    if (write_to_file_)
    {
        stream_ptr->close();
        ROS_INFO("Data written to file. Evaluate using scripts in trajopt/scripts.");
    }

    move_group.execute(thePlan);

    ROS_INFO("end");
    /*
    collisions.clear();
    found = tesseract::continuousCollisionCheckTrajectory(
            *manager, *prob->GetEnv(), *prob->GetKin(), prob->GetInitTraj(), collisions);

    ROS_INFO((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));
     */
}



