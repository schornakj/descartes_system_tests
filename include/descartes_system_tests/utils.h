#ifndef DESCARTES_AMPLING_TEST_UTILS_H
#define DESCARTES_AMPLING_TEST_UTILS_H

#include <Eigen/Eigen>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tesseract/tesseract.h>
#include <tesseract_environment/kdl/kdl_env.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_rosutils/utils.h>

#include <tf2_eigen/tf2_eigen.h>
#include <descartes_system_tests/message_serialization/geometry_msgs_yaml.h>

#include <descartes_light/descartes_light.h>
#include <descartes_tesseract/descartes_tesseract_collision_checker.h>
#include <descartes_samplers/evaluators/euclidean_distance_edge_evaluator.h>
#include <descartes_samplers/samplers/axial_symmetric_sampler.h>
#include <descartes_samplers/samplers/fixed_joint_pose_sampler.h>

#include <ur_ikfast_kinematics/descartes_ikfast_ur10e.h>

#include <yaml-cpp/yaml.h>
#include <thread>

namespace descartes_system_tests
{
static const std::size_t N_DOFS = 6;
static const std::double_t ANGULAR_STEP_INCREMENT = M_PI/60.0;

bool parseWaypointsFromYaml(const YAML::Node& waypoints, const std::string& waypoint_origin_frame, std::vector<geometry_msgs::PoseStamped> &surface_poses)
{
  for (YAML::const_iterator pose_it = waypoints.begin(); pose_it != waypoints.end(); ++pose_it)
  {
     const YAML::Node& pose = *pose_it;
     geometry_msgs::PoseStamped current_pose;
     current_pose.pose = pose["pose"].as<geometry_msgs::Pose>();
     current_pose.header.frame_id = waypoint_origin_frame;
     surface_poses.push_back(current_pose);
  }
  return true;
}

bool planDescartesSeedTraj(const tesseract_kinematics::ForwardKinematics::ConstPtr kin,
                           const tesseract_environment::Environment::Ptr env,
                           const Eigen::Isometry3d &world_to_base_link,
                           const Eigen::Isometry3d &tool0_to_tcp,
                           const std::vector<geometry_msgs::PoseStamped> &waypoints_world_frame,
                           const std::size_t num_threads,
                           Eigen::MatrixXd& joint_traj_eigen_out,
                           std::vector<std::size_t>& failed_edges_out,
                           std::vector<std::size_t>& failed_vertices_out)
{
    std::vector<double> joints_min;
    std::vector<double> joints_max;

    Eigen::MatrixX2d joint_limits = kin->getLimits();
    for (int i = 0; i < joint_limits.rows(); ++i)
    {
      joints_min.push_back(joint_limits(i,0));
      joints_max.push_back(joint_limits(i,1));
    }

    descartes_light::KinematicsInterfaceD::Ptr kin_interface =
      std::make_shared<ur_ikfast_kinematics::UR10eKinematicsD>(world_to_base_link,
                                                        tool0_to_tcp,
                                                        nullptr,
                                                        nullptr);

    tesseract_environment::AdjacencyMap::Ptr adjacency_map =
      std::make_shared<tesseract_environment::AdjacencyMap>(env->getSceneGraph(),
                                                            kin->getActiveLinkNames(),
                                                            env->getCurrentState()->transforms);

    auto collision_checker = std::make_shared<descartes_light::TesseractCollision<double>>(env, adjacency_map->getActiveLinkNames(), kin->getJointNames());

    std::vector<descartes_light::PositionSampler<double>::Ptr> sampler_result;

    for (int i = 0; i < waypoints_world_frame.size(); ++i)
    {
      Eigen::Isometry3d current_waypoint_pose;
      tf2::fromMsg(waypoints_world_frame[i].pose, current_waypoint_pose);
      auto sampler = std::make_shared<descartes_light::AxialSymmetricSampler<double>>(current_waypoint_pose,
                                                                                      kin_interface,
                                                                                      ANGULAR_STEP_INCREMENT,
                                                                                      std::shared_ptr<descartes_light::CollisionInterface<double>>(collision_checker->clone()),
                                                                                      false);
      sampler_result.push_back(sampler);
    }

    auto euclidean_edge_eval = std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluatorD>(N_DOFS);
    descartes_light::Solver<double>graph_builder(N_DOFS);
    if (!graph_builder.build(sampler_result,
                             std::vector<descartes_core::TimingConstraint<double>>(sampler_result.size(), std::numeric_limits<double>::max()),
                             euclidean_edge_eval,
                             num_threads))
    {
        failed_edges_out = graph_builder.getFailedEdges();
        failed_vertices_out = graph_builder.getFailedVertices();
        return false;
    }

    std::vector<double> solution;
    if (!graph_builder.search(solution))
    {
        return false;
    }

    Eigen::Map<Eigen::VectorXd> solution_vec (&solution[0], solution.size());
    Eigen::VectorXd seed_traj(solution_vec.size());
    seed_traj << solution_vec;
    int n_rows = seed_traj.size() / N_DOFS;
    joint_traj_eigen_out = Eigen::Map<Eigen::MatrixXd>(seed_traj.data(), N_DOFS, n_rows).transpose();
    return true;
}
} // namespace descartes_sampling_test

#endif // DESCARTES_AMPLING_TEST_UTILS_H
