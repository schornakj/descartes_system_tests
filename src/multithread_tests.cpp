#include <gtest/gtest.h>

#include <descartes_system_tests/utils.h>

// This test shows that the Descartes sampling process succeeds if the collision object is not present in the environment.
TEST(DescartesMultithreadUnitFree, Instantiation)
{
    // The environment consists of a UR-10e with a boxy end effector and no other collision objects.
    std::stringstream urdf_xml_string, srdf_xml_string;
    std::ifstream urdf_in(tesseract_rosutils::locateResource("package://descartes_system_tests/support/urdf/ur10e_no_obstacle.urdf"));
    urdf_xml_string << urdf_in.rdbuf();
    std::ifstream srdf_in(tesseract_rosutils::locateResource("package://descartes_system_tests/support/urdf/ur10e.srdf"));
    srdf_xml_string << srdf_in.rdbuf();

    // All these waypoints are within the collision bounday, so we expect Descartes sampling to always fail for every waypoint.
    std::vector<geometry_msgs::PoseStamped> waypoints;
    descartes_system_tests::parseWaypointsFromYaml(YAML::LoadFile(tesseract_rosutils::locateResource("package://descartes_system_tests/support/data/waypoints.yaml")), "world", waypoints);

    tesseract::Tesseract::Ptr tesseract_handle(new tesseract::Tesseract());
    tesseract_handle->init(urdf_xml_string.str(), srdf_xml_string.str(), tesseract_rosutils::locateResource);

    const tesseract_common::TransformMap current_transforms = tesseract_handle->getEnvironmentConst()->getCurrentState()->transforms;
    const tesseract_environment::Environment::Ptr env = tesseract_handle->getEnvironment();
    const tesseract_kinematics::ForwardKinematics::ConstPtr kin = tesseract_handle->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator");

    const Eigen::Isometry3d world_to_tcp = current_transforms.find("tcp")->second;
    const Eigen::Isometry3d world_to_base_link = current_transforms.find("base_link")->second;
    const Eigen::Isometry3d world_to_tool0 = current_transforms.find("tool0")->second;
    const Eigen::Isometry3d tool0_to_tcp = world_to_tool0.inverse() * world_to_tcp;

    // Do Descartes sampling using 1 thread
    Eigen::MatrixXd seed_traj_single_thread;
    std::vector<std::size_t> failed_edges_single_thread;
    std::vector<std::size_t> failed_vertices_single_thread;
    bool res_singlethread = descartes_system_tests::planDescartesSeedTraj(kin, env, world_to_base_link, tool0_to_tcp, waypoints, 1, seed_traj_single_thread, failed_edges_single_thread, failed_vertices_single_thread);

    // Expect single-thread case to succeed
    EXPECT_TRUE(res_singlethread);

    // Expect all vertices and edges to succeed in single-thread case
    EXPECT_EQ(failed_vertices_single_thread.size(), 0);
    EXPECT_EQ(failed_edges_single_thread.size(), 0);

    // Do Descartes sampling using 12 threads
    Eigen::MatrixXd seed_traj_multi_thread;
    std::vector<std::size_t> failed_edges_multi_thread;
    std::vector<std::size_t> failed_vertices_multi_thread;
    bool res_multithread = descartes_system_tests::planDescartesSeedTraj(kin, env, world_to_base_link, tool0_to_tcp, waypoints, 12, seed_traj_multi_thread, failed_edges_multi_thread, failed_vertices_multi_thread);

    // Expect single-thread case to succeed
    EXPECT_TRUE(res_multithread);

    // Expect all vertices and edges to succeed in multi-thread case
    EXPECT_EQ(failed_vertices_multi_thread.size(), 0);
    EXPECT_EQ(failed_edges_multi_thread.size(), 0);

    // Expect single-thread and multi-thread to produce the same results
    EXPECT_EQ(failed_vertices_single_thread.size(), failed_vertices_multi_thread.size());
    EXPECT_EQ(failed_edges_single_thread.size(), failed_edges_multi_thread.size());
}

// This test should show that the Descartes sampling process fails when the collision object is present.
TEST(DescartesMultithreadUnitCollision, Instantiation)
{
    // The environment consists of a UR-10e with a boxy end effector and a large collision boundary.
    std::stringstream urdf_xml_string, srdf_xml_string;
    std::ifstream urdf_in(tesseract_rosutils::locateResource("package://descartes_system_tests/support/urdf/ur10e_with_obstacle.urdf"));
    urdf_xml_string << urdf_in.rdbuf();
    std::ifstream srdf_in(tesseract_rosutils::locateResource("package://descartes_system_tests/support/urdf/ur10e.srdf"));
    srdf_xml_string << srdf_in.rdbuf();

    // All these waypoints are within the collision bounday, so we expect Descartes sampling to always fail for every waypoint.
    std::vector<geometry_msgs::PoseStamped> waypoints;
    descartes_system_tests::parseWaypointsFromYaml(YAML::LoadFile(tesseract_rosutils::locateResource("package://descartes_system_tests/support/data/waypoints.yaml")), "world", waypoints);

    tesseract::Tesseract::Ptr tesseract_handle(new tesseract::Tesseract());
    tesseract_handle->init(urdf_xml_string.str(), srdf_xml_string.str(), tesseract_rosutils::locateResource);

    const tesseract_common::TransformMap current_transforms = tesseract_handle->getEnvironmentConst()->getCurrentState()->transforms;
    const tesseract_environment::Environment::Ptr env = tesseract_handle->getEnvironment();
    const tesseract_kinematics::ForwardKinematics::ConstPtr kin = tesseract_handle->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator");

    const Eigen::Isometry3d world_to_tcp = current_transforms.find("tcp")->second;
    const Eigen::Isometry3d world_to_base_link = current_transforms.find("base_link")->second;
    const Eigen::Isometry3d world_to_tool0 = current_transforms.find("tool0")->second;
    const Eigen::Isometry3d tool0_to_tcp = world_to_tool0.inverse() * world_to_tcp;

    // Do Descartes sampling using 1 thread
    Eigen::MatrixXd seed_traj_single_thread;
    std::vector<std::size_t> failed_edges_single_thread;
    std::vector<std::size_t> failed_vertices_single_thread;
    bool res_singlethread = descartes_system_tests::planDescartesSeedTraj(kin, env, world_to_base_link, tool0_to_tcp, waypoints, 1, seed_traj_single_thread, failed_edges_single_thread, failed_vertices_single_thread);

    // Expect failure for single-thread case
    EXPECT_FALSE(res_singlethread);

    // Expect all vertices and edges to fail in single-thread case
    EXPECT_EQ(failed_vertices_single_thread.size(), waypoints.size());
    EXPECT_EQ(failed_edges_single_thread.size(), waypoints.size() - 1);

    // Do Descartes sampling using 12 threads
    Eigen::MatrixXd seed_traj_multi_thread;
    std::vector<std::size_t> failed_edges_multi_thread;
    std::vector<std::size_t> failed_vertices_multi_thread;
    bool res_multithread = descartes_system_tests::planDescartesSeedTraj(kin, env, world_to_base_link, tool0_to_tcp, waypoints, 12, seed_traj_multi_thread, failed_edges_multi_thread, failed_vertices_multi_thread);

    // Expect failure for multi-thread case
    EXPECT_FALSE(res_multithread);

    // Expect all vertices and edges to fail in multi-thread case
    EXPECT_EQ(failed_vertices_multi_thread.size(), waypoints.size());
    EXPECT_EQ(failed_edges_multi_thread.size(), waypoints.size() - 1);

    // Expect single-thread and multi-thread to produce the same results
    EXPECT_EQ(failed_vertices_single_thread.size(), failed_vertices_multi_thread.size());
    EXPECT_EQ(failed_edges_single_thread.size(), failed_edges_multi_thread.size());
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
