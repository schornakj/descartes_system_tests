#include <gtest/gtest.h>

#include <descartes_system_tests/utils.h>

TEST(DescartesMultithreadUnitNearMultibodyPrimitive, Instantiation)
{
    // The environment consists of a UR-10e with a boxy end effector near a "wall" and a "shelf". The geometry is defined in the URDF using box primitives.
    std::stringstream urdf_xml_string, srdf_xml_string;
    std::ifstream urdf_in(tesseract_rosutils::locateResource("package://descartes_system_tests/support/urdf/ur10e_near_obstacle_primitive.urdf"));
    urdf_xml_string << urdf_in.rdbuf();
    std::ifstream srdf_in(tesseract_rosutils::locateResource("package://descartes_system_tests/support/urdf/ur10e.srdf"));
    srdf_xml_string << srdf_in.rdbuf();

    // The waypoints run along the top edge of the shelf. We expect the waypoints to be successfully sampled and to produce a valid series of poses.
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

    // Do Descartes sampling using 12 threads
    Eigen::MatrixXd seed_traj_multi_thread;
    std::vector<std::size_t> failed_edges_multi_thread;
    std::vector<std::size_t> failed_vertices_multi_thread;
    bool res_multithread = descartes_system_tests::planDescartesSeedTraj(kin, env, world_to_base_link, tool0_to_tcp, waypoints, 12, seed_traj_multi_thread, failed_edges_multi_thread, failed_vertices_multi_thread);

    // Expect success.
    EXPECT_TRUE(res_multithread);

    // Expect no vertices or edges to fail.
    EXPECT_EQ(failed_vertices_multi_thread.size(), 0);
    EXPECT_EQ(failed_edges_multi_thread.size(), 0);
}

TEST(DescartesMultithreadUnitNearMultibodyDAE, Instantiation)
{
    // The environment consists of a UR-10e with a boxy end effector near a "wall" and a "shelf". The geometry is defined in the URDF using a multibody mesh loaded from a .dae.
    // The geometry is identical to the environment in DescartesMultithreadUnitNearMultibodyPrimitive, so we expect planning to succeed.
    std::stringstream urdf_xml_string, srdf_xml_string;
    std::ifstream urdf_in(tesseract_rosutils::locateResource("package://descartes_system_tests/support/urdf/ur10e_near_obstacle_multibody_dae.urdf"));
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

    // Do Descartes sampling using 12 threads
    Eigen::MatrixXd seed_traj_multi_thread;
    std::vector<std::size_t> failed_edges_multi_thread;
    std::vector<std::size_t> failed_vertices_multi_thread;
    bool res_multithread = descartes_system_tests::planDescartesSeedTraj(kin, env, world_to_base_link, tool0_to_tcp, waypoints, 12, seed_traj_multi_thread, failed_edges_multi_thread, failed_vertices_multi_thread);

    // Expect success.
    EXPECT_TRUE(res_multithread);

    // Expect no vertices and edges to fail in multi-thread case.
    EXPECT_EQ(failed_vertices_multi_thread.size(), 0);
    EXPECT_EQ(failed_edges_multi_thread.size(), 0);
}

TEST(DescartesMultithreadUnitNearShelfDAE, Instantiation)
{
    // The environment consists of a UR-10e with a boxy end effector near a "shelf". This case is identical to DescartesMultithreadUnitNearMultibodyDAE but the "wall" is removed.
    // The geometry is defined by a mesh loaded from a .dae file.
    std::stringstream urdf_xml_string, srdf_xml_string;
    std::ifstream urdf_in(tesseract_rosutils::locateResource("package://descartes_system_tests/support/urdf/ur10e_near_obstacle_shelf.urdf"));
    urdf_xml_string << urdf_in.rdbuf();
    std::ifstream srdf_in(tesseract_rosutils::locateResource("package://descartes_system_tests/support/urdf/ur10e.srdf"));
    srdf_xml_string << srdf_in.rdbuf();

    // The waypoints run along the top edge of the shelf. We expect the waypoints to be successfully sampled and to produce a valid series of poses.
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

    // Do Descartes sampling using 12 threads
    Eigen::MatrixXd seed_traj_multi_thread;
    std::vector<std::size_t> failed_edges_multi_thread;
    std::vector<std::size_t> failed_vertices_multi_thread;
    bool res_multithread = descartes_system_tests::planDescartesSeedTraj(kin, env, world_to_base_link, tool0_to_tcp, waypoints, 12, seed_traj_multi_thread, failed_edges_multi_thread, failed_vertices_multi_thread);

    // Expect success
    EXPECT_TRUE(res_multithread);

    // Expect no vertices and edges to fail in multi-thread case
    EXPECT_EQ(failed_vertices_multi_thread.size(), 0);
    EXPECT_EQ(failed_edges_multi_thread.size(), 0);
}



int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
