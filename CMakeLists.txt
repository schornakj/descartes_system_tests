cmake_minimum_required(VERSION 3.5.0)
project(descartes_system_tests)

add_compile_options(-std=c++14)

find_package(GTest QUIET)
if ( NOT ${GTest_FOUND} )
  include(ExternalProject)

  ExternalProject_Add(GTest
    GIT_REPOSITORY    https://github.com/google/googletest.git
    GIT_TAG           release-1.8.1
    SOURCE_DIR        ${CMAKE_BINARY_DIR}/../googletest-src
    BINARY_DIR        ${CMAKE_BINARY_DIR}/../googletest-build
    CMAKE_CACHE_ARGS
            -DCMAKE_INSTALL_PREFIX:STRING=${CMAKE_INSTALL_PREFIX}
            -DCMAKE_BUILD_TYPE:STRING=Release
            -DBUILD_GMOCK:BOOL=OFF
            -DBUILD_GTEST:BOOL=ON
            -DBUILD_SHARED_LIBS:BOOL=ON
  )

  file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/include)
  set(GTEST_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/include)
  set(GTEST_BOTH_LIBRARIES ${CMAKE_INSTALL_PREFIX}/lib/libgtest.so)
endif()

if(NOT TARGET gtest::GTest)
  add_library(gtest::GTest INTERFACE IMPORTED)
  set_target_properties(gtest::GTest PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${GTEST_INCLUDE_DIRS}")
  set_target_properties(gtest::GTest PROPERTIES INTERFACE_LINK_LIBRARIES "${GTEST_BOTH_LIBRARIES}")
endif()


find_package(tesseract REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_kinematics REQUIRED)
find_package(tesseract_msgs REQUIRED)
find_package(tesseract_visualization REQUIRED)
find_package(tesseract_monitoring REQUIRED)
find_package(tesseract_motion_planners REQUIRED)

find_package(descartes_light REQUIRED)
find_package(descartes_samplers REQUIRED)
find_package(descartes_ikfast REQUIRED)
find_package(descartes_tesseract REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(ur_ikfast_kinematics REQUIRED)

find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    sensor_msgs
    trajectory_msgs
    visualization_msgs
    tesseract_rosutils
    )

catkin_package(
   CATKIN_DEPENDS
       roscpp
       geometry_msgs
       sensor_msgs
       trajectory_msgs
       visualization_msgs
       tesseract_rosutils
   DEPENDS
       YAML_CPP
)

add_executable(multithread_tests src/multithread_tests.cpp)
target_include_directories(multithread_tests PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    ${catkin_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    )

target_link_libraries(multithread_tests
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    Eigen3::Eigen
    ur_ikfast_kinematics::ur10e_ikfast
    tesseract::tesseract
    tesseract::tesseract_environment_core
    tesseract::tesseract_kinematics_core
    tesseract::tesseract_common
    descartes::descartes_light
    descartes::descartes_light_core
    descartes::descartes_samplers
    descartes::descartes_tesseract
    descartes::descartes_ikfast
    gtest::GTest
    )

add_executable(collision_body_tests src/collision_body_tests.cpp)
target_include_directories(collision_body_tests PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    ${catkin_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    )

target_link_libraries(collision_body_tests
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    Eigen3::Eigen
    ur_ikfast_kinematics::ur10e_ikfast
    tesseract::tesseract
    tesseract::tesseract_environment_core
    tesseract::tesseract_kinematics_core
    tesseract::tesseract_common
    descartes::descartes_light
    descartes::descartes_light_core
    descartes::descartes_samplers
    descartes::descartes_tesseract
    descartes::descartes_ikfast
    gtest::GTest
    )

install(DIRECTORY
    support
    DESTINATION share/${PROJECT_NAME}/)
install(TARGETS
    collision_body_tests
    multithread_tests
    DESTINATION lib/${PROJECT_NAME})
