### Installation

Make a new ROS workspace.

Download the dependencies listed in `descartes_system_tests.rosinstall` into the workspace `src` directory, then build the workspace.

### Running

To run some tests involving collision objects:

```
rosrun descartes_system_tests collision_body_tests
```

To run some tests involving comparing the output of single-thread and multi-thread processing:

```
rosrun descartes_system_tests multithread_tests
```
