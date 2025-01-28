# Parameter Management Library of TUM Autonomous Motorsport

Managing the huge amount of parameters in autonomous driving is always challenging.
ROS 2 already provides an API for managing (set/get/load from file/dump) parameters both during launch and runtime.
However, using this API deeply in your code makes your code heavily dependent on ROS 2.

Especially for research algorithms, this might not be desireable. Keeping your algorithms independent of ROS 2 allows for: 
 - Simplified (CI) testing
 - Running your algorithms in another non ROS 2 based environment. This is especially helpful when providing your code to other researchers.
 - Ensuring future proofness of your algorithm implementation. (New ROS Versions/New Middleware/etc ...)

This is why we designed this library! 
Within this packages we provide a generic API that allows for efficiently managing parameters without external dependencies.
For a full set of features, see below:


## Features
- **Python | Cpp**: Even though the library is written in CPP we provide python bindings. With this, managing parameters in the same way in a mixed language ROS 2 Stack becomes possible.
- **Varios data types for paramters**: We support all data types that are supported by ROS 2. Since it is also possible to express parameter values as strings or in binary format, there is no limit on what parameters can be loaded with this pkg.
- Allows modules to stay **ROS 2 independent**
- Allows for **Runtime Updates** of Parameters Values.
- **Simple Integration into an ROS 2 Environment**: We provide two helper functions to automatically synchronize parameters between this package and the ROS 2 API. This allows interacting with parameters during runtime using the `ros2 param` CLI.
  [This example](./param_management_ros2_integration_cpp/examples/default.cpp) shows how to connect your parameter to your ROS 2 Node.
- **Different Parameter Management Strategies**: We provide different strategies for managing the parameters during runtime:
    - Cyclically get the current value: [See this example](./param_management_cpp/examples/value_manager.cpp) 
    - Monitor for changes and get the current value after an update: [See this example](./param_management_cpp/examples/value_manager_struct.cpp) 
    - Use the parameter manager to automatically update a certain memory location with the current value: [See this example](./param_management_cpp/examples/reference_manager.cpp) 

## Limitations
 - Currently, the ReferenceLogger does not adhere to the `UserInterface` base class. This is an implementation detail and should not matter a lot to the end user. We might change this in the future.

## Minimum Working Example

For a minimum example of using the logger in a ROS 2 environment, consider [this example](./param_management_ros2_integration_cpp/examples/default.cpp)


## Package Overview

| Package Name | Usage Examples | Description  |
| ------------ | ----------- | --------------- |
| [`param_management_cpp`](./param_management_cpp/) | [`Examples`](./param_management_cpp/examples) | Contains the Core Implementation of the Parameter Management API. | 
| [`param_management_py`](./param_management_py/) | [`Examples`](./param_management_py/examples) | Python Binding | 
| [`param_management_ros2_integration_cpp`](./param_management_ros2_integration_cpp/) | [`Examples`](./param_management_ros2_integration_cpp/examples) | Helpers for Easy Integration into a ROS 2 Node. | 
| [`param_management_ros2_integration_py`](./param_management_ros2_integration_py/) | [`Examples`](./param_management_ros2_integration_py/examples) | Python Binding | 


## Compilation

Everything should just compile using colcon. The packages `param_management_cpp` and `param_management_py` should compile without ROS 2 installed. For the other packages, having a installation of ROS 2 is required.

**Disclaimer**: This package was developed using Ubuntu 22.04 and ROS 2 Humble. 
It is expected to work also for future Ubuntu and ROS 2 releases. 
However, if you experience problems with compilation, please open an Issue.  

## Core Developers
 - [Simon Sagmeister](https://github.com/simonsag96)

## Acknowledgments

Special thanks to my colleagues for the regular technical feedback and talks during the development phase of this package.:
- [Simon Hoffmann](https://github.com/simonh92)


We gratefully acknowledge financial support by:
 - Deutsche Forschungsgemeinschaft (DFG, German Research Foundation) | Project Number - 469341384
