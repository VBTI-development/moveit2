/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Peter David Fagan
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Peter David Fagan */

#include "moveit_cpp.hpp"
#include <pybind11/pytypes.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.hpp>
#include <moveit/utils/logger.hpp>
#include <string>
// <typeinfo> and <dlfcn.h>: only for the keepControllerManagerLibraryLoaded() workaround below.
#include <typeinfo>

#ifndef _WIN32
#include <dlfcn.h>
#endif

namespace moveit_py
{
namespace bind_moveit_cpp
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.py.cpp_initializer");
}

// WORKAROUND, remove once the fix is released: https://github.com/moveit/moveit2/pull/3970
//
// Keeps the controller manager plugin's library loaded until the process exits.
//
// What it works around: every process that created a MoveItPy died with SIGSEGV when the MoveItCpp was
// destroyed (in practice at interpreter exit, right after the "Deleting MoveItCpp" log line), with
//   rclcpp::CallbackGroup::~CallbackGroup() <- rclcpp::Node::~Node()
//   <- trajectory_execution_manager::TrajectoryExecutionManager::~TrajectoryExecutionManager()
//   <- moveit_cpp::MoveItCpp::~MoveItCpp()
//
// Why: TrajectoryExecutionManager destroys its plugin loader, which unloads the plugin's library
// (libmoveit_simple_controller_manager.so), before the node it handed to the plugin. The node's callback
// group still tracks the action clients the plugin created, through weak pointers whose control blocks
// are code of the unloaded library, so destroying the node calls into unmapped memory.
//
// The real fix is in moveit_ros_planning (the TrajectoryExecutionManager destructor), PR 3970 above, with
// backports requested for kilted, jazzy and humble. This workaround lives here because moveit_py is the
// only MoveIt package built from source next to the released moveit_ros_planning binaries.
//
// When to remove: as soon as the moveit_ros_planning in use contains the fix, that is a release made
// after PR 3970 (or its backport to the distro in use) was merged; 2.12.4 and older do NOT have it.
// Check with `dpkg -s ros-$ROS_DISTRO-moveit-ros-planning | grep Version` against the release's changelog.
//
// What to remove: this function, its call in the MoveItPy constructor below, the <typeinfo> and
// <dlfcn.h> includes above, and ${CMAKE_DL_LIBS} from the `planning` target in moveit_py/CMakeLists.txt.
// Then verify: a script that creates a MoveItPy and exits must end with exit code 0, not -11.
void keepControllerManagerLibraryLoaded(const moveit_cpp::MoveItCpp& moveit_cpp)
{
#ifndef _WIN32
  const auto trajectory_execution_manager = moveit_cpp.getTrajectoryExecutionManager();
  const auto controller_manager =
      trajectory_execution_manager ? trajectory_execution_manager->getControllerManager() : nullptr;
  if (!controller_manager)
  {
    return;
  }

  // The plugin class is defined in the plugin's library, and so is its type information.
  const auto& plugin = *controller_manager;
  Dl_info library;
  if (dladdr(&typeid(plugin), &library) == 0 || library.dli_fname == nullptr ||
      dlopen(library.dli_fname, RTLD_NOW | RTLD_NODELETE) == nullptr)
  {
    RCLCPP_WARN(getLogger(), "Could not keep the controller manager plugin's library loaded, "
                             "destroying this MoveItPy instance may crash");
  }
#else
  (void)moveit_cpp;
#endif
}

std::shared_ptr<moveit_cpp::PlanningComponent>
getPlanningComponent(std::shared_ptr<moveit_cpp::MoveItCpp>& moveit_cpp_ptr, const std::string& planning_component)
{
  return std::make_shared<moveit_cpp::PlanningComponent>(planning_component, moveit_cpp_ptr);
}

void initMoveitPy(py::module& m)
{
  auto utils = py::module::import("moveit.utils");

  py::class_<moveit_cpp::MoveItCpp, std::shared_ptr<moveit_cpp::MoveItCpp>>(m, "MoveItPy", R"(
  The MoveItPy class is the main interface to the MoveIt Python API. It is a wrapper around the MoveIt C++ API.
									     )")

      .def(py::init([](const std::string& node_name, const std::string& name_space,
                       const std::vector<std::string>& launch_params_filepaths, const py::object& config_dict,
                       bool provide_planning_service,
                       const std::optional<std::map<std::string, std::string>>& remappings) {
             // This section is used to load the appropriate node parameters before spinning a moveit_cpp instance
             // Priority is given to parameters supplied directly via a config_dict, followed by launch parameters
             // and finally no supplied parameters.
             std::vector<std::string> launch_arguments;
             if (!config_dict.is(py::none()))
             {
               auto utils = py::module::import("moveit.utils");
               // TODO (peterdavidfagan): replace python method with C++ method
               std::string params_filepath =
                   utils.attr("create_params_file_from_dict")(config_dict, node_name).cast<std::string>();
               launch_arguments = { "--ros-args", "--params-file", params_filepath };
             }
             else if (!launch_params_filepaths.empty())
             {
               launch_arguments = { "--ros-args" };
               for (const auto& launch_params_filepath : launch_params_filepaths)
               {
                 launch_arguments.push_back("--params-file");
                 launch_arguments.push_back(launch_params_filepath);
               }
             }

             if (remappings.has_value())
             {
               for (const auto& [key, value] : *remappings)
               {
                 std::string argument = key;
                 argument.append(":=").append(value);
                 launch_arguments.push_back("--remap");
                 launch_arguments.push_back(std::move(argument));
               }
             }

             // Initialize ROS, pass launch arguments with rclcpp::init()
             if (!rclcpp::ok())
             {
               std::vector<const char*> chars;
               chars.reserve(launch_arguments.size());
               for (const auto& arg : launch_arguments)
               {
                 chars.push_back(arg.c_str());
               }

               rclcpp::init(launch_arguments.size(), chars.data());
               RCLCPP_INFO(getLogger(), "Initialize rclcpp");
             }

             // Build NodeOptions
             RCLCPP_INFO(getLogger(), "Initialize node parameters");
             rclcpp::NodeOptions node_options;
             node_options.allow_undeclared_parameters(true)
                 .automatically_declare_parameters_from_overrides(true)
                 .arguments(launch_arguments);

             RCLCPP_INFO(getLogger(), "Initialize node and executor");
             rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(node_name, name_space, node_options);
             std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor =
                 std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

             RCLCPP_INFO(getLogger(), "Spin separate thread");
             auto spin_node = [node, executor]() {
               executor->add_node(node);
               executor->spin();
             };
             std::thread execution_thread(spin_node);
             execution_thread.detach();

             auto custom_deleter = [executor](moveit_cpp::MoveItCpp* moveit_cpp) {
               executor->cancel();
               rclcpp::shutdown();
               delete moveit_cpp;
             };

             std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr(new moveit_cpp::MoveItCpp(node), custom_deleter);
             // WORKAROUND for https://github.com/moveit/moveit2/pull/3970, see keepControllerManagerLibraryLoaded().
             keepControllerManagerLibraryLoaded(*moveit_cpp_ptr);

             if (provide_planning_service)
             {
               moveit_cpp_ptr->getPlanningSceneMonitorNonConst()->providePlanningSceneService();
             };

             return moveit_cpp_ptr;
           }),
           py::arg("node_name") = "moveit_py", py::arg("name_space") = "",
           py::arg("launch_params_filepaths") =
               utils.attr("get_launch_params_filepaths")().cast<std::vector<std::string>>(),
           py::arg("config_dict") = py::none(), py::arg("provide_planning_service") = true,
           py::arg("remappings") = py::none(), py::return_value_policy::take_ownership,
           R"(
           Initialize moveit_cpp node and the planning scene service.
           )")
      .def("execute",
           py::overload_cast<const robot_trajectory::RobotTrajectoryPtr&, const std::vector<std::string>&>(
               &moveit_cpp::MoveItCpp::execute),
           py::arg("robot_trajectory"), py::arg("controllers"), py::call_guard<py::gil_scoped_release>(),
           R"(
	   Execute a trajectory (planning group is inferred from robot trajectory object).
	   )")
      .def("get_planning_component", &moveit_py::bind_moveit_cpp::getPlanningComponent,
           py::arg("planning_component_name"), py::return_value_policy::take_ownership,
           R"(
           Creates a planning component instance.
           Args:
               planning_component_name (str): The name of the planning component.
           Returns:
               :py:class:`moveit_py.planning.PlanningComponent`: A planning component instance corresponding to the provided plan component name.
          )")

      .def(
          "shutdown", [](std::shared_ptr<moveit_cpp::MoveItCpp>& /*moveit_cpp*/) { rclcpp::shutdown(); },
          R"(
          Shutdown the moveit_cpp node.
          )")

      .def("get_planning_scene_monitor", &moveit_cpp::MoveItCpp::getPlanningSceneMonitorNonConst,
           py::return_value_policy::reference,
           R"(
           Returns the planning scene monitor.
           )")

      .def("get_trajectory_execution_manager", &moveit_cpp::MoveItCpp::getTrajectoryExecutionManagerNonConst,
           py::return_value_policy::reference,
           R"(
           Returns the trajectory execution manager.
           )")

      .def("get_robot_model", &moveit_cpp::MoveItCpp::getRobotModel, py::return_value_policy::reference,
           R"(
           Returns robot model.
        )");
}
}  // namespace bind_moveit_cpp
}  // namespace moveit_py
