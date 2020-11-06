cmake_minimum_required(VERSION 3.5)

project(ur_dashboard_msgs)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Generate messages in the 'msg' folder
set(msg_files
  "msg/ProgramState.msg"
  "msg/RobotMode.msg"
  "msg/SafetyMode.msg"
)

# Generate services in the 'srv' folder
set(srv_files
  "srv/AddToLog.srv"
  "srv/GetLoadedProgram.srv"
  "srv/GetProgramState.srv"
  "srv/GetRobotMode.srv"
  "srv/GetSafetyMode.srv"
  "srv/IsProgramRunning.srv"
  "srv/IsProgramSaved.srv"
  "srv/Load.srv"
  "srv/Popup.srv"
  "srv/RawRequest.srv"
)

# Generate actions in the 'action' folder
set(action_files
  "action/SetMode.action"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES action_msgs builtin_interfaces std_msgs
  ADD_LINTER_TESTS
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
