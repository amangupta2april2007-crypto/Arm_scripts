################################################################################
# CMakeLists.txt for ship_repair_tasks
# ─────────────────────────────────────
# This is a pure-Python ROS 2 package built with ament_cmake.
# We use ament_cmake (not ament_python) because the rest of the workspace
# already uses ament_cmake, keeping the build system consistent.
#
# What this file does:
#  1. find_package(ament_cmake REQUIRED)   – load the ament build system macros
#  2. install(PROGRAMS …)                 – copy Python scripts into the
#                                           install/lib/<pkg>/ directory so
#                                           ros2 run can discover them
#  3. install(DIRECTORY launch config …)  – copy launch and config files into
#                                           share/<pkg>/ for ros2 launch
#  4. ament_package()                     – finalise the package
################################################################################

cmake_minimum_required(VERSION 3.8)
project(ship_repair_tasks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ── Install the Python executables ───────────────────────────────────────────
# Each script becomes available as:
#   ros2 run ship_repair_tasks <script_name>
install(PROGRAMS
  scripts/arm_control_helpers.py
  scripts/task1_deliver_tools.py
  scripts/task2_inspect_damage.py
  scripts/task3_apply_repair.py
  scripts/task4_document_progress.py
  scripts/task5_attach_pipe.py
  scripts/task6_operate_valves.py
  scripts/task7_return_tools.py
  scripts/full_mission.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install launch files and configuration ──────────────────────────────────
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
