cmake_minimum_required(VERSION 3.16)
project(pct_planner)

# Keep the top-level and nested planning_core install configurations equal.
# Otherwise ament's symlink installer looks for an unqualified target list
# while the nested project generated a Release-only list.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(PCT_THIRDPARTY_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/install"
  CACHE PATH "PCT local third-party installation root")
list(PREPEND CMAKE_PREFIX_PATH
  "${PCT_THIRDPARTY_ROOT}/gtsam"
  "${PCT_THIRDPARTY_ROOT}/osqp")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

execute_process(
  COMMAND "${Python3_EXECUTABLE}" -c "import sysconfig; print(sysconfig.get_path('platlib', vars={'platbase': '.', 'base': '.'}))"
  OUTPUT_VARIABLE PYTHON_INSTALL_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

add_subdirectory(lib)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

ament_package()
