cmake_minimum_required(VERSION 3.8)
project(soul_link)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)

ADD_SUBDIRECTORY (lib)
INCLUDE_DIRECTORIES (lib)

add_library(soul_link
  src/soul_link.cpp
  # Add other reusable sources here if needed
)

target_include_directories(soul_link PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Make sure Quad-Utils can access RBDL Libraries
target_link_libraries(soul_link
  iTekCANFD
)

# Install Libraries and Executables
install(TARGETS soul_link
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Includes
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(soul_link)

add_executable(soul_link_node src/soul_link_node.cpp)

target_include_directories(soul_link_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(soul_link_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries (soul_link_node iTekCANFD soul_link)

install(TARGETS soul_link_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
