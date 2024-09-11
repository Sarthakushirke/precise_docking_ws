# - Config file for the norlab_icp_mapper package
# It defines the following variables
#  norlab_icp_mapper_INCLUDE_DIRS - include directories for norlab_icp_mapper
#  norlab_icp_mapper_LIBRARIES    - libraries to link against

include(CMakeFindDependencyMacro)
find_dependency(libpointmatcher REQUIRED)
find_dependency(yaml-cpp REQUIRED)

# Compute paths
get_filename_component(NORLAB_ICP_MAPPER_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(norlab_icp_mapper_INCLUDE_DIRS "/usr/local/include;/usr/include/eigen3;/home/sarthak/precise_docking_ws/install/norlab_icp_mapper/include")
set(norlab_icp_mapper_LIBRARIES "$<TARGET_FILE:pointmatcher>;yaml-cpp;libnabo::nabo;Threads::Threads;Boost::thread;Boost::filesystem;Boost::system;Boost::program_options;Boost::date_time;Boost::chrono;/home/sarthak/precise_docking_ws/install/norlab_icp_mapper/lib/libnorlab_icp_mapper.a")
