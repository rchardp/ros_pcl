cmake_minimum_required(VERSION 2.8.3)
project(pclpy)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roscpp_serialization
  std_msgs
  sensor_msgs
)

## Both Boost.python and Python libs are required.
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs 2.7 REQUIRED)



## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
			INCLUDE_DIRS include
			LIBRARIES
			_pcl_common_wrapper_cpp
			_pcl_pcdio_wrapper_cpp
			_pcl_visualizer_wrapper_cpp
			_pcl_voxel_grid_wrapper_cpp
			_pcl_passthrough_wrapper_cpp
			_pcl_outliers_wrapper_cpp
			_pcl_icp_wrapper_cpp
			CATKIN_DEPENDS roscpp
			#  DEPENDS system_lib
)

###########
## Build ##
###########

# include Boost and Python.
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        )

link_directories(
        ${PCL_LIBRARY_DIRS}
        ${Boost_LIBRARIES_DIRS}
        )
add_definitions(${PCL_DEFINITIONS})

## Declare a cpp library
add_library(_pcl_common_wrapper_cpp src/pcl_common_wrapper.cpp)
add_library(_pcl_pcdio_wrapper_cpp src/pcdio/pcdio_wrapper.cpp)
add_library(_pcl_icp_wrapper_cpp src/registration/icp_wrapper.cpp)
add_library(_pcl_visualizer_wrapper_cpp src/visualization/pcl_visualizer_wrapper.cpp)
add_library(_pcl_voxel_grid_wrapper_cpp src/filters/voxel_grid_wrapper.cpp)
add_library(_pcl_passthrough_wrapper_cpp src/filters/passthrough_wrapper.cpp)
add_library(_pcl_outliers_wrapper_cpp src/filters/outliers_wrapper.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(_pcl_common_wrapper_cpp ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} )
target_link_libraries(_pcl_pcdio_wrapper_cpp ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} )
target_link_libraries(_pcl_icp_wrapper_cpp ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} )
target_link_libraries(_pcl_visualizer_wrapper_cpp ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} )
target_link_libraries(_pcl_voxel_grid_wrapper_cpp ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} )
target_link_libraries(_pcl_passthrough_wrapper_cpp ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} )
target_link_libraries(_pcl_outliers_wrapper_cpp ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} )

# Don't prepend wrapper library name with lib and add to Python libs.
set_target_properties(
			_pcl_common_wrapper_cpp
			_pcl_pcdio_wrapper_cpp
			_pcl_icp_wrapper_cpp
			_pcl_visualizer_wrapper_cpp
			_pcl_voxel_grid_wrapper_cpp
			_pcl_passthrough_wrapper_cpp
			_pcl_outliers_wrapper_cpp
			PROPERTIES
			PREFIX ""
			LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
			)
