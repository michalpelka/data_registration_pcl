cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(data_registration_pcl)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
include_directories(${PCL_INCLUDE_DIRS} include/dataFramework)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#########################################################
# Open CV
#########################################################

find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

SET(DATA_FRAMEWORK_SOURCES src/dataFramework/data_model.cpp)

add_executable (model_viewer src/model_viewer.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (model_viewer ${PCL_LIBRARIES})

add_executable (trajectory_viewer src/trajectory_viewer.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (trajectory_viewer ${PCL_LIBRARIES})

add_executable (filtering src/filtering.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (filtering ${PCL_LIBRARIES})

add_executable (icp src/icp.cpp src/dataFramework/viewerLog.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (icp ${PCL_LIBRARIES})

add_executable (ndt src/ndt.cpp src/dataFramework/viewerLog.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (ndt ${PCL_LIBRARIES})

add_executable (subsampling src/subsampling.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (subsampling ${PCL_LIBRARIES})

add_executable (lum src/lum.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (lum ${PCL_LIBRARIES})

add_executable (elch src/elch.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (elch ${PCL_LIBRARIES})

add_executable (elch_viewer src/elch_viewer.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (elch_viewer ${PCL_LIBRARIES})

add_executable (lum_viewer src/lum_viewer.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (lum_viewer ${PCL_LIBRARIES})

add_executable (icp_overlap_viewer src/icp_overlap_viewer.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (icp_overlap_viewer ${PCL_LIBRARIES})

#add_executable (ndt_grid_viewer src/ndt_grid_viewer.cpp ${DATA_FRAMEWORK_SOURCES})
#target_link_libraries (ndt_grid_viewer ${PCL_LIBRARIES})

add_executable (residuum_viewer src/residuum_viewer.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (residuum_viewer ${PCL_LIBRARIES})

add_executable (histogram_error_viewer src/histogram_error_viewer.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (histogram_error_viewer ${PCL_LIBRARIES})

add_executable (ortho_viewer src/ortho_viewer.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (ortho_viewer ${PCL_LIBRARIES})

add_executable (nn_overlap_plot src/nn_overlap_plot.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (nn_overlap_plot ${PCL_LIBRARIES})

add_executable (result_plot src/result_plot.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (result_plot ${PCL_LIBRARIES})

add_executable (get_scan_name src/get_scan_name.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (get_scan_name ${PCL_LIBRARIES})

add_executable (compute_normals src/compute_normals.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (compute_normals ${PCL_LIBRARIES})

add_executable (copy_metadata src/copy_metadata.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (copy_metadata ${PCL_LIBRARIES})

add_executable (icp_demo src/icp_demo.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (icp_demo ${PCL_LIBRARIES})

add_executable (add_noise src/add_noise.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (add_noise ${PCL_LIBRARIES})

add_executable (get_errors src/get_errors.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (get_errors ${PCL_LIBRARIES})

add_executable (reaplace_data_path src/reaplace_data_path.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (reaplace_data_path ${PCL_LIBRARIES})

add_executable (find_georeference src/find_georeference.cpp ${DATA_FRAMEWORK_SOURCES})
target_link_libraries (find_georeference ${PCL_LIBRARIES} ${OpenCV_LIBS})

