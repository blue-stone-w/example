find_package(GTSAM REQUIRED QUIET)
# list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(${GTSAM_INCLUDE_DIR})
