find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
list(APPEND ALL_TARGET_LIBRARIES ${EIGEN3_LIBRARIES})