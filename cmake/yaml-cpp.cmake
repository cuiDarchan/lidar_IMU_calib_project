find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})
list(APPEND ALL_TARGET_LIBRARIES ${YAML_CPP_LIBRARIES})