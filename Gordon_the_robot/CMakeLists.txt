# - SAI2-URDF
find_package(SAI2-PRIMITIVES REQUIRED)
include_directories(${SAI2-PRIMITIVES_INCLUDE_DIRS})
add_definitions(${SAI2-PRIMITIVES_DEFINITIONS})

# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS225A_BINARY_DIR}/Gordon-robot)
ADD_EXECUTABLE (controller_Gordon_robot src/controller.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (simviz_Gordon_robot src/simviz.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (controller_Gordon_robot ${CS225A_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_Gordon_robot ${CS225A_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})

# collision demo
ADD_EXECUTABLE (controller_demo src/controller_demo.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (simviz_demo src/simviz_demo.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (controller_Gordon_robot ${CS225A_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_Gordon_robot ${CS225A_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_demo ${CS225A_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_demo ${CS225A_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY urdf_files/world_panda_gripper.urdf urdf_files/mmp_panda.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY urdf_files/spatula.urdf urdf_files/burger.urdf urdf_files/tomato.urdf urdf_files/cheese.urdf urdf_files/lettuce.urdf urdf_files/top_bun.urdf urdf_files/bottom_bun.urdf DESTINATION ${APP_RESOURCE_DIR})

# for collision demo
FILE(COPY urdf_files/world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY urdf_files/panda_collision.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY urdf_files/cup.urdf DESTINATION ${APP_RESOURCE_DIR})
