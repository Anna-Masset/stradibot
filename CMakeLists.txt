set(STRADIBOT_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DSTRADIBOT_FOLDER="${STRADIBOT_FOLDER}")

# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS225A_BINARY_DIR}/stradibot)
ADD_EXECUTABLE (simviz_stradibot simviz.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (controller_stradibot controller.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (simviz_stradibot ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_stradibot ${CS225A_COMMON_LIBRARIES})
