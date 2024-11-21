# set(SOURCES Demo_test.c)
# add_executable(demo_test ${SOURCES})

# target_include_directories(demo_test PUBLIC
#     ${CMAKE_SOURCE_DIR}/soem
#     ${CMAKE_SOURCE_DIR}/osal
#     ${CMAKE_SOURCE_DIR}/osal/linux
#     ${CMAKE_SOURCE_DIR}/oshw/linux
# )

# target_link_libraries(demo_test soem)
# install(TARGETS demo_test DESTINATION bin)


cmake_minimum_required(VERSION 3.9)
project(ethercat_demo CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add source files
set(SOURCES 
    main.cpp
    Ethercat_control.cpp
)

# Create executable
add_executable(demo_test ${SOURCES})

# Include directories
target_include_directories(demo_test PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/soem
    ${CMAKE_SOURCE_DIR}/osal
    ${CMAKE_SOURCE_DIR}/osal/linux
    ${CMAKE_SOURCE_DIR}/oshw/linux
)


# Link libraries
target_link_libraries(demo_test soem pthread rt)

# Install target
install(TARGETS demo_test DESTINATION bin)
