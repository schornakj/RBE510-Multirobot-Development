cmake_minimum_required(VERSION 2.8)

project(RBE_MULTI_ROBOT_SYSTEMS_WPI)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
	message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

include_directories("${PROJECT_SOURCE_DIR}/src")

find_package (Threads)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY})

# add_executable(rbe_server ${PROJECT_SOURCE_DIR}/src/rbe_server.cpp)
# add_executable(rbe_test_client ${PROJECT_SOURCE_DIR}/src/rbe_test_client.cpp )
# add_executable(g2p ${PROJECT_SOURCE_DIR}/src/g2p.cpp)
# add_executable(rbe_test_client_transforms ${PROJECT_SOURCE_DIR}/src/rbe_test_client_transforms.cpp)
# add_executable(rbe_test_client_transforms ${PROJECT_SOURCE_DIR}/src/rbe510.cpp ${PROJECT_SOURCE_DIR}/src/rbe_test_client_transforms.cpp)
# add_executable(Assignment1 ${PROJECT_SOURCE_DIR}/src/Assignment1.cpp ${PROJECT_SOURCE_DIR}/src/rbe510.cpp)
# add_executable(Assignment1 ${PROJECT_SOURCE_DIR}/src/Assignment1.cpp ${PROJECT_SOURCE_DIR}/src/planner.cpp)

add_executable(Assignment2 ${PROJECT_SOURCE_DIR}/src/Assignment2.cpp)
# add_executable(Assignment2 ${PROJECT_SOURCE_DIR}/src/Assignment2.cpp ${PROJECT_SOURCE_DIR}/src/rbe510.cpp)

# target_link_libraries (rbe_server ${OpenCV_LIBS})
# target_link_libraries (rbe_server ${CMAKE_THREAD_LIBS_INIT})
# target_link_libraries (rbe_test_client_transforms  ${CMAKE_THREAD_LIBS_INIT})
# target_link_libraries (rbe_test_client_transforms  ${OpenCV_LIBS})
# target_link_libraries (Assignment1  ${CMAKE_THREAD_LIBS_INIT})
# target_link_libraries (Assignment1 ${OpenCV_LIBS})

target_link_libraries (Assignment2  ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries (Assignment2 ${OpenCV_LIBS})