# 最低需求版本
cmake_minimum_required(VERSION 2.8.3)
# 项目名
project(sed_hk) # 生成两个变量 PROJECT_BINARY_DIR / PROJECT_SOURCE_DIR

set(cv_bridge_DIR ${CMAKE_SOURCE_DIR}/src/cv_bridge)

# 查找需要的 package
find_package(OpenCV  3.4.4 EXACT REQUIRED)
find_package(Boost)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    sensor_msgs
    cv_bridge
    image_transport
    message_generation
)

add_service_files(
    FILES
    image.srv
)

if(NOT WIN32)
ADD_DEFINITIONS("-std=c++0x -O3")
endif(NOT WIN32)

# 服务器消息传输需要使用 sensor_msgs
generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs

)

catkin_package(CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    message_runtime
)

link_directories(${OpenCV_LIB_DIR})

MESSAGE(STATUS "OpenCV_LIB_DIR: ${OpenCV_LIB_DIR}")
MESSAGE(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/common
    ${PROJECT_SOURCE_DIR}/common/include
    ${PROJECT_SOURCE_DIR}/common/include/GLFW
    ${PROJECT_SOURCE_DIR}/common/include/glad
    ${PROJECT_SOURCE_DIR}/common/include/KHR
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_BINARY_DIR}/
    # 用来关联 catkin 工作空间下的头文件，包括 ros.h 之类
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS} 
)

# 添加需要链接的库文件目录，用来加载外加的=动态链接库或静态链接库的搜索路径，相当于gcc的-L参数
link_directories(
    /usr/lib
    ${PROJECT_SOURCE_DIR}/commmon/lib/linux64
    ${PROJECT_SOURCE_DIR}/lib
)

# 搜索一个目录下所有的源文件
aux_source_directory(common/include/glad glad)

add_executable(sed_hk_client src/client.cpp src/HKcamInterface.cpp ${glad}) # 生成可执行文件，需要有 main 函数
add_executable(sed_hk_server src/server.cpp src/sed_hk.cpp ${glad}) # 生成可执行文件，需要有 main 函数

add_dependencies(sed_hk_client ${PROJECT_NAME}_gencpp)
add_dependencies(sed_hk_server ${PROJECT_NAME}_gencpp)

target_link_libraries(sed_hk_client -lyaml-cpp -ldl -lMv3dRgbd -lglfw -lGL -lGLU ${OpenCV_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})
target_link_libraries(sed_hk_server -lyaml-cpp -ldl -lMv3dRgbd -lglfw -lGL -lGLU ${OpenCV_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})
