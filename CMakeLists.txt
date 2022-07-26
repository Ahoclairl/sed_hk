# 最低需求版本
cmake_minimum_required(VERSION 2.8.3)
# 项目名
project(sed_hk) # 生成两个变量 PROJECT_BINARY_DIR / PROJECT_SOURCE_DIR

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/common/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common/include/GLFW
    ${CMAKE_CURRENT_SOURCE_DIR}/common/include/glad
    ${CMAKE_CURRENT_SOURCE_DIR}/common/include/KHR
)

# 查找需要的 package
find_package(OpenCV 3.4.4 EXACT REQUIRED)

MESSAGE(STATUS "OpenCV_LIB_DIR: ${OpenCV_LIB_DIR}")
MESSAGE(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

# 添加需要链接的库文件目录，用来加载外加的=动态链接库或静态链接库的搜索路径，相当于gcc的-L参数
link_directories(
    /usr/lib
    ${PROJECT_SOURCE_DIR}/commmon/lib/linux64
    ${PROJECT_SOURCE_DIR}/lib
)

# 搜索一个目录下所有的源文件
aux_source_directory(./src SOURCE) # 将 . (当前)目录下所有的源文件统一为 SOURCE
aux_source_directory(./common/include/glad GLAD)
# 设置编译类型
# add_library(demo ${SOURCE})
add_executable(sed ${SOURCE} ${GLAD}) # 生成可执行文件，需要有 main 函数

# 链接库
target_link_libraries(sed -lyaml-cpp -ldl -lMv3dRgbd -lglfw -lGL -lGLU ${OpenCV_LIBS})