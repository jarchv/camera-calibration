cmake_minimum_required(VERSION 3.10)
project(camcalib C CXX)
project(base C CXX)
find_package(OpenCV REQUIRED )

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

message(STATUS "\nOpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}\n")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")


add_library(base
    src/utils.cpp
)

target_compile_features(base PUBLIC cxx_std_11)

add_executable(out src/main.cpp)

set_target_properties(out PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(out PRIVATE base ${OpenCV_LIBS})
install(TARGETS out DESTINATION /usr/lib)




