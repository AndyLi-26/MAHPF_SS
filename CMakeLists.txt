project(mahpf)

cmake_minimum_required (VERSION 3.5)
set (CMAKE_CXX_STANDARD 14)
#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-omit-frame-pointer -fsanitize=address")

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()

include_directories("inc" "inc/CBS" "inc/lacam")
file(GLOB SOURCES "src/*.cpp" "src/CBS/*.cpp" "src/lacam/*.cpp")
add_executable(mahpf ${SOURCES})

# Find Boost
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)

# Find Eigen3 for PIBT
find_package (Eigen3 3.3 REQUIRED NO_MODULE)


include_directories( ${Boost_INCLUDE_DIRS} )
target_link_libraries(mahpf ${Boost_LIBRARIES} Eigen3::Eigen)

add_compile_options(-fsanitize=address)
add_link_options(-fsanitize=address)
