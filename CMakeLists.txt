cmake_minimum_required( VERSION 3.15 )
project( simple_ris_bpt )

set( CMAKE_CXX_STANDARD 17 )

file( GLOB HEADER_FILES src/inc/*.hpp src/inc/*/*.hpp src/inc/*/*/*.hpp src/inc/*/*/*/*.hpp )
file( GLOB SOURCE_FILES src/main.cpp )
add_executable( ${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES} )
target_link_libraries( ${PROJECT_NAME} )