cmake_minimum_required(VERSION 3.5)

project(softNeckStabilization)

if(NOT YARP_FOUND)
    find_package(YARP 3.5 REQUIRED COMPONENTS os dev)
endif()


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(${PROJECT_SOURCE_DIR}/lib/CiA402Device/)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/fcontrol/)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/sensor-integration/)


INCLUDE_DIRECTORIES(${SUBDIR_INCLUDE_DIRECTORIES})


aux_source_directory(main SRC_LIST)
aux_source_directory(utils SRC_LIST)

#add all executables in foreach
foreach( sourcefile ${SRC_LIST} )
    #Used a simple string replace, to cut off .cpp.
    string( REGEX REPLACE "[A-z]+[/]" "" name ${sourcefile} )
    string( REPLACE ".cpp" "" name ${name} )
    #add executable
    add_executable( ${name} ${sourcefile} )
    message ("Including executable: " ${name})
    target_link_libraries( ${name} ${SUBDIR_LINK_NAMES}
    			             YARP::YARP_os
                                     YARP::YARP_init
                                     YARP::YARP_dev)

endforeach( sourcefile ${SRC_LIST} )

