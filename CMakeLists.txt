cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(pwr-ctrl CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
add_definitions(-DBOOST_SYSTEM_NO_DEPRECATED)
add_definitions(-DBOOST_ALL_NO_LIB)
add_definitions(-DBOOST_NO_RTTI)
add_definitions(-DBOOST_NO_TYPEID)
add_definitions(-DBOOST_ASIO_DISABLE_THREADS)

set(SRC_FILES pwr.cpp)

add_executable(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} systemd)
target_link_libraries(${PROJECT_NAME} sdbusplus)
target_link_libraries(${PROJECT_NAME} gpiodcxx)

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES pwr-ctrl.service DESTINATION /lib/systemd/system)
