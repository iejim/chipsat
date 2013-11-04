
# Build the dac-example program
set(dac-example_SRC ${dac-example_SRC} dac-example.cpp)
add_executable(dac-example ${dac-example_SRC})
set_target_properties(dac-example PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# Build the minimu-example program
set(minimu-example_SRC ${minimu-example_SRC} minimu-example.cpp)
add_executable(minimu-example ${minimu-example_SRC})
target_link_libraries(minimu-example pthread rt)
set_target_properties(minimu-example PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# Build the motor-example program
set(motor-example_SRC ${motor-example_SRC} motor-example.cpp motorprogrammer.hpp datacollector.hpp)
add_executable(motor-example ${motor-example_SRC})
target_link_libraries(motor-example rt pthread)
set_target_properties(motor-example PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# Build the gx3-example program
set(gx3-example_SRC ${gx3-example_SRC} gx3-example.cpp)
add_executable(gx3-example ${gx3-example_SRC})
target_link_libraries(gx3-example serial pthread rt)
set_target_properties(gx3-example PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# Build the threading-example program
set(threading-example_SRC ${threading-example_SRC} threading-example.cpp)
add_executable(threading-example ${threading-example_SRC})
target_link_libraries(threading-example pthread rt)
set_target_properties(threading-example PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

