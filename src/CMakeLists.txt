
# Source files for dac-example
set(dac-example_SRC
        ${CMAKE_CURRENT_SOURCE_DIR}/max127.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/I2CBus.cpp
        PARENT_SCOPE
    )

# Source files for minimu-example
set(minimu-example_SRC
        ${CMAKE_CURRENT_SOURCE_DIR}/minimu.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/periodicrtthread.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/RtThread.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/I2CBus.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/L3G.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/LSM303.cpp
        PARENT_SCOPE
    )

# Source files for motor-example
set(motor-example_SRC
        ${CMAKE_CURRENT_SOURCE_DIR}/motorcontrol.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/max127.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/cPWM.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Beagle_GPIO.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/motor.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/I2CBus.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/periodicrtthread.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/RtThread.cpp
        PARENT_SCOPE
    )

# Source files for gx3-example
set(gx3-example_SRC
        ${CMAKE_CURRENT_SOURCE_DIR}/gx3communicator.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/RtThread.cpp
        PARENT_SCOPE
    )

# Source files for threading-example
set(threading-example_SRC
        ${CMAKE_CURRENT_SOURCE_DIR}/RtThread.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/periodicrtthread.cpp
        PARENT_SCOPE
    )

# Use all .cpp-files for chipsat
FILE(GLOB chipsat_SRC *.cpp)
# Build chipsat
add_executable(chipsat ${chipsat_SRC} ${chipsat_HEADER})
target_link_libraries(chipsat pthread serial rt)
set_target_properties(chipsat PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

