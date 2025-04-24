cmake_minimum_required (VERSION 3.13)

project(CAN_XLAnalyzer)

add_definitions( -DLOGIC2 )

# enable generation of compile_commands.json, helpful for IDEs to locate include files.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# custom CMake Modules are located in the cmake directory.
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include(ExternalAnalyzerSDK)

set(SOURCES 
src/CAN_XLAnalyzer.cpp
src/CAN_XLAnalyzer.h
src/CAN_XLAnalyzerResults.cpp
src/CAN_XLAnalyzerResults.h
src/CAN_XLAnalyzerSettings.cpp
src/CAN_XLAnalyzerSettings.h
src/CAN_XLSimulationDataGenerator.cpp
src/CAN_XLSimulationDataGenerator.h
)

add_analyzer_plugin(${PROJECT_NAME} SOURCES ${SOURCES})
