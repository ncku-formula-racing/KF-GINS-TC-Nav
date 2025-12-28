cmake_minimum_required(VERSION 3.10)
project(KF_GINS_TC_Nav)

set(CMAKE_C_STANDARD 11)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Debug flags
add_compile_options(-g -O0 -Wall)

# CMSIS Configuration for PC (Generic C implementation)
add_definitions(-DARM_MATH_LOOPUNROLL)

set(CMSIS_ROOT "${CMAKE_SOURCE_DIR}/thirdparty/cmsis-dsp")

include_directories(
    src
    src/EKF
    ${CMSIS_ROOT}/Include
    ${CMSIS_ROOT}/PrivateInclude
    ${CMSIS_ROOT}/ComputeLibrary/Include
)

file(GLOB_RECURSE MY_SOURCES "src/*.c")

file(GLOB_RECURSE CMSIS_SOURCES
    "${CMSIS_ROOT}/Source/MatrixFunctions/*.c"
    "${CMSIS_ROOT}/Source/CommonTables/*.c"
    "${CMSIS_ROOT}/Source/BasicMathFunctions/*.c"
)

list(FILTER CMSIS_SOURCES EXCLUDE REGEX ".*/_.*\\.c$")
list(FILTER CMSIS_SOURCES EXCLUDE REGEX ".*/MatrixFunctions\\.c$")
list(FILTER CMSIS_SOURCES EXCLUDE REGEX ".*/CommonTables\\.c$")
list(FILTER CMSIS_SOURCES EXCLUDE REGEX ".*/BasicMathFunctions\\.c$")
list(FILTER CMSIS_SOURCES EXCLUDE REGEX ".*/MatrixFunctionsF16\\.c$")
list(FILTER CMSIS_SOURCES EXCLUDE REGEX ".*/BasicMathFunctionsF16\\.c$")
list(FILTER CMSIS_SOURCES EXCLUDE REGEX ".*F16\\.c$")
list(FILTER CMSIS_SOURCES EXCLUDE REGEX ".*F128\\.c$")
list(FILTER CMSIS_SOURCES EXCLUDE REGEX ".*_q15\\.c$")
list(FILTER CMSIS_SOURCES EXCLUDE REGEX ".*_q31\\.c$")

message(STATUS "CMSIS_ROOT is: ${CMSIS_ROOT}")
message(STATUS "CMSIS_SOURCES found: ${CMSIS_SOURCES}")

add_executable(main ${MY_SOURCES} ${CMSIS_SOURCES})
set_source_files_properties(${CMSIS_SOURCES} PROPERTIES COMPILE_FLAGS "-Wno-pragmas")
