cmake_minimum_required(VERSION 2.8)

if(${CMAKE_VERSION} VERSION_GREATER 2.8.3)
    cmake_policy(SET CMP0017 NEW)
endif()
if(NOT ${CMAKE_VERSION} VERSION_LESS 3.0.0)
    cmake_policy(SET CMP0042 OLD)
endif()
project(bspapp CXX C)

# set the default build type
if (NOT CMAKE_BUILD_TYPE)
    # By default, use Debug mode
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Type of build" FORCE)
endif()

message(STATUS "Building ${CMAKE_BUILD_TYPE}")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake-2.8/Modules/" "${PROJECT_SOURCE_DIR}/CMakeModules/")

if(APPLE)
	set(QT_QMAKE_EXECUTABLE /opt/local/libexec/qt4/bin/qmake)
endif()

if(APPLE AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    include(GetPrerequisites)
    get_prerequisites("${Boost_SYSTEM_LIBRARY}" _libs 0 0 "/" "")
    set(CXXSTDLIB "")
    foreach(_lib ${_libs})
        if(_lib MATCHES "libc\\+\\+")
            set(CXXSTDLIB "libc++")
        elseif(_lib MATCHES "libstdc\\+\\+")
            set(CXXSTDLIB "libstdc++")
        endif()
    endforeach()
    if(CXXSTDLIB)
        add_definitions(-stdlib=${CXXSTDLIB})
    endif()
endif()

# reqd packages
find_package(Boost QUIET 1.54)
find_package(Boost COMPONENTS date_time thread serialization filesystem system program_options unit_test_framework chrono REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Qt4 4.4.3 REQUIRED QtCore QtGui QtXml REQUIRED)
find_package(OMPL REQUIRED)
find_package(Armadillo REQUIRED)
find_package(TINYXML REQUIRED)

set(QT_USE_QTOPENGL true)

# here we specify the additional include directories for the project. These files come in additional include directories option of VC++
# project.
# either the variable values from package like ${PCL_INCLUDE_DIRS} or absolute paths as shown in second and third line may be used.
include_directories(
	${PROJECT_SOURCE_DIR}/include/
	${Boost_INCLUDE_DIRS}
	${OPENGL_INCLUDE_DIRS}
	${Qt4_INCLUDE_DIRS}
	${OMPL_INCLUDE_DIRS}
	${ARMADILLO_INCLUDE_DIRS}
	${QT_INCLUDES}
	${TINYXML_INCLUDE_DIR}
	${GLUT_INCLUDE_DIR}
)

include(${QT_USE_FILE})

# here we specify the additional library directories for the linker in the project. These files come in additional library directories
# option of VC++ project.
# either the variable values from package like ${PCL_LIBRARY_DIRS} or absolute paths as shown in second and third line may be used.
# An update, link_directories is being pulled out, you may use TARGET_LINK_LIBRARIES instead as shown below
link_directories(${Boost_LIBRARY_DIRS}
	${OPENGL_LIBRARY_DIRS}
	${Qt4_LIBRARY_DIRS}
	${OMPL_LIBRARY_DIRS}
	${OMPLAPP_LIBRARY_DIRS}
	${TINYXML_LIBRARY_DIRS}
)

# We have commented out ROSSpaceInformation.cpp so that
# this project can build and run without ROS.
# To do ros based stuff, add catkinized cmake directives
# to turn it into a ros node and compile for example
# M3PROSDemo.cpp as executable instead of main.
add_library (bsp_lib
	src/MotionModels/TwoDPointMotionModel.cpp
	src/MotionModels/OmnidirectionalMotionModel.cpp
	src/MotionModels/UnicycleMotionModel.cpp
	src/MotionModels/UnicycleStatePropagator.cpp
	src/MotionModels/TwoDPointStatePropagator.cpp
	src/ObservationModels/CamAruco2DObservationModel.cpp
	src/Planner/FIRM.cpp
	src/Planner/NBM3P.cpp
	src/Samplers/GaussianValidBeliefSampler.cpp
	src/Samplers/UniformValidBeliefSampler.cpp
	src/SeparatedControllers/RHCICreate.cpp
	#src/SpaceInformation/ROSSpaceInformation.cpp
	src/SpaceInformation/SpaceInformation.cpp
	src/Spaces/SE2BeliefSpace.cpp
	src/Spaces/R2BeliefSpace.cpp
	src/Utils/FIRMUtils.cpp
	src/Visualization/GLWidget.cpp
	src/Visualization/Visualizer.cpp
	src/Visualization/Window.cpp
	src/Visualization/moc_GLWidget.cpp
	src/Visualization/moc_Window.cpp
	src/Filters/ExtendedKF.cpp
	src/Filters/LinearizedKF.cpp
)


target_link_libraries (bsp_lib
	${Boost_LIBRARIES}
	${QT_LIBRARIES}
	${OPENGL_LIBRARIES}
	${GLUT_LIBRARIES}
	${TINYXML_LIBRARIES}
	${ARMADILLO_LIBRARIES}
	${OMPL_LIBRARY}
	${OMPLAPP_LIBRARY}
)


# Add the main as executable
add_executable (bsp-app-demo src/main.cpp)

target_link_libraries (bsp-app-demo
	bsp_lib
	${OMPLAPP_LIBRARY}
	${OMPLAPPBASE_LIBRARY}
)


