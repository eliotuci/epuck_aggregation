cmake_minimum_required(VERSION 2.8)

SET ( EXECUTABLE dsim )

add_definitions(-D_GRAPHICS_)
FIND_PACKAGE( Qt COMPONENTS QtOpenGL REQUIRED )
FIND_PACKAGE( Qt4 REQUIRED )
SET( QT_USE_QTOPENGL TRUE )
SET(  QT_USE_QTXML TRUE )
INCLUDE( ${QT_USE_FILE} )
FIND_PACKAGE ( GLUT )
FIND_PACKAGE ( OpenGL )

FILE ( GLOB SOURCES ./*.cpp ./MISC/*.cpp ./EA/*.cpp ./CONTROLLERS/*.cpp ./EXP/*.cpp ./WORLD/*.cpp ./RENDERING/*.cpp ) # 
FILE ( GLOB HEADERS ./*.h ./MISC/*.h ./EA/*.h ./CONTROLLERS/*.h ./EXP/*.h ./WORLD/*.h ./RENDERING/*.h ) # 
FILE ( GLOB UIS ./*.ui ./RENDERING/*.ui ) # 

QT4_WRAP_CPP( SOURCES ${HEADERS} )
QT4_WRAP_UI( SOURCES  ${UIS} )

FIND_PACKAGE ( MPI )

INCLUDE_DIRECTORIES( ${BULLET_INCLUDE_DIR} /usr/include/bullet/ ${MPI_INCLUDE_PATH} /usr/include/ /usr/include/openmpi/ /usr/include/mpi/ /usr/local/include/ ${CMAKE_CURRENT_BINARY_DIR} )

# To add c++11 support
if (MSVC) # MSVC compiler (Win32 only)
        # Display more warnings
        SET (CMAKE_CXX_FLAGS "/W3")
elseif (UNIX OR CMAKE_COMPILER_IS_GNUCXX) # Clang OR Gcc (Linux, Mac OS or Win32 with MingW)
        # Enable C++11 and displays all warnings
        SET (CMAKE_CXX_COMPILER mpicxx)
        SET (CMAKE_CXX_FLAGS "-pthread -lstdc++ -std=c++11")
        if (APPLE) # Clang / Mac OS only
                # Required on OSX to compile c++11
                SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++ -mmacosx-version-min=10.7")
        endif(APPLE)
endif()

ADD_EXECUTABLE( ${EXECUTABLE} ${SOURCES})
TARGET_LINK_LIBRARIES( ${EXECUTABLE}  ${QT_LIBRARIES} ${OPENGL_glu_LIBRARY}  ${GLUT_LIBRARIES} -lGL -lGLU -lQGLViewer-qt4 -lqwt)
TARGET_LINK_LIBRARIES( ${EXECUTABLE} "-lmpi -L/sw/lib -lgsl -lgslcblas -lm -lBulletSoftBody -lBulletDynamics -lBulletCollision -lLinearMath")
