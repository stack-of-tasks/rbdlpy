#
#  Copyright 2014 CNRS
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME rbdlpy)
SET(PROJECT_DESCRIPTION "Bindings Python for RBDL")
SET(PROJECT_URL "http://github.com/stac-of-tasks/rbdlpy")

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)
SET(CMAKE_VERBOSE_MAKEFILE True)

project(${PROJECT_NAME})
SETUP_PROJECT()

# ----------------------------------------------------
# --- DEPENDANCIES -----------------------------------
# ----------------------------------------------------
ADD_REQUIRED_DEPENDENCY("eigenpy >= v1.0.1")
ADD_REQUIRED_DEPENDENCY("rbdl")
#ADD_REQUIRED_DEPENDENCY("rbdl_urdfreader")
#ADD_REQUIRED_DEPENDENCY("rbdl_luamodel")

MESSAGE ( ${RBDL_LIBRARIES} )
#MESSAGE ( ${RBDL_LUAMODEL_LIBRARY_DIRS} )

# ----------------------------------------------------
# --- INCLUDE ----------------------------------------
# ----------------------------------------------------
SET(${PROJECT_NAME}_HEADERS
  src/rbdlpy.hpp
)
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/rbdlpy")

FOREACH(header ${${PROJECT_NAME}_HEADERS})
  GET_FILENAME_COMPONENT(headerName ${header} NAME)
  IF(WIN32)
    execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    ${${PROJECT_NAME}_SOURCE_DIR}/${header}
                    ${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/)
  ELSE(WIN32)
    execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
                    ${${PROJECT_NAME}_SOURCE_DIR}/${header}
                    ${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/${headerName})
  ENDIF(WIN32)
  INSTALL(FILES ${${PROJECT_NAME}_SOURCE_DIR}/${header}
	  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}
          PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)
ENDFOREACH(header)

# ----------------------------------------------------
# --- TARGETS ----------------------------------------
# ----------------------------------------------------
ADD_LIBRARY(simple SHARED unittest/simple.cpp)
PKG_CONFIG_USE_DEPENDENCY(simple eigenpy)
TARGET_LINK_LIBRARIES ( simple ${RBDL_LIBRARIES} )
#PKG_CONFIG_USE_DEPENDENCY(simple rbdl)

ADD_LIBRARY(body SHARED unittest/body.cpp)
PKG_CONFIG_USE_DEPENDENCY(body eigenpy)
TARGET_LINK_LIBRARIES ( body ${RBDL_LIBRARIES} )

ADD_LIBRARY(joint SHARED unittest/joint.cpp)
PKG_CONFIG_USE_DEPENDENCY(joint eigenpy)
TARGET_LINK_LIBRARIES ( joint ${RBDL_LIBRARIES} )

ADD_LIBRARY(model SHARED unittest/model.cpp)
PKG_CONFIG_USE_DEPENDENCY(model eigenpy)
TARGET_LINK_LIBRARIES ( model ${RBDL_LIBRARIES} )

ADD_LIBRARY(kinematics SHARED unittest/kinematics.cpp)
PKG_CONFIG_USE_DEPENDENCY(kinematics eigenpy)
TARGET_LINK_LIBRARIES ( kinematics ${RBDL_LIBRARIES} )

ADD_LIBRARY(dynamics SHARED unittest/dynamics.cpp)
PKG_CONFIG_USE_DEPENDENCY(dynamics eigenpy)
TARGET_LINK_LIBRARIES ( dynamics ${RBDL_LIBRARIES} )

ADD_LIBRARY(urdfreader SHARED unittest/urdfreader.cpp)
PKG_CONFIG_USE_DEPENDENCY(urdfreader eigenpy)
TARGET_LINK_LIBRARIES ( urdfreader ${RBDL_LIBRARIES} )
TARGET_LINK_LIBRARIES ( urdfreader ${RBDL_URDFREADER_LIBRARIES} )

ADD_LIBRARY(luamodel SHARED unittest/luamodel.cpp)
PKG_CONFIG_USE_DEPENDENCY(luamodel eigenpy)
TARGET_LINK_LIBRARIES ( luamodel ${RBDL_LIBRARIES} )
TARGET_LINK_LIBRARIES ( luamodel ${RBDL_LUAMODEL_LIBRARIES} )

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()

