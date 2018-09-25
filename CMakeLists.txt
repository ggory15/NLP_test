CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0)

SET(CXX_DISABLE_WERROR TRUE)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/test.cmake)

set(BUILD_DOCUMENTATION OFF)
#list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake )
set(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall -Wwrite-strings -pedantic -O3 -funroll-loops -fPIC")
SET(PROJECT_NAME NLP-foot)
SET(PROJECT_DESCRIPTION
  "Foor Trajectory Generation by Using Hyper Plane"
  )
SET(PROJECT_URL "")

SETUP_PROJECT()
SEARCH_FOR_BOOST()

ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.0")
ADD_REQUIRED_DEPENDENCY("yaml-cpp >= 0.5")
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${YAML-CPP_INCLUDE_DIRS})
find_package(catkin COMPONENTS ifopt)
include_directories(${catkin_INCLUDE_DIRS})
#include_directories(/user/include/coin)
add_definitions(-DHAVE_STDDEF_H)

set(INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
SET(${PROJECT_NAME}_HEADERS
	include/hpp/foot/utils/Box.hh
  include/hpp/foot/utils/FixedPlan.hh
  include/hpp/foot/utils/PlanForHull.hh
  include/hpp/foot/utils/ProblemConfig.hh
  include/hpp/foot/utils/defs.hh
  include/hpp/foot/utils/Printer.hh
  #
  include/hpp/foot/functions/BoxAbovePlan.hh
  include/hpp/foot/functions/BoxAboveFixedPlan.hh
  include/hpp/foot/functions/CostDistance.hh
  include/hpp/foot/functions/FixedBoxPosition.hh
  include/hpp/foot/functions/PlanBetweenBoxAndObstacle.hh
  #
  include/hpp/foot/solvers/LPQPsolver.hh
  include/hpp/foot/solvers/QP.hh
  include/hpp/foot/solvers/QPPlanesFixed.hh
  include/hpp/foot/solvers/QPBoxesFixed.hh
  include/hpp/foot/solvers/QPBoxesFixedIndividual.hh
  
  include/hpp/foot/solvers/NLP/NLP_formulation.hh
  include/hpp/foot/solvers/NLP/NLP_variable.hh
  include/hpp/foot/solvers/NLP/NLP_constraint.hh
  include/hpp/foot/solvers/NLP/NLP_cost.hh
  include/hpp/foot/solvers/NLP/MyNLP.hh
  #
  include/hpp/foot/TrajectoryProblem.hh
  include/hpp/foot/BoxesHullTrajProblem.hh
)

#

add_subdirectory (src)
add_subdirectory (demo)

PKG_CONFIG_APPEND_LIBS("NLP-foot")

SETUP_PROJECT_FINALIZE()
