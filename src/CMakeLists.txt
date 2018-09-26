SET(LIBRARY_NAME ${PROJECT_NAME})

SET(${LIBRARY_NAME}_SOURCES
  utils/Box.cc
  utils/ProblemConfig.cc
  utils/Printer.cc

  functions/BoxAbovePlan.cc
  functions/BoxAboveFixedPlan.cc
  functions/CostDistance.cc
  functions/FixedBoxPosition.cc
  functions/PlanBetweenBoxAndObstacle.cc

  # solvers/QP.cc
  # solvers/QPPlanesFixed.cc
  # solvers/QPBoxesFixed.cc
  # solvers/QPBoxesFixedIndividual.cc
  # solvers/LPQPsolver.cc

  solvers/NLP/NLP_formulation.cc
  solvers/NLP/NLP_variable.cc
  solvers/NLP/NLP_cost.cc
  solvers/NLP/NLP_constraint.cc

  TrajectoryProblem.cc
  BoxesHullTrajProblem.cc

  solvers/NLP/MyNLP.cc

)
ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  ${${LIBRARY_NAME}_SOURCES}
  )

#PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} qpOASES)

#add_executable(exp1 test.cc)
TARGET_LINK_LIBRARIES (${LIBRARY_NAME} 
${Boost_LIBRARIES} 
#${QPOASES_LIBRARY} 
${catkin_LIBRARIES}
) #fixme)
INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)
