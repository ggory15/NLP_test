# INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
# INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)

# ADD_DEFINITIONS(-DTESTS_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
ADD_DEFINITIONS(-DCONFIGS_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/configs")

MACRO(ADD_TESTCASE NAME GENERATED)
  IF (${GENERATED} STREQUAL TRUE)
    ADD_UNIT_TEST(${NAME} ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.cc)
  ELSE()
    ADD_UNIT_TEST(${NAME} ${NAME}.cc)
  ENDIF()

  # Link against Boost and project library.
  TARGET_LINK_LIBRARIES(${NAME}
    ${Boost_LIBRARIES} 
    ${PROJECT_NAME}
    yaml-cpp
    -lipopt
    #${catkin_LIBRARIES}
   # ${QPOASES_LIBRARY} 
    )
ENDMACRO(ADD_TESTCASE)
#ADD_TESTCASE(test_LPQP FALSE)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
ADD_TESTCASE(test_NLP FALSE)
#ADD_TESTCASE(test_ipopt FALSE)
#ADD_TESTCASE(test_Formulation FALSE)
#ADD_TESTCASE(ifopt_ipopt FALSE)
