

SET( multirobot_kractions_SRC ${multirobot_kractions_SRC}
	${CMAKE_CURRENT_SOURCE_DIR}/ActionFactory.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/LogicalNavigation.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/ApproachDoor.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/GoThrough.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/OpenSimulatedDoor.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/OpenDoor.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/CallGUI.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/GoTo.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Wait.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/WaitForOpen.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/ApproachArea.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../StaticFacts.cpp
	PARENT_SCOPE)
