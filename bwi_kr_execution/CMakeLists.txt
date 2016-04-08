cmake_minimum_required(VERSION 2.8.3)
project(bwi_kr_execution)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs 
  bwi_msgs
  message_generation
  roscpp
  roslib
  sound_play
  std_msgs
  std_srvs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
    AnswerSet.msg
    AspFluent.msg
    AspRule.msg
    NewPlan.msg)

## Generate services in the 'srv' folder
add_service_files(
  FILES
    ComputeAllPlans.srv
    ComputePlan.srv
    CurrentStateQuery.srv
    IsPlanValid.srv
    UpdateFluents.srv
    MultirobotComputePlan.srv
    MultirobotUpdateFluents.srv)

## Generate actions in the 'action' folder
add_action_files(FILES ExecutePlan.action)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    std_msgs)
 
catkin_package(
  INCLUDE_DIRS actasp/include
  LIBRARIES actasp
  CATKIN_DEPENDS
    actionlib_msgs
    bwi_msgs
    message_runtime
    roscpp
    sound_play
    std_msgs
    std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  actasp/include
)

set(actasp_SRC)

add_subdirectory(actasp/src/)

add_library(actasp ${actasp_SRC})
target_link_libraries(actasp ${catkin_LIBRARIES})

set(kractions_SRC)
add_subdirectory(src/actions/)
add_library(kractions ${kractions_SRC})
add_dependencies(kractions ${catkin_EXPORTED_TARGETS})
add_dependencies(kractions ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(kractions actasp)

set(multirobot_kractions_SRC)
add_subdirectory(src/multirobot_actions)
add_library(multirobot_kractions ${multirobot_kractions_SRC})
add_dependencies(multirobot_kractions ${catkin_EXPORTED_TARGETS})
add_dependencies(multirobot_kractions ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(multirobot_kractions actasp)

set(spexec_SRC)
set(anyexec_SRC)
set(lexec_SRC)
set(multiexec_SRC)
set(krreasoner_SRC)
set(multikr_SRC)
set(multicr_SRC)

add_subdirectory(src)
add_executable(single_plan_executor_node ${spexec_SRC})
target_link_libraries(single_plan_executor_node
  ${catkin_LIBRARIES} actasp kractions)

add_executable(any_plan_executor_node ${anyexec_SRC})
target_link_libraries(any_plan_executor_node
  ${catkin_LIBRARIES} actasp kractions)

add_executable(learning_executor_node ${lexec_SRC})
target_link_libraries(learning_executor_node
  ${catkin_LIBRARIES} actasp kractions)

add_executable(multirobot_executor_node ${multiexec_SRC})
target_link_libraries(multirobot_executor_node
  ${catkin_LIBRARIES} actasp multirobot_kractions)
 
add_executable(knowledge_reasoning_node  ${krreasoner_SRC})
target_link_libraries(knowledge_reasoning_node
  ${catkin_LIBRARIES} actasp kractions)

add_executable(multirobot_knowledge_reasoning_node  ${multikr_SRC})
target_link_libraries(multirobot_knowledge_reasoning_node
  ${catkin_LIBRARIES} actasp multirobot_kractions)

add_executable(multirobot_central_reasoning_node  ${multicr_SRC})
target_link_libraries(multirobot_central_reasoning_node
  ${catkin_LIBRARIES} actasp)

add_executable(asp_formatter src/asp_formatter.cpp)
target_link_libraries(asp_formatter actasp)

add_executable(state_tracker src/state_tracker.cpp src/msgs_utils.cpp)
target_link_libraries(state_tracker kractions)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

foreach(dir domain domain_dialog domain_simulation launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

install(TARGETS actasp kractions
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS
          any_plan_executor_node
          asp_formatter
          knowledge_reasoning_node
          learning_executor_node
          single_plan_executor_node
	  state_tracker
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
