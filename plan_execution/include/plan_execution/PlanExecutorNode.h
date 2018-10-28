#include <actionlib/server/simple_action_server.h>
#include <plan_execution/observers.h>

namespace plan_exec {
class PlanExecutorNode {

  typedef actionlib::SimpleActionServer<plan_execution::ExecutePlanAction> Server;

public:
  PlanExecutorNode(actasp::PlanExecutor* executor);

  Server& getActionServer() {return server;}

  void setRosObserver(RosActionServerInterfaceObserver* ros_observer);

  ~PlanExecutorNode();
private:
  void executePlan(const plan_execution::ExecutePlanGoalConstPtr &plan);
  std::unique_ptr<actasp::PlanExecutor> executor;
  Server server;

  std::unique_ptr<RosActionServerInterfaceObserver> ros_observer;

};
}
