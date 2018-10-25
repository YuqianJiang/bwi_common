#include <pybind11/embed.h> // everything needed for embedding
#include <pybind11/stl.h>
#include <ros/ros.h>
#include <ros/package.h>
namespace py = pybind11;

int main(int argc, char **argv) {
		ros::init(argc, argv, "pybind_example");
		ros::NodeHandle nh;

		std::string path = ros::package::getPath("bwi_kr_execution") + "/src/bwi_kr_execution";

    py::scoped_interpreter guard{}; // start the interpreter and keep it alive

    py::module sys = py::module::import("sys");
    sys.attr("path").attr("append")(path);
    //py::print(sys.attr("path"));

    /*py::exec(R"(
    		import os,sys
    		print(os.path.dirname(os.path.realpath(sys.path[-1])))
    	)"); */

    py::object learner_class = py::module::import("tmp_learner").attr("TMPLearner");
    py::object learner = learner_class();

    std::vector<std::string> state = {"is_near(o3_410f)"};
    std::vector<std::string> state_next = {"is_near(d3_414a1)"};
    learner.attr("learn")(state, state_next, "navigate_to(d3_414a1)", 100);
    learner.attr("ro_table_to_asp")();

    std::pair<int, int> state_action = {0, 0};
    std::vector<std::pair<int, int>> state_action_path = {state_action};
    learner.attr("constrain_plan_quality")(state_action_path);

}