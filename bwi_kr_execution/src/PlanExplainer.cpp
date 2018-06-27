#include "PlanExplainer.h"

#include <ros/ros.h>

#include <string>
#include <iterator>
#include <algorithm>
#include <numeric>

using namespace std;
using namespace actasp;

namespace bwi_krexec {

const double discount_factor = 0.7;
const double starting_learning_rate = 0.5;
const int default_horizon = 10;
const int default_n_epochs = 500;

struct ExplanationState{

  ExplanationState(const vector<Predicate>& predicates, 
                  const map<Predicate, int, PredicateComp>& predicateMap) : 
  predicates(predicates), 
  features(predicateMap.size()) {

    for (vector<Predicate>::const_iterator it = predicates.begin(); it != predicates.end(); ++it) {
      if (predicateMap.find(*it) != predicateMap.end()) {
        ++features[predicateMap.at(*it)];
      }
    }
  }

  string toString() {
    stringstream ss;
    copy(predicates.begin(),predicates.end(),ostream_iterator<string>(ss," "));
    copy(next.begin(),next.end(),ostream_iterator<int>(ss," "));
    return ss.str();
  }

  vector<Predicate> predicates; //unexplained predicates
  vector<int> previous;
  vector<int> next;

  vector<int> features;
};

struct IsPredicateExplained {
  IsPredicateExplained(Predicate& predicate) : predicate(predicate) {}
  bool operator() (const Predicate& other) {
    return (other == predicate) || (this->predicate.fluent.getTimeStep() > other.fluent.getTimeStep());
  }
  Predicate predicate;
};

string fluent2Text(const AspFluent& fluent, bool isTrue);

void PlanExplainer::setPlan(const AnswerSet& newFluents) {

  plan.clear();

  for (int t = 1; t <= newFluents.maxTimeStep(); ++t) {

    vector<Predicate> changesAtTime = getFluentDifference(newFluents.getFluentsAtTime(t-1), newFluents.getFluentsAtTime(t));
    plan.insert(plan.end(), changesAtTime.begin(), changesAtTime.end());
    
    //stringstream predicatesStream;
    //copy(changesAtTime.begin(),changesAtTime.end(),ostream_iterator<string>(predicatesStream," "));
    //ROS_INFO_STREAM("changes at " << t << ": " << predicatesStream.str());
  }

  plan.push_back(Predicate(AspFluent("terminate", vector<string>(), newFluents.maxTimeStep()+1), true));
  
  liftPlan(plan);

  //buildStateSpace(plan);
}

string PlanExplainer::getAllPairs() {

  stringstream feedback;

  int count = 0;

  for (vector<Predicate>::iterator it1 = plan.begin(); it1 != plan.end(); ++it1) {

    for (vector<Predicate>::iterator it2 = it1; it2 != plan.end(); ++it2) {
      feedback << fluent2Text(it1->grounded, it1->isTrue) << ", ";
      feedback << fluent2Text(it2->grounded, it2->isTrue) << "\n";
      count++;
    }
  }

  feedback << "Number of possible explanations: " << count;

  return feedback.str();
}

string PlanExplainer::getRandomExplanation(const int length) {

  srand (time(NULL));

  if (plan.size() == 0) return "";

  stringstream explanation_ss;
  int current_i = -1;
  int i = 0;
  int count = 0;

  while ((count < length) && ((length-count) < (plan.size()-current_i))) {
    i = rand() % ((plan.size()-current_i) - (length-count)) + current_i + 1;
    explanation_ss << fluent2Text(plan[i].grounded, plan[i].isTrue) << ", ";
    current_i = i;
    count++;
  }
  
  for (; (count < length) && (i < plan.size()); ++i) {
    explanation_ss << fluent2Text(plan[i].grounded, plan[i].isTrue) << ", ";
  }

  string explanation = explanation_ss.str();
  return explanation.substr(0, explanation.size()-2) + ".\n";

}

string PlanExplainer::getRandomExplanation() {
  srand (time(NULL));

  if (plan.size() == 0) return "";

  stringstream explanation_ss;

  vector<ExplanationState> mdp = buildStateSpace(plan);

  int state = 0;
  int length = 0;
  while ((mdp[state].next.size() != 0) && (length < 4)) {
    
    vector<int> nextStates = mdp[state].next;
    int nextStateIndex = rand() % nextStates.size();
    ROS_INFO_STREAM(nextStates[nextStateIndex]);
    Predicate chosenPredicate = mdp[state].predicates[nextStateIndex];

    if (chosenPredicate.fluent.getName() != "terminate")
      explanation_ss << fluent2Text(chosenPredicate.grounded, chosenPredicate.isTrue) << ", ";
    
    state = nextStates[nextStateIndex];
    ++length;
  }

  string explanation = explanation_ss.str();
  explanation = explanation.substr(0, explanation.size()-2) + ".\n";
  ROS_INFO_STREAM(explanation);
  return explanation;

}

string PlanExplainer::getLearnedExplanation() {
  if (plan.size() == 0) return "";
  
  stringstream explanation_ss;

  vector<ExplanationState> mdp = buildStateSpace(plan);
  vector<vector<double> > policy = calcMaxEntPolicy(mdp, default_horizon);

  for (vector<Predicate>::iterator it = plan.begin(); it != plan.end(); ++it) {
    if (predicateMap.find(*it) != predicateMap.end()) {
      ROS_INFO_STREAM(it->toString() << ": " << weights[predicateMap.at(*it)]);
    }
  }

  /*vector<double> start_dist(mdp.size());
  start_dist[0] = 1;
  vector<double> state_freq = calcExpectedStateFreq(mdp, start_dist, policy, default_horizon);

  vector<double> feature_freq_plan(predicateMap.size());
  for (int j = 0; j < mdp.size(); ++j) {
    ExplanationState state = mdp[j];
    vector<double> feature_freq;
    transform(state.features.begin(), state.features.end(), 
              back_inserter(feature_freq), bind2nd(multiplies<double>(), state_freq[j]));
    transform(feature_freq_plan.begin(), feature_freq_plan.end(), feature_freq.begin(), feature_freq_plan.begin(), plus<double>());
  }

  stringstream feature_freq_ss;
  copy(feature_freq_plan.begin(), feature_freq_plan.end(), ostream_iterator<double>(feature_freq_ss, " "));
  ROS_INFO_STREAM("feature frequency: " << feature_freq_ss.str());*/

  /*stringstream state_freq_ss;
  copy(state_freq.begin(), state_freq.end(), ostream_iterator<double>(state_freq_ss, " "));
  ROS_INFO_STREAM("state frequency: " << state_freq_ss.str());*/

  int state = 0;
  while (policy[state].size() != 0) {
    /*stringstream policy_ss;
    copy(policy[state].begin(), policy[state].end(), ostream_iterator<double>(policy_ss, " "));
    ROS_INFO_STREAM("policy[ " << state << "]: " << policy_ss.str());*/

    int nextStateIndex = distance(policy[state].begin(), max_element(policy[state].begin(), policy[state].end()));
    
    vector<int> nextStates = mdp[state].next;
    Predicate chosenPredicate = mdp[state].predicates[nextStateIndex];

    if (chosenPredicate.fluent.getName() != "terminate")
      explanation_ss << fluent2Text(chosenPredicate.grounded, chosenPredicate.isTrue) << ", ";
    state = nextStates[nextStateIndex];
  }

  string explanation = explanation_ss.str();
  explanation = explanation.substr(0, explanation.size()-2) + ".\n";
  ROS_INFO_STREAM(explanation);
  return explanation;
}

vector<Predicate> PlanExplainer::getFluentDifference(const set<AspFluent>& set1, 
                                                      const set<AspFluent>& set2) {

  vector<Predicate> result;

  set<AspFluent>::const_iterator it1 = set1.begin();
  set<AspFluent>::const_iterator it2 = set2.begin();

  int timeStep = it2->getTimeStep();

  while (true) {
    if (it1 == set1.end()) {
      //transform(it2, set2.end(), back_inserter(result), boost::bind(make_pair<AspFluent, bool>, _1, true));
      for (; it2 != set2.end(); ++it2) {
        if (! isAnAction(*it2)) {
          result.push_back(Predicate(*it2, true)); 
        }
      }
      return result;
    }
    if (it2==set2.end()) {
      //transform(it1, set1.end(), back_inserter(result), boost::bind(make_pair<AspFluent, bool>, _1, true));
      for (; it1 != set1.end(); ++it1) {
        if (! isAnAction(*it1)) {
          AspFluent change(it1->getName(), it1->getParameters(), timeStep);
          result.push_back(Predicate(change, false)); 
        }
      }
      return result;
    }
    if (it1->toString(0) < it2->toString(0)) {
      if (! isAnAction(*it1)) {
        AspFluent change(it1->getName(), it1->getParameters(), timeStep);
        result.push_back(Predicate(change, false));
      }
      ++it1; 
    }
    else if (it2->toString(0) < it1->toString(0)) { 
      if (! isAnAction(*it2)) {
        result.push_back(Predicate(*it2, true)); 
      }
      ++it2; 
    }
    else {
      ++it1; 
      ++it2; 
    }
  }

  sort(result.begin(), result.end());

  return result;
}

vector<ExplanationState> PlanExplainer::buildStateSpace(const vector<Predicate>& predicates) {
  vector<ExplanationState> states;

  map<vector<Predicate>, int> visited;

  ExplanationState initial(predicates, predicateMap); 
  states.push_back(initial);
  visited[predicates] = 0;

  for (int i = 0; i < states.size(); ++i) {
    ExplanationState state = states[i];


    for (vector<Predicate>::iterator predIt = state.predicates.begin(); predIt != state.predicates.end(); ++predIt) {
      
      vector<Predicate> unexplained;
      remove_copy_if(state.predicates.begin(), state.predicates.end(), back_inserter(unexplained), IsPredicateExplained(*predIt));

      int index;
      if (visited.find(unexplained) != visited.end()) {
        index = visited.at(unexplained);
      }
      else {
        index = states.size();
        visited[unexplained] = index;
        states.push_back(ExplanationState(unexplained, predicateMap));
      }

      states[i].next.push_back(index);
      states[index].previous.push_back(i);
    }

    //ROS_INFO_STREAM("state " << i << ": " << states[i].toString());
  }

  return states;
}

void Predicate::lift(EntityGeneralizer& entityGeneralizer) {
  
  if (!isGrounded()) return;
  if (this->fluent.getName() == "terminate") return;

  vector<string> groundedVar = this->fluent.getParameters();
  vector<string> liftedVar;
  for (vector<string>::iterator it = groundedVar.begin(); it != groundedVar.end(); ++it) {
    liftedVar.push_back(entityGeneralizer(*it));
  }

  this->grounded = this->fluent;
  this->fluent = AspFluent(this->grounded.getName(), liftedVar, this->grounded.getTimeStep());

}

void PlanExplainer::liftPlan(std::vector<Predicate>& predicates) {
  EntityGeneralizer entityGeneralizer;

  for (vector<Predicate>::iterator it = predicates.begin(); it != predicates.end(); ++it) {
    it->lift(entityGeneralizer);
  }

  /*stringstream predicatesStream;
  predicatesStream << "plan changes: ";
  transform(predicates.begin(),predicates.end(),ostream_iterator<string>(predicatesStream," "), boost::bind(&Predicate::toString, _1, true));
  ROS_INFO_STREAM(predicatesStream.str());*/
}

vector<vector<double> > PlanExplainer::calcMaxEntPolicy(const vector<ExplanationState>& mdp, 
                                        const int horizon) {
  int n_states = mdp.size();
  vector<vector<double> > z_a;
  vector<double> z_s(n_states);
  int terminal = *(mdp[0].next.rbegin());

  z_s[terminal] = 1;

  for (int i = 0; i < n_states; ++i) {
    z_a.push_back(vector<double>(mdp[i].next.size()));
  }

  for (int n = 0; n < horizon; ++n) {

    for (int i = 0; i < n_states; ++i) {

      double dot = inner_product(weights.begin(), weights.end(), mdp[i].features.begin(), 0.0) * discount_factor;

      for (int j = 0; j < mdp[i].next.size(); ++j) {
        //z_a[i][j] += trans_mat[i,j,k]*np.exp(np.dot(r_weights, state_features[i]))*z_s[k]
        z_a[i][j] = exp(dot) * z_s[mdp[i].next[j]];
      }
    }

    for (int i = 0; i < n_states; ++i) {
      z_s[i] = accumulate(z_a[i].begin(), z_a[i].end(), 0.0);
    }

    z_s[terminal]++;

    /*
    stringstream z_s_ss;
    copy(z_s.begin(), z_s.end(), ostream_iterator<double>(z_s_ss, " "));
    ROS_INFO_STREAM("z_s: " << z_s_ss.str());

    stringstream z_a_ss;
    copy(z_a[0].begin(), z_a[0].end(), ostream_iterator<double>(z_a_ss, " "));
    ROS_INFO_STREAM("z_a[" << 0 << "]: " << z_a_ss.str());
    */
  }

  for (int i = 0; i < n_states; ++i) {
    transform(z_a[i].begin(), z_a[i].end(), z_a[i].begin(), bind2nd(divides<double>(), z_s[i]));
  }

  return z_a;

}

vector<double> PlanExplainer::calcExpectedStateFreq(const vector<ExplanationState>& mdp,
                                                    const vector<double>& start_dist,
                                                    const vector<vector<double> >& policy,
                                                    const int horizon) {
  int n_states = mdp.size();
  vector<double> state_freq(start_dist);
  vector<double> d_t(n_states);
  vector<double> d_next(start_dist);

  for (int t = 0; t < horizon; ++t) {
    d_t = d_next;
    d_next = vector<double>(n_states);

    for (int i = 0; i < n_states; ++i) {
      for (int j = 0; j < mdp[i].next.size(); ++j) {
        int k = mdp[i].next[j];
        d_next[k] += d_t[i] * policy[i][j];
      }
    }

    //state_freq += D_next
    transform(state_freq.begin(), state_freq.end(), d_next.begin(), state_freq.begin(), plus<double>());
  }

  return state_freq;
}

void PlanExplainer::irl(const vector<vector<ExplanationState> >& mdps, 
                        const vector<vector<double> >& feature_expectation,
                        const int n_epochs,
                        const int horizon,
                        const double learning_rate) {

  double l = learning_rate;

  for (int e = 0; e < n_epochs; ++e) {

    //l -= learning_rate/n_epochs;

    for (int i = 0; i < mdps.size(); ++i) {
      vector<double> gradient(feature_expectation[i]);

      vector<vector<double> > policy = calcMaxEntPolicy(mdps[i], horizon);

      vector<double> start_dist(mdps[i].size());
      start_dist[0] = 1;

      vector<double> state_freq = calcExpectedStateFreq(mdps[i], start_dist, policy, horizon);

      /*stringstream state_freq_ss;
      copy(state_freq.begin(), state_freq.end(), ostream_iterator<double>(state_freq_ss, " "));
      ROS_INFO_STREAM("state frequency: " << state_freq_ss.str());*/

      vector<double> feature_freq_plan(feature_expectation[i].size());

      for (int j = 0; j < mdps[i].size(); ++j) {
        ExplanationState state = mdps[i][j];
        vector<double> feature_freq;

        //gradient -= features*state_freq[j]
        transform(state.features.begin(), state.features.end(), 
                  back_inserter(feature_freq), bind2nd(multiplies<double>(), state_freq[j]));

        transform(gradient.begin(), gradient.end(), feature_freq.begin(), gradient.begin(), minus<double>());
        transform(feature_freq_plan.begin(), feature_freq_plan.end(), feature_freq.begin(), feature_freq_plan.begin(), plus<double>());
      }

      //weights += gradient*learning_rate
      transform(gradient.begin(), gradient.end(), gradient.begin(), bind2nd(multiplies<double>(), l));
      transform(weights.begin(), weights.end(), gradient.begin(), weights.begin(), plus<double>());

      /*stringstream feature_freq_ss;
      copy(feature_freq_plan.begin(), feature_freq_plan.end(), ostream_iterator<double>(feature_freq_ss, " "));
      ROS_INFO_STREAM("feature frequency[" << i << "]: " << feature_freq_ss.str());

      stringstream gradient_ss;
      copy(gradient.begin(), gradient.end(), ostream_iterator<double>(gradient_ss, " "));
      ROS_INFO_STREAM("gradient[" << i << "]: " << gradient_ss.str());

      stringstream weights_ss;
      copy(weights.begin(), weights.end(), ostream_iterator<double>(weights_ss, " "));
      ROS_INFO_STREAM("weights[" << i << "]: " << weights_ss.str());*/


    }

    
    /*stringstream weights_ss;
    copy(weights.begin(), weights.end(), ostream_iterator<double>(weights_ss, " "));
    ROS_INFO_STREAM("weights: " << weights_ss.str());

    stringstream gradient_ss;
    copy(gradient.begin(), gradient.end(), ostream_iterator<double>(gradient_ss, " "));
    ROS_INFO_STREAM("gradient: " << gradient_ss.str());*/
  }

}

/*
void PlanExplainer::lfd() {

  vector<vector<Predicate> > plans;
  vector<vector<ExplanationState> > explanations;
  vector<vector<ExplanationState> > mdps;

  { 
    vector<Predicate> plan;
    plan.push_back(Predicate(AspFluent("at(l3_302,1)"), false));
    plan.push_back(Predicate(AspFluent("at(l3_300,1)"), true));
    plan.push_back(Predicate(AspFluent("beside(o3_302_printer,1)"), false));
    plan.push_back(Predicate(AspFluent("beside(d3_400,1)"), true));
    plan.push_back(Predicate(AspFluent("facing(o3_302_printer,1)"), false));
    plan.push_back(Predicate(AspFluent("facing(d3_400,1)"), true));
    plan.push_back(Predicate(AspFluent("open(d3_400,2)"), true));
    plan.push_back(Predicate(AspFluent("at(l3_300,3)"), false));
    plan.push_back(Predicate(AspFluent("at(l3_400,3)"), true));
    plan.push_back(Predicate(AspFluent("facing(d3_400,3)"), false));
    plan.push_back(Predicate(AspFluent("beside(d3_400,4)"), false));
    plan.push_back(Predicate(AspFluent("beside(d3_414b2,4)"), true));
    plan.push_back(Predicate(AspFluent("facing(d3_414b2,4)"), true));
    plan.push_back(Predicate(AspFluent("open(d3_414b2,5)"), true));
    plan.push_back(Predicate(AspFluent("at(l3_400,6)"), false));
    plan.push_back(Predicate(AspFluent("at(l3_414b,6)"), true));
    plan.push_back(Predicate(AspFluent("facing(d3_414b2,6)"), false));
    plan.push_back(Predicate(AspFluent("found(y,7)"), true));
    plan.push_back(Predicate(AspFluent("inroom(y,l3_414b,7)"), true));
    plan.push_back(Predicate(AspFluent("messagedelivered(y,m0,8)"), true));
    plan.push_back(Predicate(AspFluent("terminate(deliver1,9)"), true));
    
    plans.push_back(plan);
  }

  {
    vector<Predicate> plan;
    plan.push_back(Predicate(AspFluent("beside(o3_500_printer,1)"), false));
    plan.push_back(Predicate(AspFluent("beside(d3_414b1,1)"), true));
    plan.push_back(Predicate(AspFluent("facing(o3_500_printer,1)"), false));
    plan.push_back(Predicate(AspFluent("facing(d3_414b1,1)"), true));
    plan.push_back(Predicate(AspFluent("open(d3_414b1,2)"), true));
    plan.push_back(Predicate(AspFluent("at(l3_500,3)"), false));
    plan.push_back(Predicate(AspFluent("at(l3_414b,3)"), true));
    plan.push_back(Predicate(AspFluent("facing(d3_414b1,3)"), false));
    plan.push_back(Predicate(AspFluent("found(y,4)"), true));
    plan.push_back(Predicate(AspFluent("inroom(y,l3_414b,4)"), true));
    plan.push_back(Predicate(AspFluent("messagedelivered(y,m0,5)"), true));
    plan.push_back(Predicate(AspFluent("terminate(deliver2,6)"), true));

    plans.push_back(plan);
  }

  {
    vector<Predicate> plan;
    plan.push_back(Predicate(AspFluent("found(y,1)"), true));
    plan.push_back(Predicate(AspFluent("inroom(y,l3_516,1)"), true));
    plan.push_back(Predicate(AspFluent("messagedelivered(y,m0,2)"), true));
    plan.push_back(Predicate(AspFluent("terminate(deliver3,3)"), true));

    plans.push_back(plan);
  }

  { 
    vector<Predicate> plan;
    plan.push_back(Predicate(AspFluent("at(l3_302,1)"), false));
    plan.push_back(Predicate(AspFluent("at(l3_300,1)"), true));
    plan.push_back(Predicate(AspFluent("beside(o3_302_printer,1)"), false));
    plan.push_back(Predicate(AspFluent("beside(d3_400,1)"), true));
    plan.push_back(Predicate(AspFluent("facing(o3_302_printer,1)"), false));
    plan.push_back(Predicate(AspFluent("facing(d3_400,1)"), true));
    plan.push_back(Predicate(AspFluent("open(d3_400,2)"), true));
    plan.push_back(Predicate(AspFluent("at(l3_300,3)"), false));
    plan.push_back(Predicate(AspFluent("at(l3_400,3)"), true));
    plan.push_back(Predicate(AspFluent("facing(d3_400,3)"), false));
    plan.push_back(Predicate(AspFluent("beside(d3_400,4)"), false));
    plan.push_back(Predicate(AspFluent("beside(d3_414b2,4)"), true));
    plan.push_back(Predicate(AspFluent("facing(d3_414b2,4)"), true));
    plan.push_back(Predicate(AspFluent("open(d3_414b2,5)"), true));
    plan.push_back(Predicate(AspFluent("at(l3_400,6)"), false));
    plan.push_back(Predicate(AspFluent("at(l3_414b,6)"), true));
    plan.push_back(Predicate(AspFluent("facing(d3_414b2,6)"), false));
    plan.push_back(Predicate(AspFluent("terminate(goto1,7)"), true));
    
    plans.push_back(plan);
  }

 {
    vector<Predicate> plan;
    plan.push_back(Predicate(AspFluent("beside(o3_500_printer,1)"), false));
    plan.push_back(Predicate(AspFluent("beside(d3_414a2,1)"), true));
    plan.push_back(Predicate(AspFluent("facing(o3_500_printer,1)"), false));
    plan.push_back(Predicate(AspFluent("facing(d3_414a2,1)"), true));
    plan.push_back(Predicate(AspFluent("open(d3_414a2,2)"), true));
    plan.push_back(Predicate(AspFluent("at(l3_500,3)"), false));
    plan.push_back(Predicate(AspFluent("at(l3_414a,3)"), true));
    plan.push_back(Predicate(AspFluent("facing(d3_414a2,3)"), false));
    plan.push_back(Predicate(AspFluent("terminate(goto2,4)"), true));

    plans.push_back(plan);
  }

  { 
    vector<Predicate> plan;
    plan.push_back(Predicate(AspFluent("facing(d3_414b2,1)"), false));
    plan.push_back(Predicate(AspFluent("at(l3_400,1)"), false));
    plan.push_back(Predicate(AspFluent("at(l3_414b,1)"), true));
    plan.push_back(Predicate(AspFluent("terminate(goto3,2)"), true));
    
    plans.push_back(plan);
  }

  for (int i = 0; i < plans.size(); ++i) {
    liftPlan(plans[i]);

    vector<Predicate>::iterator it = plans[i].begin();
    for (; it != plans[i].end(); ++it) {
      //if ((it->fluent.getName() != "terminate") && (predicateMap.find(*it) == predicateMap.end())) {
      if ((predicateMap.find(*it) == predicateMap.end())) {
        int index = predicateMap.size();
        predicateMap[*it] = index;
      }
    }
  }

  {
    vector<ExplanationState> mdp = buildStateSpace(plans[0]);
    mdps.push_back(mdp);

    vector<ExplanationState> explanation;
    explanation.push_back(mdp[0]);
    explanation.push_back(mdp[16]);
    explanation.push_back(mdp[18]);
    explanation.push_back(mdp[20]);
    explanation.push_back(mdp[21]);

    explanations.push_back(explanation);
  }  

  {
    vector<ExplanationState> mdp = buildStateSpace(plans[1]);
    mdps.push_back(mdp);

    vector<ExplanationState> explanation;
    explanation.push_back(mdp[0]);
    explanation.push_back(mdp[7]);
    explanation.push_back(mdp[9]);
    explanation.push_back(mdp[11]);
    explanation.push_back(mdp[12]);

    explanations.push_back(explanation);
  }

  {
    vector<ExplanationState> mdp = buildStateSpace(plans[2]);
    mdps.push_back(mdp);

    vector<ExplanationState> explanation;
    explanation.push_back(mdp[0]);
    explanation.push_back(mdp[1]);
    explanation.push_back(mdp[3]);
    explanation.push_back(mdp[4]);

    explanations.push_back(explanation);
  }

  {
    vector<ExplanationState> mdp = buildStateSpace(plans[3]);
    mdps.push_back(mdp);

    vector<ExplanationState> explanation;
    explanation.push_back(mdp[0]);
    explanation.push_back(mdp[9]);
    explanation.push_back(mdp[12]);
    explanation.push_back(mdp[16]);
    explanation.push_back(mdp[18]);

    explanations.push_back(explanation);
  }  

  {
    vector<ExplanationState> mdp = buildStateSpace(plans[4]);
    mdps.push_back(mdp);

    vector<ExplanationState> explanation;
    explanation.push_back(mdp[0]);
    explanation.push_back(mdp[2]);
    explanation.push_back(mdp[7]);
    explanation.push_back(mdp[9]);

    explanations.push_back(explanation);
  }

  {
    vector<ExplanationState> mdp = buildStateSpace(plans[5]);
    mdps.push_back(mdp);

    vector<ExplanationState> explanation;
    explanation.push_back(mdp[0]);
    explanation.push_back(mdp[3]);
    explanation.push_back(mdp[4]);

    explanations.push_back(explanation);
  }

  vector<vector<double> > feature_expectation;

  //calculate feature expectation for each demo
  vector<vector<ExplanationState> >::const_iterator xIt = explanations.begin();
  for (; xIt != explanations.end(); ++xIt) {

    vector<double> feature_expectation_plan(predicateMap.size());
    vector<ExplanationState>::const_iterator stateIt = xIt->begin();
    for (; stateIt != xIt->end(); ++stateIt) {
      for (int i = 0; i < stateIt->features.size(); ++i) {
        feature_expectation_plan[i] += stateIt->features[i];
      }
    }

    //print feature expectation
    stringstream feature_expec_ss;
    copy(feature_expectation_plan.begin(), feature_expectation_plan.end(), ostream_iterator<double>(feature_expec_ss, " "));
    ROS_INFO_STREAM("feature expectation: " << feature_expec_ss.str());

    feature_expectation.push_back(feature_expectation_plan);
  }

  weights = vector<double>(predicateMap.size());

  irl(mdps, feature_expectation, default_n_epochs, default_horizon, starting_learning_rate);

  for (map<Predicate, int, PredicateComp>::iterator it = predicateMap.begin(); it != predicateMap.end(); ++it) {
    ROS_INFO_STREAM(it->first.toString() << " " << it->second << " " << weights[it->second]);
  }

  plan = plans[0];
  getLearnedExplanation();
  plan = plans[1];
  getLearnedExplanation();
  plan = plans[2];
  getLearnedExplanation();
  plan = plans[3];
  getLearnedExplanation();
  plan = plans[4];
  getLearnedExplanation();
  plan = plans[5];
  getLearnedExplanation();
  
}

string location2Text(const string& variable) {
  string text = variable;
  if (text[0] == 'l') {
    text = text.substr(1);
    replace(text.begin(), text.end(), '_', '.');
  }
  return text;
}

string person2Text(const string& variable) {
  string text = variable;
  text[0] = toupper(text[0]);
  return text;
}

string door2Text(const string& variable) {
  string text = variable;
  if (text[0] == 'd') {
    text = variable.substr(1, 6);
    replace(text.begin(), text.end(), '_', '.');
  }
  return "the door connecting to " + text;
}

string object2Text(const string& variable) {
  string text = variable;
  if (text[0] == 'o') {
    text = variable.substr(1);
    replace(text.begin(), text.end(), '_', '.');
  }
  return text;
}

string fluent2Text(const AspFluent& fluent, bool isTrue) {
  if (fluent.getName() == "at") 
    if (isTrue) {return "go to " + location2Text(fluent.getParameters()[0]);}
    else {return "leave " + location2Text(fluent.getParameters()[0]);}

  if (fluent.getName() == "facing") {
    string parameter = fluent.getParameters()[0];

    if (parameter[0] == 'd') {
      parameter = door2Text(parameter);
      if (isTrue) {return "turn to " + parameter;}
      else {return "go through " + parameter;}
    }
    else{
      parameter = object2Text(parameter);
      if (isTrue) {return "turn to " + parameter;}
      else {return "turn away from " + parameter;}
    }
  }

  if (fluent.getName() == "beside") {
    string parameter = fluent.getParameters()[0];
    if (parameter[0] == 'd') {parameter = door2Text(parameter);}
    else {parameter = object2Text(parameter);}

    if (isTrue) {return "approach " + parameter;}
    else {return "get away from " + parameter;}
  }

  if (fluent.getName() == "open") 
    if (isTrue) {return "open " + door2Text(fluent.getParameters()[0]);}
    else {return "check that " + door2Text(fluent.getParameters()[0]) + " is closed";}

  if (fluent.getName() == "accessgranted") 
    if (isTrue) {return "check that I'm allowed to go through " + door2Text(fluent.getParameters()[0]);}
    else {return "know that I'm not allowed to go through " + door2Text(fluent.getParameters()[0]);}

  if (fluent.getName() == "found") 
    if (isTrue) {return "find " + person2Text(fluent.getParameters()[0]);}
    else {return "not know the location of " + person2Text(fluent.getParameters()[0]);}

  if (fluent.getName() == "inroom") 
    if (isTrue) {return "check that " + person2Text(fluent.getParameters()[0]) + " is in room " + location2Text(fluent.getParameters()[1]);}
    else {return "check that " + person2Text(fluent.getParameters()[0]) + " is not in room " + location2Text(fluent.getParameters()[1]);}

  if (fluent.getName() == "messagedelivered") 
    if (isTrue) {return "deliver a message to " + person2Text(fluent.getParameters()[0]);}
    else {return "not deliver a message to " + person2Text(fluent.getParameters()[0]);}

  return "undefined predicate " + fluent.getName();
}
*/

// generalize container
std::string EntityGeneralizer::operator()(const std::string& variable){
  
  //if entity is found
  if (entityMap.find(variable) != entityMap.end()) {
    return entityMap.at(variable);
  }

  if (variable[0] == 'l') {
    std::stringstream ss;
    ss << 'l' << room_count++;
    entityMap[variable] = ss.str();
    return ss.str();
  }

  if (variable[0] == 'd') {
    std::stringstream ss;
    ss << 'd' << door_count++;
    entityMap[variable] = ss.str();
    return ss.str();
  }

  if (variable[0] == 'o') {
    std::stringstream ss;
    ss << 'o' << object_count++;
    entityMap[variable] = ss.str();
    return ss.str();
  }

  if (variable[0] == 'm') {
    std::stringstream ss;
    ss << 'm' << message_count++;
    entityMap[variable] = ss.str();
    return ss.str();
  }

  if (variable.substr(0, 10) == "checkpoint") {
    std::stringstream ss;
    ss << 'o' << object_count++;
    entityMap[variable] = ss.str();
    return ss.str();
  }

  //TODO: extend message server to entity tracker service to remove this assumption
  //TODO: standardize person representation
  std::stringstream ss;
  ss << 'p' << person_count++;
  entityMap[variable] = ss.str();
  return ss.str();

}

}