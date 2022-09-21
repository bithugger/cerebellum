/******************************************************************************
*                    _         _ _            
*     __ ___ _ _ ___| |__  ___| | |_  _ _ __  
*    / _/ -_) '_/ -_) '_ \/ -_) | | || | '  \ 
*    \__\___|_| \___|_.__/\___|_|_|\_,_|_|_|_|
*
*    A Universal Discrete Event Control System Library
*    Copyright (C) 2020  Xingbo Wang
*
*    This program is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*
******************************************************************************/

#ifndef _CEREBELLUM_
#define _CEREBELLUM_

#include <string>
#include <initializer_list>
#include <vector>
#include <list>
#include <set>
#include <map>
#include <memory>
#include <limits>
#include <iostream>
#include <functional>

namespace cerebellum {

class AtomicState;
typedef std::shared_ptr<AtomicState> astate_p;

class Transition;
typedef std::shared_ptr<Transition> transition_p;

//-----------------------------------------------------------------------------
// Atomic State
//-----------------------------------------------------------------------------

enum AtomicStateType {
	PURE = 0,
	INT_DATA,
	MAX
};

class AtomicState {

public:

	static astate_p create(std::string);

	virtual ~AtomicState() = default;

	virtual AtomicStateType type() const;

	const std::string label;
	const std::string output;

	friend std::ostream& operator<<(std::ostream& os, const AtomicState& s);

protected:

	AtomicState(std::string, std::string);

};

std::ostream& operator<<(std::ostream& os, const AtomicState& a);

//-----------------------------------------------------------------------------
// State
//-----------------------------------------------------------------------------

class State {

public:

	State();
	State(astate_p);
	State(std::vector<astate_p>);
	State(std::initializer_list<astate_p>);

	virtual ~State() = default;

	bool empty() const;

	int dimension() const;

	bool contains(const astate_p) const;
	bool contains(std::vector<astate_p>) const;
	bool contains(const State&) const;

	bool operator==(const State&) const;
	bool operator!=(const State&) const;

	bool operator<(const State&) const;

	bool accepts(const transition_p) const;

	State move(const transition_p) const;
	State move_back(const transition_p) const;

	void extend(astate_p);
	void extend(std::vector<astate_p>);
	void extend(std::initializer_list<astate_p>);

	friend std::ostream& operator<<(std::ostream& os, const State& s);

protected:

	std::vector<astate_p> components;
	friend class Transition;
	friend class StateModel;

};

std::ostream& operator<<(std::ostream& os, const State& s);

//-----------------------------------------------------------------------------
// Transition
//-----------------------------------------------------------------------------

class Transition {

public:

	static transition_p create_natural(std::string label, const astate_p from, 
			const astate_p to, double probability = 1, double cost = 1, int priority = 0);

	static transition_p create_controlled(std::string label, const astate_p from, 
			const astate_p to, double cost = 1, double probability = 1, int priority = 0);

	static transition_p create_natural(std::string label, const State& from, 
			const State& to, double probability = 1, double cost = 1, int priority = 0);

	static transition_p create_controlled(std::string label, const State& from, 
			const State& to, double cost = 1, double probability = 1, int priority = 0);

	virtual ~Transition() = default;

	bool available_at(const State&) const;

	void activate(const State&);
	void activate(const astate_p);
	void activate(std::initializer_list<astate_p>);

	void inhibit(const State&);
	void inhibit(const astate_p);
	void inhibit(std::initializer_list<astate_p>);

	void set_probability_at(const State&, double);
	void set_probability_at(const astate_p, double);
	void set_probability_at(std::initializer_list<astate_p>, double);

	void set_cost_at(const State&, double);
	void set_cost_at(const astate_p, double);
	void set_cost_at(std::initializer_list<astate_p>, double);

	double cost(const State&) const;
	double probability(const State&) const;

	const std::string label;

	const State from;
	const State to;

	const bool controllable;
	const double base_cost;
	const double base_probability;
	const int priority;

	/* pointer to a utility transition that represents the failure to complete */
	transition_p fail;

protected:

	std::set<State> conditions;
	bool active_conditions;

	std::list<std::pair<State, double>> conditional_costs;
	std::list<std::pair<State, double>> conditional_probabilities;

	Transition(std::string label, const State& from, const State& to, 
				bool controllable, double cost, double probability, 
				int level, bool active_conditions);

	friend class StateModel;
};

//-----------------------------------------------------------------------------
// Path
//-----------------------------------------------------------------------------

class Path {

public:

	Path(State);
	Path(const Path& b);
	virtual ~Path() = default;

	std::list<std::string> inputs;
	std::list<transition_p> transitions;
	std::list<State> states;

	double cost;
	double probability;

	bool operator<(const Path& b) const;

	void operator<<=(const Path& b);
	void operator<<=(transition_p t);
	void operator>>=(const Path& b);
	void operator>>=(transition_p t);

	Path operator<<(const Path& b) const;
	Path operator<<(transition_p t) const;
	Path operator>>(const Path& b) const;
	Path operator>>(transition_p t) const;

	Path& operator=(const Path& b) = default;
};

//-----------------------------------------------------------------------------

class PathWay : public Path{

public:

	PathWay(State);
	virtual ~PathWay() = default;

	std::list<std::vector<Path> > cycles;

	double probability_arrival_within_step(unsigned int n);
	double probability_arrival_eventually(double tolerance = 1e-6);

protected:

	double probability_arrival_at_step(unsigned int n, std::map<unsigned int, double>& memo);

};

//-----------------------------------------------------------------------------
// State Model
//-----------------------------------------------------------------------------

class StateModel {

public:

	StateModel(std::vector<transition_p> transitions, bool allow_wait = true);
	StateModel(std::initializer_list<transition_p> transitions, bool allow_wait = true);
	StateModel(const StateModel&);
	virtual ~StateModel() = default;

	const std::set<transition_p> transitions;

	/** Output a string containing text input to GraphViz for visualization **/
	std::string as_dot_file() const;

	/** Find a single cost-optimal path using Djikstra's algorithm **/
	Path find_path(const State from, const State to);

	Path find_path_around(const State from, const State to, const State avoid);

	Path find_path_around(const State from, const State to, std::vector<State> avoid);

	Path find_path_around(const State from, const State to, std::initializer_list<State> avoid);

	/** Find a single cost/probability optimal path, with probability/cost restrictions, using B3 search **/
	Path find_best_path_over_likelihood(const State from, const State to, double prob_limit);

	Path find_best_path_around_over_likelihood(const State from, const State to, 
				const State avoid, double prob_limit);

	Path find_best_path_around_over_likelihood(const State from, const State to, 
				std::vector<State> avoid, double prob_limit);

	Path find_best_path_around_over_likelihood(const State from, const State to, 
				std::initializer_list<State> avoid, double prob_limit);
	
	Path find_likeliest_path_under_cost(const State from, const State to, double cost_limit);

	Path find_likeliest_path_around_under_cost(const State from, const State to, 
				const State avoid, double cost_limit);

	Path find_likeliest_path_around_under_cost(const State from, const State to, 
				std::vector<State> avoid, double cost_limit);

	Path find_likeliest_path_around_under_cost(const State from, const State to, 
				std::initializer_list<State> avoid, double cost_limit);

	/** Find all simple paths (no cycles) using DFS **/
	std::vector<Path> find_all_paths(const State from, const State to);

	std::vector<Path> find_all_paths_around(const State from, const State to, const State avoid);

	std::vector<Path> find_all_paths_around(const State from, const State to, std::vector<State> avoid);

	std::vector<Path> find_all_paths_back(const State x);

	std::vector<Path> find_all_paths_back_around(const State x, const State avoid);

	std::vector<Path> find_all_paths_back_around(const State x, std::vector<State> avoid);

	/** Find all pathways (simple and with cycles) using two-pass DFS **/
	std::vector<PathWay> find_all_pathways(const State from, const State to);

	std::vector<PathWay> find_all_pathways_around(const State from, const State to, const State avoid);

	std::vector<PathWay> find_all_pathways_around(const State from, const State to, std::vector<State> avoid);

protected:

	bool allow_wait;

	/** path finding algorithms **/
	Path path_find_djikstra(const State from, const State to, std::vector<State> avoid);

	/* B3 (best-first branch and bound) path finding for lowest cost with privilege and total probability/cost limits */
	Path path_find_b3(const State from, const State to, std::vector<State> avoid, 
				double probability_limit, double cost_limit);

	/* find all paths from one state to another given obstacles, this is memory and computationally intensive */
	std::vector<Path> path_find_dfs(const State from, const State to, std::vector<State> avoid, bool back);

	std::vector<Path> _recursive_dfs(const State from, const State to, std::vector<State> avoid, 
				std::set<State> visited, Path path_so_far, bool back);

	/* find all pathways from one state to another given obstacles, this is memory and computationally intensive */
	std::vector<PathWay> pathway_find_dfs(const State from, const State to, std::vector<State> avoid);

	/** returns all enabled transitions from a given state **/
	std::list<transition_p> all_transitions_from(const State from);

	std::list<transition_p> natural_transitions_from(const State from);

	std::map<std::string, std::list<transition_p>> controlled_transitions_from(const State from);

	/** some natural transitions can fail, leading to alternatives in terms of nonconflicting
	 * pathways. this function computes all potential paths */
	std::set<std::list<transition_p>> all_potential_natural_transitions_from(const State from);

	/** some controlled transitions can fail, leading to alternatives in terms of nonconflicting
	 * pathways. this function computes all potential paths for each control input */
	std::map<std::string, std::set<std::list<transition_p>>> all_potential_controlled_transitions_from(const State from);

	/* follow a series of natural transitions from state x until a conflict arises or all transitions are used */
	std::set<std::list<transition_p>> _recursive_expand_possible_transitions(const State x, std::list<transition_p> t_to_go);

};

//-----------------------------------------------------------------------------
// State Machine
//-----------------------------------------------------------------------------

class StateMachine : public StateModel {

public:

	StateMachine(const StateModel& model, State initial);
	virtual ~StateMachine() = default;

	std::list<State> move(std::string);

	State get_state();

	void on_enter(const State s, std::function<void(void)> cb);
	void on_remain(const State s, std::function<void(void)> cb);
	void on_exit(const State s, std::function<void(void)> cb);

	Path find_path(const State to);

protected:

	State current_state;
	State previous_state;

	std::map< State, std::vector<std::function<void(void)>> > enter_callbacks;
	std::map< State, std::vector<std::function<void(void)>> > remain_callbacks;
	std::map< State, std::vector<std::function<void(void)>> > exit_callbacks;

};

//-----------------------------------------------------------------------------
// Data State
//-----------------------------------------------------------------------------

class DataState;
typedef std::shared_ptr<DataState> dstate_p;
class DataModel;
typedef std::shared_ptr<DataModel> DataSource;

class DataState : public AtomicState {

public:

	const DataSource source;
	const enum Qualifier {
		EQUALS = 0,
		NOT_EQUALS,
		LESS_THAN,
		LESS_THAN_EQUALS,
		GREATER_THAN,
		GREATER_THAN_EQUALS,
		NONE
	} qualifier;
	const int value;

	bool is_subset(const DataState&) const;
	bool is_subset(const dstate_p) const;

	AtomicStateType type() const override;

protected:

	DataState(const DataSource, Qualifier, int);
	friend class DataModel;

};

//-----------------------------------------------------------------------------
// Data Model
//-----------------------------------------------------------------------------

class DataModel : public std::enable_shared_from_this<DataModel> {

public:

	static DataSource create_source(std::string);
	static DataSource create_source(std::string, int ub, int lb);
	
	const std::string name;
	const int lower_bound;
	const int upper_bound;

	/* states */
	dstate_p value_at(int);

	/* conditions */
	dstate_p value_eq(int);
	dstate_p value_lt(int);
	dstate_p value_gt(int);
	dstate_p value_neq(int);
	dstate_p value_leq(int);
	dstate_p value_geq(int);

	/* transitions */
	dstate_p value_any();
	dstate_p value_change(int);

protected:

	DataModel(std::string, int, int);

};

bool operator==(const astate_p& x, const astate_p& y);
bool operator<(const astate_p& x, const astate_p& y);

} // namespace cerebellum

#endif // #ifndef _CEREBELLUM_