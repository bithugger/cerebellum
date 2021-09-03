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
#include <utility>
#include <algorithm>
#include <iostream>
#include <typeinfo>
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

	friend std::ostream& operator<<(std::ostream& os, const AtomicState&);

protected:

	AtomicState(std::string, std::string);

};

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

	friend std::ostream& operator<<(std::ostream& os, const State&);

protected:

	std::vector<astate_p> components;
	friend class Transition;

};

//-----------------------------------------------------------------------------
// Transition
//-----------------------------------------------------------------------------

class Transition {

public:

	static transition_p create_natural(std::string label, const astate_p from, 
			const astate_p to, double probability = 1, double cost = 1, unsigned int priority = 0);

	static transition_p create_controlled(std::string label, const astate_p from, 
			const astate_p to, double cost = 1, unsigned int restriction = 0, double probability = 1);

	static transition_p create_external(std::string label, const astate_p from, 
			const astate_p to, double cost = 1, double probability = 1);

	static transition_p create_natural(std::string label, const State& from, 
			const State& to, double probability = 1, double cost = 1, unsigned int priority = 0);

	static transition_p create_controlled(std::string label, const State& from, 
			const State& to, double cost = 1, unsigned int restriction = 0, double probability = 1);

	static transition_p create_external(std::string label, const State& from, 
			const State& to, double cost = 1, double probability = 1);

	virtual ~Transition() = default;

	bool available_at(const State&) const;

	void activate(const State&);
	void activate(const astate_p);
	void activate(std::initializer_list<astate_p>);

	void inhibit(const State&);
	void inhibit(const astate_p);
	void inhibit(std::initializer_list<astate_p>);

	const std::string label;

	const State from;
	const State to;

	const enum transition_type {
		NATURAL,
		CONTROLLED,
		EXTERNAL
	} type;
	
	const double cost;
	const double probability;
	const unsigned int level;

	/* pointer to a utility transition that represents the failure to complete */
	transition_p fail;

protected:

	std::vector<std::set<State>> conditions;
	bool active_conditions;

	Transition(std::string label, const State& from, const State& to, 
				transition_type type, double cost, double probability, 
				unsigned int level, bool active_conditions);

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
	unsigned int level;

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

	typedef std::pair<std::string, std::set<transition_p>> transition_bundle;

public:

	StateModel(std::vector<transition_p> transitions, bool allow_wait = true);
	StateModel(std::initializer_list<transition_p> transitions, bool allow_wait = true);
	StateModel(const StateModel&);
	virtual ~StateModel() = default;

	const std::set<transition_p> transitions;

	/** Find single optimal path using Djikstra's algorithm **/
	Path find_path(const State from, const State to, 
				unsigned int restriction = std::numeric_limits<unsigned int>::max());

	Path find_path_around(const State from, const State to, const State avoid,
				unsigned int restriction = std::numeric_limits<unsigned int>::max());

	Path find_path_around(const State from, const State to, std::vector<State> avoid,
				unsigned int restriction = std::numeric_limits<unsigned int>::max());

	Path find_path_around(const State from, const State to, std::initializer_list<State> avoid,
				unsigned int restriction = std::numeric_limits<unsigned int>::max());

	/** Find all simple paths using DFS **/
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

	/** returns all enabled transitions from a given state **/
	std::map<std::string, std::set<transition_p>> all_transitions_from(const State from);

	std::map<std::string, std::set<transition_p>> natural_transitions_from(const State from);

	std::map<std::string, std::set<transition_p>> controlled_transitions_from(const State from,
				unsigned int restriction = std::numeric_limits<unsigned int>::max());

	std::map<std::string, std::set<transition_p>> external_transitions_from(const State from);

	std::set<transition_p> prioritized_natural_transitions_from(const State from);

	/** returns all transitions to a given state from a state where it is enabled **/
	std::map<std::string, std::set<transition_p>> all_transitions_to(const State to);

	std::map<std::string, std::set<transition_p>> natural_transitions_to(const State to);

	std::map<std::string, std::set<transition_p>> controlled_transitions_to(const State to,
				unsigned int restriction = std::numeric_limits<unsigned int>::max());

	std::map<std::string, std::set<transition_p>> external_transitions_to(const State to);

	/** path finding algorithms **/
	Path path_find_djikstra(const State from, const State to, 
				std::vector<State> avoid, unsigned int restriction);

	std::vector<Path> path_find_dfs(const State from, const State to, std::vector<State> avoid, bool back);

	std::vector<Path> _recursive_dfs(const State from, const State to, std::vector<State> avoid, 
				std::set<State> visited, Path path_so_far, bool back);

	std::vector<PathWay> pathway_find_dfs(const State from, const State to, std::vector<State> avoid);

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

	Path find_path(const State to, 
				unsigned int restriction = std::numeric_limits<unsigned int>::max());

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

inline bool operator==(const astate_p& x, const astate_p& y);
inline bool operator<(const astate_p& x, const astate_p& y);

//=============================================================================
// Implementation
//=============================================================================

AtomicState::AtomicState(std::string label, std::string output) :
label(label),
output(output)
{

}

astate_p AtomicState::create(std::string s){
	return astate_p(new AtomicState(s, s));
}

AtomicStateType AtomicState::type() const {
	return AtomicStateType::PURE;
}

std::ostream& operator<<(std::ostream& os, const AtomicState& a){
	return os << a.label;
}

//-----------------------------------------------------------------------------

State::State() :
components()
{

}

State::State(astate_p as) :
components()
{
	components.push_back(as);
}

State::State(std::vector<astate_p> components) :
components(components)
{

}

State::State(std::initializer_list<astate_p> components) :
components(components)
{

}

bool State::empty() const {
	return components.empty();
}

int State::dimension() const {
	return components.size();
}

bool State::contains(const astate_p x) const {
	if(x->type() == AtomicStateType::INT_DATA){
		return std::find_if(components.begin(), components.end(), [&](const astate_p y){
			if(y->type() == AtomicStateType::INT_DATA){
				DataState& xx = static_cast<DataState&>(*x);
				DataState& yy = static_cast<DataState&>(*y); 
				return yy.is_subset(xx);
			}else{
				return false;
			}
		}) != components.end();
	}else{
		return std::find(components.begin(), components.end(), x) != components.end();		
	}
}

bool State::contains(std::vector<astate_p> X) const {
	bool yes = true;
	for(astate_p x : X){
		yes &= contains(x);
	}
	return yes;
}

bool State::contains(const State& s) const {
	return contains(s.components);
}

bool State::operator==(const State& b) const {
	return this->contains(b.components) && b.contains(this->components);
}

bool State::operator!=(const State& b) const {
	return !(operator==(b));
}

bool State::operator<(const State& o) const {
	if(dimension() != o.dimension()){
		return dimension() < o.dimension();
	}else{
		std::vector<astate_p> a = components;
		std::vector<astate_p> b = o.components;
		std::sort(a.begin(), a.end());
		std::sort(b.begin(), b.end());
		for(int i = 0; i < dimension(); i++){
			if(a[i] != b[i]){
				return a[i] < b[i];
			}
		}
		return false;
	}
}

bool State::accepts(const transition_p t) const {
	return contains(t->from) && t->available_at(*this);
}

State State::move(const transition_p t) const {
	if(contains(t->from)){
		std::vector<std::pair<astate_p, astate_p>> changes;
		for(int i = 0; i < t->from.dimension(); i++){
			changes.push_back(std::make_pair(t->from.components[i], t->to.components[i]));
		}

		std::vector<astate_p> new_components = components;
		for(auto ft : changes){
			if(ft.first->type() == AtomicStateType::INT_DATA){
				DataState& xx = static_cast<DataState&>(*ft.first);
				auto it = std::find_if(new_components.begin(), new_components.end(), [&](const astate_p y){
					if(y->type() == AtomicStateType::INT_DATA){
						DataState& yy = static_cast<DataState&>(*y); 
						return yy.is_subset(xx);
					}else{
						return false;
					}
				});
				DataState& yy = static_cast<DataState&>(*(*it));
				DataState& zz = static_cast<DataState&>(*ft.second);
				if(zz.qualifier == DataState::NONE){
					*it = yy.source->value_at(yy.value + zz.value);
				}else if(zz.qualifier == DataState::EQUALS){
					*it = yy.source->value_at(zz.value);
				}
			}else{
				auto it = std::find(new_components.begin(), new_components.end(), ft.first);
				*it = ft.second;	
			}
		}
		return State(new_components);
	}else{
		return State();
	}
}

State State::move_back(const transition_p t) const {
	if(contains(t->to)){
		std::vector<std::pair<astate_p, astate_p>> changes;
		for(int i = 0; i < t->from.dimension(); i++){
			changes.push_back(std::make_pair(t->from.components[i], t->to.components[i]));
		}

		std::vector<astate_p> new_components = components;
		for(auto ft : changes){
			if(ft.first->type() == AtomicStateType::INT_DATA){
				DataState& xx = static_cast<DataState&>(*ft.first);
				auto it = std::find_if(new_components.begin(), new_components.end(), [&](const astate_p y){
					if(y->type() == AtomicStateType::INT_DATA){
						DataState& yy = static_cast<DataState&>(*y); 
						return yy.is_subset(xx);
					}else{
						return false;
					}
				});
				DataState& yy = static_cast<DataState&>(*(*it));
				DataState& zz = static_cast<DataState&>(*ft.second);
				if(zz.qualifier == DataState::NONE){
					*it = yy.source->value_at(yy.value - zz.value);
				}else if(zz.qualifier == DataState::EQUALS){
					*it = yy.source->value_at(yy.value);
				}
			}else{
				auto it = std::find(new_components.begin(), new_components.end(), ft.second);
				*it = ft.first;
			}
		}
		return State(new_components);
	}else{
		return State();
	}
}

void State::extend(astate_p a){
	components.push_back(a);
}

void State::extend(std::vector<astate_p> as){
	for(const astate_p a : as){
		extend(a);
	}
}

void State::extend(std::initializer_list<astate_p> as){
	for(const astate_p a : as){
		extend(a);
	}
}

std::ostream& operator<<(std::ostream& os, const State& s){
	os << "(";
	for(size_t i = 0; i < s.components.size(); i++){
		os << *(s.components[i]);
		if(i < s.components.size() - 1){
			os << ", ";
		}
	}
	return os << ")";
}

//-----------------------------------------------------------------------------

Transition::Transition(std::string label, const State& from, const State& to, 
				transition_type type, double cost, double probability, 
				unsigned int level, bool active_conditions) :
label(label),
from(from),
to(to),
type(type),
cost(cost),
probability(probability),
level(level),
conditions(),
active_conditions(active_conditions),
fail(nullptr)
{

}

transition_p Transition::create_natural(std::string label, const astate_p from, 
				const astate_p to, double probability, double cost, unsigned int priority){
	transition_p tp(new Transition(label, State({from}), State({to}), 
						NATURAL, cost, probability, priority, true));
	transition_p tpf(new Transition("!" + label, State({from}), State({from}),
						NATURAL, 0, 1.0 - probability, priority, true));
	tp->fail = tpf;
	return tp;
}

transition_p Transition::create_controlled(std::string label, const astate_p from,
				const astate_p to, double cost, unsigned int restriction, double probability){
	transition_p tp(new Transition(label, State({from}), State({to}), 
						CONTROLLED, cost, probability, restriction, false));
	transition_p tpf(new Transition("!" + label, State({from}), State({from}),
						CONTROLLED, 0, 1.0 - probability, restriction, false));
	tp->fail = tpf;
	return tp;
}

transition_p Transition::create_external(std::string label, const astate_p from, 
				const astate_p to, double cost, double probability){
	transition_p tp(new Transition(label, State({from}), State({to}), 
						EXTERNAL, cost, probability, 0, false));
	transition_p tpf(new Transition("!" + label, State({from}), State({from}),
						EXTERNAL, 0, 1.0 - probability, 0, false));
	tp->fail = tpf;
	return tp;
}

transition_p Transition::create_natural(std::string label, const State& from, 
				const State& to, double probability, double cost, unsigned int priority){
	transition_p tp(new Transition(label, from, to, 
						NATURAL, cost, probability, priority, true));
	transition_p tpf(new Transition("!" + label, from, from,
						NATURAL, 0, 1.0 - probability, priority, true));
	tp->fail = tpf;
	return tp;
}

transition_p Transition::create_controlled(std::string label, const State& from,
				const State& to, double cost, unsigned int restriction, double probability){
	transition_p tp(new Transition(label, from, to, 
						CONTROLLED, cost, probability, restriction, false));
	transition_p tpf(new Transition("!" + label, from, from,
						CONTROLLED, 0, 1.0 - probability, restriction, false));
	tp->fail = tpf;
	return tp;
}

transition_p Transition::create_external(std::string label, const State& from, 
				const State& to, double cost, double probability){
	transition_p tp(new Transition(label, from, to, 
						EXTERNAL, cost, probability, 0, false));
	transition_p tpf(new Transition("!" + label, from, from,
						EXTERNAL, 0, 1.0 - probability, 0, false));
	tp->fail = tpf;
	return tp;
}

bool Transition::available_at(const State& x) const {
	/* check transition conditions */
	bool pass = !active_conditions;
	for(const std::set<State> s : conditions){
		bool sat = true;
		for(const State& c : s){
			sat &= x.contains(c);
		}

		if(active_conditions){
			pass |= sat;
		}else{
			pass &= !sat;
		}
	}

	/* check data state saturation */
	std::vector<std::pair<astate_p, astate_p>> changes;
	for(int i = 0; i < from.dimension(); i++){
		changes.push_back(std::make_pair(from.components[i], to.components[i]));
	}

	for(std::pair<astate_p, astate_p> change : changes){
		const astate_p f = change.first;
		const astate_p t = change.second;

		if(f->type() == AtomicStateType::INT_DATA){
			DataState& ff = static_cast<DataState&>(*f);
			for(const astate_p a : x.components){
				if(a->type() == AtomicStateType::INT_DATA){
					DataState& aa = static_cast<DataState&>(*a);
					if(aa.is_subset(ff)){
						DataState& bb = static_cast<DataState&>(*t);
						int v = 0;
						if(bb.qualifier == DataState::NONE){
							v = aa.value + bb.value;
						}else if(bb.qualifier == DataState::EQUALS){
							v = bb.value;
						}
						pass &= (v <= ff.source->upper_bound);
						pass &= (v >= ff.source->lower_bound);
					}
				}
			}
		}
	}

	return pass;
}

void Transition::activate(const State& s){
	if(!active_conditions){
		active_conditions = true;
		conditions.clear();
	}
	std::set<State> S;
	S.insert(s);
	conditions.push_back(S);
	if(fail){
		fail->activate(s);
	}
}

void Transition::activate(const astate_p as){
	if(!active_conditions){
		active_conditions = true;
		conditions.clear();
	}
	std::set<State> S;
	S.insert(State(as));
	conditions.push_back(S);
	if(fail){
		fail->activate(as);
	}
}

void Transition::activate(std::initializer_list<astate_p> as){
	if(!active_conditions){
		active_conditions = true;
		conditions.clear();
	}
	std::set<State> S;
	S.insert(State(as));
	conditions.push_back(S);
	if(fail){
		fail->activate(as);
	}
}

void Transition::inhibit(const State& s){
	if(active_conditions){
		active_conditions = false;
		conditions.clear();
	}
	std::set<State> S;
	S.insert(s);
	conditions.push_back(S);
	if(fail){
		fail->inhibit(s);
	}
}

void Transition::inhibit(const astate_p as){
	if(active_conditions){
		active_conditions = false;
		conditions.clear();
	}
	std::set<State> S;
	S.insert(State(as));
	conditions.push_back(S);
	if(fail){
		fail->inhibit(as);
	}
}

void Transition::inhibit(std::initializer_list<astate_p> as){
	if(active_conditions){
		active_conditions = false;
		conditions.clear();
	}
	std::set<State> S;
	S.insert(State(as));
	conditions.push_back(S);
	if(fail){
		fail->inhibit(as);
	}
}

//-----------------------------------------------------------------------------

Path::Path(State start) :
inputs(),
transitions(),
states(),
cost(0),
probability(1),
level(0)
{
	states.push_back(start);
}

Path::Path(const Path& b) :
inputs(b.inputs),
transitions(b.transitions),
states(b.states),
cost(b.cost),
probability(b.probability),
level(b.level)
{

}

bool Path::operator<(const Path& b) const {
	// order by level, cost, probability, input size, transitions size, state size
	if(level == b.level){
		if(cost == b.cost){
			if(probability == b.probability){
				if(inputs.size() == b.inputs.size()){
					if(transitions.size() == b.transitions.size()){
						return states.size() < b.states.size();
					}else{
						return transitions.size() < b.transitions.size();
					}
				}else{
					return inputs.size() < b.inputs.size();
				}
			}else{
				return probability > b.probability;
			}
		}else{
			return cost < b.cost;
		}
	}else{
		return level < b.level;
	}
}


void Path::operator<<=(const Path& b){
	for(transition_p t : b.transitions){
		operator<<=(t);
	}
}

void Path::operator>>=(const Path& b){
	for(auto it = b.transitions.rbegin(); it != b.transitions.rend(); **it){
		operator>>=(*it);
	}
}

void Path::operator<<=(transition_p t){
	cost += t->cost;
	probability *= t->probability;
	if(t->level > level){
		level = t->level;
	}

	if(t->type == Transition::CONTROLLED){
		inputs.push_back(t->label);
	}
	transitions.push_back(t);
	states.push_back(states.back().move(t));
}

void Path::operator>>=(transition_p t){
	cost += t->cost;
	probability *= t->probability;
	if(t->level > level){
		level = t->level;
	}

	if(t->type == Transition::CONTROLLED){
		inputs.push_front(t->label);
	}
	transitions.push_front(t);
	states.push_front(t->to);
}


Path Path::operator<<(const Path& b) const {
	Path c(*this);
	c <<= b;
	return c;
}

Path Path::operator<<(transition_p t) const {
	Path c(*this);
	c <<= t;
	return c;
}

Path Path::operator>>(const Path& b) const {
	Path c(*this);
	c >>= b;
	return c;
}

Path Path::operator>>(transition_p t) const {
	Path c(*this);
	c >>= t;
	return c;
}

//-----------------------------------------------------------------------------

PathWay::PathWay(State start) :
Path(start),
cycles()
{

}

double PathWay::probability_arrival_within_step(unsigned int n){
	double total_probability = 0;
	std::map<unsigned int, double> memo;

	for(int i = inputs.size(); i <= n; i++){
		double inst_probability = probability_arrival_at_step(i, memo);
		total_probability += inst_probability;
		if(total_probability > 1){
			total_probability = 1;
		}
	}

	return total_probability;
}

double PathWay::probability_arrival_eventually(double tolerance){
	double total_probability = 0;
	double diff = 1;
	unsigned int i = inputs.size();

	/* find the size of the largest cycle, which is the number of consecutive turns the 
	 * differential probability needs to stay below the tolerance for convergence */
	unsigned int M = 0;
	for(std::vector<Path> cycles_ : cycles){
		for(Path cycle : cycles_){
			if(cycle.inputs.size() > M){
				M = cycle.inputs.size();
			}
		}
	}
	unsigned int m = 0;

	std::map<unsigned int, double> memo;
	do{
		double inst_probability = probability_arrival_at_step(i, memo);
		double next_total_probability = total_probability + inst_probability;
		if(next_total_probability > 1){
			next_total_probability = 1;
		}
		diff = next_total_probability - total_probability;
		total_probability = next_total_probability;

		if(diff < tolerance){
			m++;
		}else{
			m = 0;
		}
	}while(m < M && i++ < std::numeric_limits<unsigned int>::max());
	return total_probability;
}

double PathWay::probability_arrival_at_step(unsigned int n, std::map<unsigned int, double>& memo){
	if(n < inputs.size()){
		return 0;
	}else if(n == inputs.size()){
		return probability;
	}

	/* n > inputs.size() */
	if(memo.find(n) == memo.end()){
		double x = 0;
		for(std::vector<Path> cycles_now : cycles){
			for(Path c : cycles_now){
				unsigned int k = c.inputs.size();
				double sub_prob = probability_arrival_at_step(n - k, memo);
				double y = c.probability*sub_prob;
				if(y > x){
					x = y;
				}
			}
		}
		memo.insert({n, x});
	}
	return memo[n];
}

//-----------------------------------------------------------------------------

StateModel::StateModel(std::vector<transition_p> transitions, bool wait) :
transitions(transitions.begin(), transitions.end()),
allow_wait(wait)
{

}

StateModel::StateModel(std::initializer_list<transition_p> transitions, bool wait) :
transitions(transitions.begin(), transitions.end()),
allow_wait(wait)
{

}

StateModel::StateModel(const StateModel& b) :
transitions(b.transitions),
allow_wait(b.allow_wait)
{

}

Path StateModel::find_path_around(const State from, const State to, 
											std::vector<State> avoid, unsigned int restriction){
	return path_find_djikstra(from, to, avoid, restriction);
}

Path StateModel::find_path_around(const State from, const State to, 
											const State avoid, unsigned int restriction){
	std::vector<State> avoids;
	avoids.push_back(avoid);
	return path_find_djikstra(from, to, avoids, restriction);
}

Path StateModel::find_path_around(const State from, const State to, 
											std::initializer_list<State> avoid, unsigned int restriction){
	std::vector<State> avoids(avoid.begin(), avoid.end());
	return path_find_djikstra(from, to, avoids, restriction);
}

Path StateModel::find_path(const State from, const State to, unsigned int restriction){
	std::vector<State> avoids;
	return path_find_djikstra(from, to, avoids, restriction);
}

std::vector<Path> StateModel::find_all_paths(const State from, const State to){
	std::vector<State> avoids;
	return find_all_paths_around(from, to, avoids);
}

std::vector<Path> StateModel::find_all_paths_around(const State from, const State to, const State avoid){
	std::vector<State> avoids;
	avoids.push_back(avoid);
	return find_all_paths_around(from, to, avoids);
}

std::vector<Path> StateModel::find_all_paths_around(const State from, const State to, std::vector<State> avoid){
	return path_find_dfs(from, to, avoid, false);
}


std::vector<Path> StateModel::find_all_paths_back(const State x){
	std::vector<State> avoids;
	return find_all_paths_back_around(x, avoids);
}

std::vector<Path> StateModel::find_all_paths_back_around(const State x, const State avoid){
	std::vector<State> avoids;
	avoids.push_back(avoid);
	return find_all_paths_back_around(x, avoid);
}

std::vector<Path> StateModel::find_all_paths_back_around(const State x, std::vector<State> avoid){
	return path_find_dfs(x, x, avoid, true);
}


std::vector<PathWay> StateModel::find_all_pathways(const State from, const State to){
	std::vector<State> avoids;
	return find_all_pathways_around(from, to, avoids);
}

std::vector<PathWay> StateModel::find_all_pathways_around(const State from, const State to, const State avoid){
	std::vector<State> avoids;
	avoids.push_back(avoid);
	return find_all_pathways_around(from, to, avoids);
}

std::vector<PathWay> StateModel::find_all_pathways_around(const State from, const State to, std::vector<State> avoid){
	return pathway_find_dfs(from, to, avoid);
}

Path StateModel::path_find_djikstra(const State from, const State to, std::vector<State> avoid,
			unsigned int restriction){
	Path path(from);

	if(from.dimension() < to.dimension()){
		return path;
	}else if(to.empty() || from.empty()){
		return path;
	}

	std::set<State> visited_states;
	std::set<State> known_states;
	std::map<State, double> cost_map;

	/* map from each state to the controlled/natural pair of transitions */
	/* the first item in the pair, the transition_bundle type, is controlled */
	/* the second item in the pair, the set of transitions, is natural */
	std::map<State, std::pair<transition_bundle, std::set<transition_p>> > trans_map;

	known_states.insert(from);
	cost_map[from] = 0;
	State u = from;
	State dest = to;

	while(visited_states != known_states){
		/* find unvisited vertex with min cost */
		double cost = std::numeric_limits<double>::max();
		for(auto& kv : cost_map){
			if(visited_states.find(kv.first) == visited_states.end() && kv.second < cost){
				u = kv.first;
				cost = kv.second;
			}
		}

		/* if arrived at destination (set), terminate */
		if(u.contains(to)){
			dest = u;
			break;
		}

		/* mark u as visited */
		visited_states.insert(u);

		/* find available controlled transitions */
		std::map<std::string, std::set<transition_p>> t_controlled = controlled_transitions_from(u, restriction);
		/* see where they go */
		for(const transition_bundle& tb : t_controlled){
			/* determine the cost and destination of this move */
			double tcost = 0;
			State nu = u;
			for(const transition_p t : tb.second){
				nu = nu.move(t);
				tcost += t->cost;
			}

			/* TODO check destination reached, although transient? */
			/* this option should be parametrized */

			/* after arriving, apply all non-conflicting natural transitions */
			std::set<transition_p> t_natural = prioritized_natural_transitions_from(nu);
			for(const transition_p nt : t_natural){
				nu = nu.move(nt);
				tcost += nt->cost;
			}

			/* check if this state needs to be avoided */
			bool to_avoid = false;
			for(State obst : avoid){
				to_avoid |= nu.contains(obst);
			}
			if(!to_avoid){
				/* add the neighbors to the known state set */
				known_states.insert(nu);

				/* see if this is a better path to reach nu */
				double ncost = cost + tcost;
				auto it = cost_map.find(nu);
				if(it == cost_map.end() || (it != cost_map.end() && ncost < it->second)){
					cost_map[nu] = ncost;
					trans_map[nu] = std::make_pair(tb, t_natural);
				}
			}
		}
	}

	/* is the destination reachable? */
	std::list<transition_p> transitions_in_path;
	if(u == from || trans_map.find(dest) != trans_map.end()){
		u = dest;
		while(u != from){
			std::pair<transition_bundle, std::set<transition_p>> ts = trans_map.find(u)->second;
			transition_bundle& tb = ts.first;

			/* apply in reverse order */
			/* first the natural, then the controlled */
			for(const transition_p nt : ts.second){
				u = u.move_back(nt);
				transitions_in_path.push_front(nt);
			}

			if(tb.second.empty()){
				transition_p no_action = Transition::create_controlled("", u, u, 0, 0);
				transitions_in_path.push_front(no_action);
			}else{
				for(const transition_p t : tb.second){
					u = u.move_back(t);
					transitions_in_path.push_front(t);
				}
			}
		}
	}else{
		/* destination not reachable, clear path */
		path.states.clear();
		path.probability = 0;
	}

	for(transition_p t : transitions_in_path){
		path <<= t;
	}

	return path;
}

std::vector<Path> StateModel::path_find_dfs(const State from, const State to, std::vector<State> avoid, bool back){
	std::vector<Path> all_paths;

	if(from.dimension() < to.dimension()){
		return all_paths;
	}else if(to.empty() || from.empty()){
		return all_paths;
	}else{
		std::set<State> visited;
		if(!back){
			visited.insert(from);
		}
		all_paths = _recursive_dfs(from, to, avoid, visited, Path(from), back);

		std::sort(all_paths.begin(), all_paths.end());

		return all_paths;
	}
}


std::vector<Path> StateModel::_recursive_dfs(const State from, const State to, std::vector<State> avoid, 
			std::set<State> visited, Path path_so_far, bool back){

	std::vector<Path> paths;
	if(!back && from.contains(to)){
		// reached the destination
		// return an vector with the current path
		paths.push_back(path_so_far);
		return paths;
	}else{
		/* find available controlled transitions */
		std::map<std::string, std::set<transition_p>> t_controlled = controlled_transitions_from(from);
		/* see where they go */
		for(const transition_bundle& tb : t_controlled){
			Path inst_path_controlled(from);
			State state_after_controlled = from;

			/* determine the cost and destination of this move */
			for(const transition_p t : tb.second){
				state_after_controlled = state_after_controlled.move(t);
				inst_path_controlled <<= t;
			}

			/* TODO check destination reached, although transient? */
			/* this option should be parametrized */

			/* after arriving, apply all non-conflicting natural transitions */
			std::set<transition_p> t_natural = prioritized_natural_transitions_from(state_after_controlled);

			/* if allowed to wait, only do so if there are non-trivial natural transitions afterwards */
			if(tb.second.empty()){
				if(!t_natural.empty()){
					transition_p no_action = Transition::create_controlled("", from, from, 0, 0, 1);
					inst_path_controlled <<= no_action;
				}else{
					continue;
				}
			}

			/* create a powerset of the natural transitions corresponding to each success/failure outcome */
			/* exclude ones with certain outcomes, i.e. probability = 1 */
			std::list<State> next_candidates;
			std::list<Path> inst_path_candidates;
			std::set<transition_p> uncertain_natural_ts;
			for(const transition_p nt : t_natural){
				if(nt->probability == 1){
					/* certain outcome, add it to existing chain */
					state_after_controlled = state_after_controlled.move(nt);
					inst_path_controlled <<= nt;
				}else{
					/* transition could fail, add to uncertain set */
					uncertain_natural_ts.insert(nt);
				}
			}
			if(uncertain_natural_ts.empty()){
				next_candidates.push_back(state_after_controlled);
				inst_path_candidates.push_back(inst_path_controlled);
			}else{
				for(int mask = 0; mask < (1 << uncertain_natural_ts.size()); mask++){
					auto it = uncertain_natural_ts.begin();
					for(int i = 0; i < uncertain_natural_ts.size(); i++){
						transition_p nt;
						if(((mask >> i) & 1) == 0){
							/* not fail, success */
							nt = *it;
						}else{
							/* not success, fail */
							nt = (*it)->fail;
						}
						next_candidates.push_back(state_after_controlled.move(nt));
						inst_path_candidates.push_back(inst_path_controlled << nt);
						++it;
					}
				}
			}

			auto next_state_it = next_candidates.begin();
			auto inst_path_it = inst_path_candidates.begin();
			while(next_state_it != next_candidates.end() && inst_path_it != inst_path_candidates.end()){
				State next = *next_state_it;
				Path inst_path = *inst_path_it;

				/* check if this state needs to be avoided */
				bool to_avoid = false;
				for(State obst : avoid){
					to_avoid |= next.contains(obst);
				}

				/* check if this state has already been visited */
				for(State rep : visited){
					to_avoid |= (next == rep);
				}

				if(!to_avoid){
					/* add the neighbors to the known state set */
					std::set<State> next_visited = visited;
					next_visited.insert(next);

					/* recursion */
					Path next_path = path_so_far << inst_path;
					std::vector<Path> viable_paths = _recursive_dfs(next, to, avoid, next_visited, next_path, false);

					paths.insert(paths.end(), viable_paths.begin(), viable_paths.end());
				}

				++next_state_it;
				++inst_path_it;
			}
		}
		return paths;
	}
}


std::vector<PathWay> StateModel::pathway_find_dfs(const State from, const State to, std::vector<State> avoid){
	std::vector<PathWay> all_pathways;

	if(from.dimension() < to.dimension()){
		return all_pathways;
	}else if(to.empty() || from.empty()){
		return all_pathways;
	}else{
		std::vector<Path> all_paths = _recursive_dfs(from, to, avoid, std::set<State>({from}), Path(from), false);
		std::map<std::pair<State, std::vector<State>>, std::vector<Path>> cycles_memo;

		/** find cycles in existing paths **/
		for(Path path : all_paths){
			PathWay pathway(from);
			std::vector<State> verboten_states = avoid;
			bool avoid_to = true;

			/* must be done in reverse to avoid double-counting */
			auto si = path.states.rbegin();
			for(auto ti = path.transitions.rbegin(); (ti != path.transitions.rend() && si != path.states.rend()); ++ti){
				transition_p t = *ti;
				State ts = *si;
				State fs = ts.move_back(t);
				++si;

				/* the to state is added to the path's list of states */
				pathway >>= t;

				/* add the to state to the list of obstacles */
				if(ts != fs && avoid_to){
					verboten_states.push_back(ts);
					avoid_to = false;
				}

				if(t->type != Transition::CONTROLLED){
					/* cannot stop in the middle of a 'turn' */
					pathway.cycles.push_front(std::vector<Path>());
					continue;
				}else{
					avoid_to = true;
				}

				/* find and add cycles */
				std::pair<State, std::vector<State>> key({fs, verboten_states});
				if(cycles_memo.find(key) == cycles_memo.end()){
					cycles_memo[key] = find_all_paths_back_around(fs, verboten_states);
				}
				pathway.cycles.push_front(cycles_memo[key]);
			}

			all_pathways.push_back(pathway);
		}

		std::sort(all_pathways.begin(), all_pathways.end());

		return all_pathways;
	}
}

std::map<std::string, std::set<transition_p>> StateModel::all_transitions_from(const State from){
	std::map<std::string, std::set<transition_p>> tm;
	std::map<std::string, bool> disabled;

	for(const transition_p t : transitions){
		if(from.contains(t->from)){
			/* check transition conditions */
			bool pass = (disabled.find(t->label) == disabled.end()) && t->available_at(from);

			if(pass){
				tm[t->label].insert(t);
			}else{
				tm.erase(t->label);
				disabled[t->label] = true;
			}
		}
	}

	return tm;
}

std::map<std::string, std::set<transition_p>> StateModel::natural_transitions_from(const State from){
	std::map<std::string, std::set<transition_p>> tm;
	std::map<std::string, bool> disabled;

	for(const transition_p t : transitions){
		if(from.contains(t->from) && t->type == Transition::NATURAL){
			/* check transition conditions */
			bool pass = (disabled.find(t->label) == disabled.end()) && t->available_at(from);

			if(pass){
				tm[t->label].insert(t);
			}else{
				tm.erase(t->label);
				disabled[t->label] = true;
			}
		}
	}

	return tm;
}

std::map<std::string, std::set<transition_p>> StateModel::controlled_transitions_from(const State from, unsigned int restriction){
	std::map<std::string, std::set<transition_p>> tm;
	std::map<std::string, bool> disabled;

	for(const transition_p t : transitions){
		if(from.contains(t->from) && t->type == Transition::CONTROLLED && t->level <= restriction){
			/* check transition conditions */
			bool pass = (disabled.find(t->label) == disabled.end()) && t->available_at(from);

			if(pass){
				tm[t->label].insert(t);
			}else{
				tm.erase(t->label);
				disabled[t->label] = true;
			}
		}
	}

	if(allow_wait){
		tm.insert({"", std::set<transition_p>()});
	}

	return tm;
}

std::map<std::string, std::set<transition_p>> StateModel::external_transitions_from(const State from){
	std::map<std::string, std::set<transition_p>> tm;
	std::map<std::string, bool> disabled;

	for(const transition_p t : transitions){
		if(from.contains(t->from) && t->type == Transition::EXTERNAL){
			/* check transition conditions */
			bool pass = (disabled.find(t->label) == disabled.end()) && t->available_at(from);

			if(pass){
				tm[t->label].insert(t);
			}else{
				tm.erase(t->label);
				disabled[t->label] = true;
			}
		}
	}

	return tm;
}

std::set<transition_p> StateModel::prioritized_natural_transitions_from(const State from){
	std::map<std::string, std::set<transition_p>> tm = natural_transitions_from(from);
	std::map<State, std::vector<transition_p>> ntv;

	/* check the bundle and see if there are multiple transitions
	 * that originate from the same atomic state. if so, only apply
	 * the one with the lowest level / highest priority */
	 for(auto& kv : tm){
	 	for(const transition_p t : kv.second){
	 		ntv[t->from].push_back(t);
	 	}
	 }

	std::set<transition_p> nts;
	for(auto& kv : ntv){
		/* pick the element with the lowest level */
		nts.insert(*std::min_element(kv.second.begin(), kv.second.end(), 
			[](transition_p l, transition_p r){
				return l->level < r->level;
		}));
	}

	return nts;
}


std::map<std::string, std::set<transition_p>> StateModel::all_transitions_to(const State to){
	std::map<std::string, std::set<transition_p>> tm;
	std::map<std::string, bool> disabled;

	for(const transition_p t : transitions){
		if(to.contains(t->to)){
			/* check transition conditions */
			bool pass = (disabled.find(t->label) == disabled.end()) && t->available_at(to.move_back(t));

			if(pass){
				tm[t->label].insert(t);
			}else{
				tm.erase(t->label);
				disabled[t->label] = true;
			}
		}
	}

	return tm;
}

std::map<std::string, std::set<transition_p>> StateModel::natural_transitions_to(const State to){
	std::map<std::string, std::set<transition_p>> tm;
	std::map<std::string, bool> disabled;

	for(const transition_p t : transitions){
		if(to.contains(t->to) && t->type == Transition::NATURAL){
			/* check transition conditions */
			bool pass = (disabled.find(t->label) == disabled.end()) && t->available_at(to.move_back(t));

			if(pass){
				tm[t->label].insert(t);
			}else{
				tm.erase(t->label);
				disabled[t->label] = true;
			}
		}
	}

	return tm;
}

std::map<std::string, std::set<transition_p>> StateModel::controlled_transitions_to(const State to, unsigned int restriction){
	std::map<std::string, std::set<transition_p>> tm;
	std::map<std::string, bool> disabled;

	for(const transition_p t : transitions){
		if(to.contains(t->to) && t->type == Transition::CONTROLLED && t->level <= restriction){
			/* check transition conditions */
			bool pass = (disabled.find(t->label) == disabled.end()) && t->available_at(to.move_back(t));

			if(pass){
				tm[t->label].insert(t);
			}else{
				tm.erase(t->label);
				disabled[t->label] = true;
			}
		}
	}

	if(allow_wait){
		tm.insert({"", std::set<transition_p>()});
	}

	return tm;
}

std::map<std::string, std::set<transition_p>> StateModel::external_transitions_to(const State to){
	std::map<std::string, std::set<transition_p>> tm;
	std::map<std::string, bool> disabled;

	for(const transition_p t : transitions){
		if(to.contains(t->to) && t->type == Transition::EXTERNAL){
			/* check transition conditions */
			bool pass = (disabled.find(t->label) == disabled.end()) && t->available_at(to.move_back(t));

			if(pass){
				tm[t->label].insert(t);
			}else{
				tm.erase(t->label);
				disabled[t->label] = true;
			}
		}
	}

	return tm;
}

//-----------------------------------------------------------------------------

StateMachine::StateMachine(const StateModel& model, State initial) :
StateModel(model),
current_state(initial),
previous_state(),
enter_callbacks(),
remain_callbacks(),
exit_callbacks()
{

}

std::list<State> StateMachine::move(std::string input){
	std::list<State> passed_states;

	previous_state = current_state;
	std::map<std::string, std::set<transition_p>> cts = controlled_transitions_from(current_state);
	for(const transition_p t : cts[input]){
		current_state = current_state.move(t);
	}

	/* report state changes after controlled transitions */
	if(current_state != previous_state){
		passed_states.push_back(current_state);
	}

	/* enter callbacks */
	for (auto const& scb : enter_callbacks){
		if(!previous_state.contains(scb.first) && current_state.contains(scb.first)){
			for(auto const& cb : scb.second){
				cb();
			}
		}
	}

	/* exit callbacks */
	for (auto const& scb : exit_callbacks){
		if(previous_state.contains(scb.first) && !current_state.contains(scb.first)){
			for(auto const& cb : scb.second){
				cb();
			}
		}
	}

	State intermediate_state = current_state;
	std::set<transition_p> nts = prioritized_natural_transitions_from(current_state);
	for(const transition_p t : nts){
		current_state = current_state.move(t);
	}

	/* report state changes after natural transitions */
	if(current_state != intermediate_state){
		passed_states.push_back(current_state);
	}

	/* enter callbacks */
	for (auto const& scb : enter_callbacks){
		if(!intermediate_state.contains(scb.first) && current_state.contains(scb.first)){
			for(auto const& cb : scb.second){
				cb();
			}
		}
	}

	/* remain callbacks */
	for(auto const& scb: remain_callbacks){
		if(previous_state.contains(scb.first) && current_state.contains(scb.first) && intermediate_state.contains(scb.first)){
			for(auto const& cb : scb.second){
				cb();
			}
		}	
	}

	/* exit callbacks */
	for (auto const& scb : exit_callbacks){
		if(intermediate_state.contains(scb.first) && !current_state.contains(scb.first)){
			for(auto const& cb : scb.second){
				cb();
			}
		}
	}

	return passed_states;
}

State StateMachine::get_state(){
	return current_state;
}

void StateMachine::on_enter(const State s, std::function<void(void)> cb){
	enter_callbacks[s].push_back(cb);
}

void StateMachine::on_remain(const State s, std::function<void(void)> cb){
	remain_callbacks[s].push_back(cb);
}

void StateMachine::on_exit(const State s, std::function<void(void)> cb){
	exit_callbacks[s].push_back(cb);
}

Path StateMachine::find_path(const State to, unsigned int restriction){
	return StateModel::find_path(current_state, to, restriction);
}

//-----------------------------------------------------------------------------

std::vector<std::string> qualifier_names({"=", "!=", "<", ">", "<=", ">=", "*"});

DataState::DataState(DataSource source, DataState::Qualifier qualifier, int value) :
AtomicState("" + source->name + " " + qualifier_names[qualifier] + " " + std::to_string(value), 
			"" + source->name + " " + qualifier_names[qualifier] + " " + std::to_string(value)),
source(source),
qualifier(qualifier),
value(value)
{

}

bool DataState::is_subset(const DataState& o) const {
	if(source != o.source){
		return false;
	}

	switch(qualifier){
		case EQUALS:
			switch(o.qualifier){
				case EQUALS:
					return value == o.value;
				case NOT_EQUALS:
					return value != o.value;
				case LESS_THAN:
					return value < o.value;
				case GREATER_THAN:
					return value > o.value;
				case LESS_THAN_EQUALS:
					return value <= o.value;
				case GREATER_THAN_EQUALS:
					return value >= o.value;
				case NONE:
					return true;
				default:
					return false;
			}
			break;
		case NOT_EQUALS:
			switch(o.qualifier){
				case NOT_EQUALS:
					return value == o.value;
				case NONE:
					return true;
				default:
					return false;
			}
			break;
		case LESS_THAN:
			switch(o.qualifier){
				case NOT_EQUALS:
					return value <= o.value;
				case LESS_THAN:
					return value <= o.value;
				case LESS_THAN_EQUALS:
					return value <= o.value - 1;
				case NONE:
					return true;
				default:
					return false;
			}
			break;
		case GREATER_THAN:
			switch(o.qualifier){
				case NOT_EQUALS:
					return value >= o.value;
				case GREATER_THAN:
					return value >= o.value;
				case GREATER_THAN_EQUALS:
					return value >= o.value + 1;
				case NONE:
					return true;
				default:
					return false;
			}
			break;
		case LESS_THAN_EQUALS:
			switch(o.qualifier){
				case NOT_EQUALS:
					return value < o.value;
				case LESS_THAN:
					return value < o.value;
				case LESS_THAN_EQUALS:
					return value <= o.value;
				case NONE:
					return true;
				default:
					return false;
			}
			break;
		case GREATER_THAN_EQUALS:
			switch(o.qualifier){
				case NOT_EQUALS:
					return value > o.value;
				case GREATER_THAN:
					return value > o.value;
				case GREATER_THAN_EQUALS:
					return value >= o.value;
				case NONE:
					return true;
				default:
					return false;
			}
			break;

		case NONE:
			switch(o.qualifier){
				case NONE:
					return true;
				default:
					return false;
			}
			break;
		default:
			return false;
	}
}

bool DataState::is_subset(const dstate_p o) const {
	return is_subset(*o);
}

AtomicStateType DataState::type() const {
	return AtomicStateType::INT_DATA;
}

//-----------------------------------------------------------------------------

DataModel::DataModel(std::string name, int lb, int ub) :
name(name),
lower_bound(lb),
upper_bound(ub)
{

}

DataSource DataModel::create_source(std::string name){
	return DataSource(new DataModel(name, std::numeric_limits<int>::min(), std::numeric_limits<int>::max()));
}

DataSource DataModel::create_source(std::string name, int lb, int ub){
	return DataSource(new DataModel(name, lb, ub));
}

dstate_p DataModel::value_at(int x){
	return value_eq(x);
}

dstate_p DataModel::value_eq(int x){
	return dstate_p(new DataState(shared_from_this(), DataState::EQUALS, x));
}

dstate_p DataModel::value_lt(int x){
	return dstate_p(new DataState(shared_from_this(), DataState::LESS_THAN, x));
}

dstate_p DataModel::value_gt(int x){
	return dstate_p(new DataState(shared_from_this(), DataState::GREATER_THAN, x));
}

dstate_p DataModel::value_neq(int x){
	return dstate_p(new DataState(shared_from_this(), DataState::NOT_EQUALS, x));
}

dstate_p DataModel::value_leq(int x){
	return dstate_p(new DataState(shared_from_this(), DataState::LESS_THAN_EQUALS, x));
}

dstate_p DataModel::value_geq(int x){
	return dstate_p(new DataState(shared_from_this(), DataState::GREATER_THAN_EQUALS, x));
}

dstate_p DataModel::value_any(){
	return dstate_p(new DataState(shared_from_this(), DataState::NONE, 0));
}

dstate_p DataModel::value_change(int x){
	return dstate_p(new DataState(shared_from_this(), DataState::NONE, x));
}

//-----------------------------------------------------------------------------

inline bool operator==(const astate_p& x, const astate_p& y){
	if(x->type() == AtomicStateType::INT_DATA && y->type() == AtomicStateType::INT_DATA){
		DataState& xx = static_cast<DataState&>(*x);
		DataState& yy = static_cast<DataState&>(*y);
		return xx.is_subset(yy) && yy.is_subset(xx);
	}else if(x->type() == AtomicStateType::INT_DATA || y->type() == AtomicStateType::INT_DATA){
		return false;
	}else{
		return x.get() == y.get();
	}
}

inline bool operator<(const astate_p& x, const astate_p& y){
	if(x->type() == AtomicStateType::INT_DATA && y->type() == AtomicStateType::INT_DATA){
		DataState& xx = static_cast<DataState&>(*x);
		DataState& yy = static_cast<DataState&>(*y);
		if(xx.source != yy.source){
			return xx.source < yy.source;
		}else{
			return xx.value < yy.value;
		}
	}else if(x->type() == AtomicStateType::INT_DATA){
		return false;
	}else if(y->type() == AtomicStateType::INT_DATA){
		return true;
	}else{
		return x.get() < y.get();
	}
}

//-----------------------------------------------------------------------------

} // namespace cerebellum

#endif // #ifndef _CEREBELLUM_