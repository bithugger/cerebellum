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
#include <sstream>

namespace cerebellum {

class AtomicState;
typedef std::shared_ptr<AtomicState> astate_p;
class Transition;
typedef std::shared_ptr<Transition> transition_p;

//-----------------------------------------------------------------------------
// Atomic State
//-----------------------------------------------------------------------------

class AtomicState {

public:

	static astate_p create(std::string);

	virtual ~AtomicState() = default;

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
	State(std::vector<astate_p>);
	State(std::initializer_list<astate_p>);

	virtual ~State() = default;

	std::vector<astate_p> components;

	bool empty() const;

	int dimension() const;

	bool contains(const astate_p) const;
	bool contains(std::vector<astate_p>) const;
	bool contains(const State&) const;

	bool operator==(const State&) const;
	bool operator!=(const State&) const;

	bool operator<(const State&) const;

	State move(const transition_p) const;
	State move_back(const transition_p) const;

	friend std::ostream& operator<<(std::ostream& os, const State&);

protected:

};

//-----------------------------------------------------------------------------
// Transition
//-----------------------------------------------------------------------------

class Transition {

public:

	static transition_p create_natural(std::string label, 
							const astate_p from, const astate_p to, 
							double probability = 1, unsigned int priority = 0);

	static transition_p create_controlled(std::string label, 
							const astate_p from, const astate_p to, 
							unsigned int restriction = 0, double cost = 1);

	static transition_p create_external(std::string label, 
							const astate_p from, const astate_p to,
							double cost = 1, double probability = 1);

	virtual ~Transition() = default;

	const std::string label;

	const astate_p from;
	const astate_p to;

	const enum transition_type {
		NATURAL,
		CONTROLLED,
		EXTERNAL
	} type;
	
	const double cost;
	const double probability;

	const unsigned int level;

	std::set<astate_p> conditions;
	bool active_conditions;

protected:

	Transition(std::string label, const astate_p from, const astate_p to, 
				transition_type type, double cost, double probability, 
				unsigned int level, bool active_conditions);

};

//-----------------------------------------------------------------------------
// State Model
//-----------------------------------------------------------------------------

class StateModel {

public:

	StateModel(std::vector<astate_p> states, std::vector<transition_p> transitions);
	StateModel(std::initializer_list<astate_p> states, std::vector<transition_p> transitions);
	StateModel(std::vector<astate_p> states, std::initializer_list<transition_p> transitions);
	StateModel(std::initializer_list<astate_p> states, std::initializer_list<transition_p> transitions);
	StateModel(const StateModel&);
	virtual ~StateModel() = default;

	const std::set<astate_p> atomic_states;
	const std::set<transition_p> transitions;

	std::list<std::string> find_path(const State from, const State to, 
				unsigned int restriction = std::numeric_limits<unsigned int>::max());

protected:

	std::map<std::string, std::set<transition_p>> all_transitions_from(const State from);

	std::map<std::string, std::set<transition_p>> natural_transitions_from(const State from);

	std::map<std::string, std::set<transition_p>> controlled_transitions_from(const State from,
				unsigned int restriction = std::numeric_limits<unsigned int>::max());

	std::map<std::string, std::set<transition_p>> external_transitions_from(const State from);

	std::set<transition_p> prioritized_natural_transitions_from(const State from);
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

protected:

	State current_state;

};

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


std::ostream& operator<<(std::ostream& os, const AtomicState& a){
	return os << a.label;
}

//-----------------------------------------------------------------------------

State::State() :
components()
{

}

State::State(std::vector<astate_p> components) :
components(components)
{
	std::sort(this->components.begin(), this->components.end());
}

State::State(std::initializer_list<astate_p> components) :
components(components)
{
	std::sort(this->components.begin(), this->components.end());
}

bool State::empty() const {
	return components.empty();
}

int State::dimension() const {
	return components.size();
}

bool State::contains(const astate_p x) const {
	return std::find(components.begin(), components.end(), x) != components.end();
}

bool State::contains(std::vector<astate_p> X) const {
	std::vector<astate_p> a = components;
	std::vector<astate_p> b = X;
	return std::includes(a.begin(), a.end(), b.begin(), b.end());
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
		for(int i = 0; i < dimension(); i++){
			if(components[i] != o.components[i]){
				return components[i] < o.components[i];
			}
		}
		return false;
	}
}

State State::move(const transition_p t) const {
	if(contains(t->from)){
		std::vector<astate_p> new_components = components;
		auto it = std::find(new_components.begin(), new_components.end(), t->from);
		*it = t->to;
		return State(new_components);
	}else{
		return State();
	}
}

State State::move_back(const transition_p t) const {
	if(contains(t->to)){
		std::vector<astate_p> new_components = components;
		auto it = std::find(new_components.begin(), new_components.end(), t->to);
		*it = t->from;
		return State(new_components);
	}else{
		return State();
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

Transition::Transition(std::string label, const astate_p from, const astate_p to, 
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
active_conditions(active_conditions)
{

}

transition_p Transition::create_natural(std::string label, const astate_p from, 
				const astate_p to, double probability, unsigned int priority){
	return transition_p(new Transition(label, from, to, NATURAL, 1, probability, priority, true));
}

transition_p Transition::create_controlled(std::string label, const astate_p from,
				const astate_p to, unsigned int restriction, double cost){
	return transition_p(new Transition(label, from, to, CONTROLLED, cost, 1, restriction, false));
}

transition_p Transition::create_external(std::string label, const astate_p from, 
				const astate_p to, double cost, double probability){
	return transition_p(new Transition(label, from, to, EXTERNAL, cost, probability, 0, false));
}

//-----------------------------------------------------------------------------

StateModel::StateModel(std::vector<astate_p> states, std::vector<transition_p> transitions) :
atomic_states(states.begin(), states.end()),
transitions(transitions.begin(), transitions.end())
{

}

StateModel::StateModel(std::initializer_list<astate_p> states, std::vector<transition_p> transitions) :
atomic_states(states.begin(), states.end()),
transitions(transitions.begin(), transitions.end())
{

}

StateModel::StateModel(std::vector<astate_p> states, std::initializer_list<transition_p> transitions) :
atomic_states(states.begin(), states.end()),
transitions(transitions.begin(), transitions.end())
{

}

StateModel::StateModel(std::initializer_list<astate_p> states, std::initializer_list<transition_p> transitions) :
atomic_states(states.begin(), states.end()),
transitions(transitions.begin(), transitions.end())
{

}

StateModel::StateModel(const StateModel& b) :
atomic_states(b.atomic_states),
transitions(b.transitions)
{

}

std::list<std::string> StateModel::find_path(const State from, const State to, unsigned int restriction){
	
	std::list<std::string> path;

	if(from.dimension() < to.dimension()){
		return path;
	}else if(to.empty()){
		return path;
	}

	std::set<State> visited_states;
	std::set<State> known_states;
	std::map<State, double> cost_map;

	typedef std::pair<std::string, std::set<transition_p>> transition_bundle;

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
			if(visited_states.find(kv.first) == visited_states.end()){
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

	/* is the destination reachable? */
	if(u == from || trans_map.find(dest) != trans_map.end()){
		u = dest;
		while(u != from){
			std::pair<transition_bundle, std::set<transition_p>> ts = trans_map.find(u)->second;
			transition_bundle& tb = ts.first;
			path.push_front(tb.first);

			/* apply in reverse order */
			/* first the natural, then the controlled */
			for(const transition_p nt : ts.second){
				u = u.move_back(nt);
			}

			for(const transition_p t : tb.second){
				u = u.move_back(t);
			}
		}
	}

	return path;
}

std::map<std::string, std::set<transition_p>> StateModel::all_transitions_from(const State from){
	std::map<std::string, std::set<transition_p>> tm;

	for(const transition_p t : transitions){
		if(from.contains(t->from)){
			/* check transition conditions */
			bool pass = true;
			for(const astate_p c : t->conditions){
				pass &= (!from.contains(c) ^ t->active_conditions);
			}

			if(pass){
				tm[t->label].insert(t);
			}
		}
	}

	return tm;
}

std::map<std::string, std::set<transition_p>> StateModel::natural_transitions_from(const State from){
	std::map<std::string, std::set<transition_p>> tm;

	for(const transition_p t : transitions){
		if(from.contains(t->from) && t->type == Transition::NATURAL){
			/* check transition conditions */
			bool pass = true;
			for(const astate_p c : t->conditions){
				pass &= (!from.contains(c) ^ t->active_conditions);
			}

			if(pass){
				tm[t->label].insert(t);
			}
		}
	}

	return tm;
}

std::map<std::string, std::set<transition_p>> StateModel::controlled_transitions_from(const State from, unsigned int restriction){
	std::map<std::string, std::set<transition_p>> tm;

	for(const transition_p t : transitions){
		if(from.contains(t->from) && t->type == Transition::CONTROLLED && t->level <= restriction){
			/* check transition conditions */
			bool pass = true;
			for(const astate_p c : t->conditions){
				pass &= (!from.contains(c) ^ t->active_conditions);
			}

			if(pass){
				tm[t->label].insert(t);
			}
		}
	}

	return tm;
}

std::map<std::string, std::set<transition_p>> StateModel::external_transitions_from(const State from){
	std::map<std::string, std::set<transition_p>> tm;

	for(const transition_p t : transitions){
		if(from.contains(t->from) && t->type == Transition::EXTERNAL){
			/* check transition conditions */
			bool pass = true;
			for(const astate_p c : t->conditions){
				pass &= (!from.contains(c) ^ t->active_conditions);
			}

			if(pass){
				tm[t->label].insert(t);
			}
		}
	}

	return tm;
}

std::set<transition_p> StateModel::prioritized_natural_transitions_from(const State from){
	std::map<astate_p, std::vector<transition_p>> ntv;

	/* check the bundle and see if there are multiple transitions
	 * that originate from the same atomic state. if so, only apply
	 * the one with the lowest level / highest priority */
	for(const transition_p t : transitions){
		if(from.contains(t->from) && t->type == Transition::NATURAL){
			/* check transition conditions */
			bool pass = true;
			for(const astate_p c : t->conditions){
				pass &= (!from.contains(c) ^ t->active_conditions);
			}

			if(pass){
				ntv[t->from].push_back(t);
			}
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

//-----------------------------------------------------------------------------

StateMachine::StateMachine(const StateModel& model, State initial) :
StateModel(model),
current_state(initial)
{

}

std::list<State> StateMachine::move(std::string input){
	std::list<State> states;

	State old_state = current_state;
	std::map<std::string, std::set<transition_p>> cts = controlled_transitions_from(current_state);
	for(const transition_p t : cts[input]){
		current_state = current_state.move(t);
	}

	/* report state changes after controlled transitions */
	if(current_state != old_state){
		states.push_back(current_state);	
	}

	old_state = current_state;
	std::set<transition_p> nts = prioritized_natural_transitions_from(current_state);
	for(const transition_p t : nts){
		current_state = current_state.move(t);
	}

	/* report state changes after natural transitions */
	if(current_state != old_state){
		states.push_back(current_state);	
	}

	return states;
}

State StateMachine::get_state(){
	return current_state;
}

} // namespace cerebellum

#endif // #ifndef _CEREBELLUM_