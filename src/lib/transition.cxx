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

#include <cerebellum.h>

using namespace cerebellum;
using namespace std;

//-----------------------------------------------------------------------------

Transition::Transition(string label, const State& from, const State& to, bool controllable, 
                double cost, double probability, int priority, bool active_conditions) :
label(label),
from(from),
to(to),
controllable(controllable),
cost(cost),
probability(probability),
priority(priority),
conditions(),
active_conditions(active_conditions),
fail(nullptr)
{

}

transition_p Transition::create_natural(string label, const astate_p from, 
				const astate_p to, double probability, double cost, int priority){
	transition_p tp(new Transition(label, State({from}), State({to}), 
						false, cost, probability, priority, true));
	transition_p tpf(new Transition("!" + label, State({from}), State({from}),
						false, 0, 1.0 - probability, priority, true));
	tp->fail = tpf;
	return tp;
}

transition_p Transition::create_controlled(string label, const astate_p from,
				const astate_p to, double cost, double probability, int priority){
	transition_p tp(new Transition(label, State({from}), State({to}), 
						true, cost, probability, priority, false));
	transition_p tpf(new Transition("!" + label, State({from}), State({from}),
						true, 0, 1.0 - probability, priority, false));
	tp->fail = tpf;
	return tp;
}

transition_p Transition::create_natural(string label, const State& from, 
				const State& to, double probability, double cost, int priority){
	transition_p tp(new Transition(label, from, to, 
						false, cost, probability, priority, true));
	transition_p tpf(new Transition("!" + label, from, from,
						false, 0, 1.0 - probability, priority, true));
	tp->fail = tpf;
	return tp;
}

transition_p Transition::create_controlled(string label, const State& from,
				const State& to, double cost, double probability, int priority){
	transition_p tp(new Transition(label, from, to, 
						true, cost, probability, priority, false));
	transition_p tpf(new Transition("!" + label, from, from,
						true, 0, 1.0 - probability, priority, false));
	tp->fail = tpf;
	return tp;
}

bool Transition::available_at(const State& x) const {
	/* check transition conditions */
	bool pass = !active_conditions;
	for(const set<State> s : conditions){
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
	vector<pair<astate_p, astate_p>> changes;
	for(int i = 0; i < from.dimension(); i++){
		changes.push_back(make_pair(from.components[i], to.components[i]));
	}

	for(pair<astate_p, astate_p> change : changes){
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
	set<State> S;
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
	set<State> S;
	S.insert(State(as));
	conditions.push_back(S);
	if(fail){
		fail->activate(as);
	}
}

void Transition::activate(initializer_list<astate_p> as){
	if(!active_conditions){
		active_conditions = true;
		conditions.clear();
	}
	set<State> S;
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
	set<State> S;
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
	set<State> S;
	S.insert(State(as));
	conditions.push_back(S);
	if(fail){
		fail->inhibit(as);
	}
}

void Transition::inhibit(initializer_list<astate_p> as){
	if(active_conditions){
		active_conditions = false;
		conditions.clear();
	}
	set<State> S;
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
probability(1)
{
	states.push_back(start);
}

Path::Path(const Path& b) :
inputs(b.inputs),
transitions(b.transitions),
states(b.states),
cost(b.cost),
probability(b.probability)
{

}

bool Path::operator<(const Path& b) const {
	// order by cost, probability, input size, transitions size, state size
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

	if(t->controllable){
		inputs.push_back(t->label);
	}
	transitions.push_back(t);
	states.push_back(states.back().move(t));
}

void Path::operator>>=(transition_p t){
	cost += t->cost;
	probability *= t->probability;

	if(t->controllable){
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
	map<unsigned int, double> memo;

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
	for(vector<Path> cycles_ : cycles){
		for(Path cycle : cycles_){
			if(cycle.inputs.size() > M){
				M = cycle.inputs.size();
			}
		}
	}
	unsigned int m = 0;

	map<unsigned int, double> memo;
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
	}while(m < M && i++ < numeric_limits<unsigned int>::max());
	return total_probability;
}

double PathWay::probability_arrival_at_step(unsigned int n, map<unsigned int, double>& memo){
	if(n < inputs.size()){
		return 0;
	}else if(n == inputs.size()){
		return probability;
	}

	/* n > inputs.size() */
	if(memo.find(n) == memo.end()){
		double x = 0;
		for(vector<Path> cycles_now : cycles){
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