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
#include <utility>
#include <algorithm>
#include <tuple>

using namespace cerebellum;
using namespace std;


typedef pair<string, set<transition_p>> transition_bundle;
typedef pair<transition_bundle, list<transition_p>> move_pair;

//-----------------------------------------------------------------------------

StateModel::StateModel(vector<transition_p> transitions, bool wait) :
transitions(transitions.begin(), transitions.end()),
allow_wait(wait)
{

}

StateModel::StateModel(initializer_list<transition_p> transitions, bool wait) :
transitions(transitions.begin(), transitions.end()),
allow_wait(wait)
{

}

StateModel::StateModel(const StateModel& b) :
transitions(b.transitions),
allow_wait(b.allow_wait)
{

}

Path StateModel::find_path_around(const State from, const State to, vector<State> avoid){
	return path_find_djikstra(from, to, avoid);
}

Path StateModel::find_path_around(const State from, const State to, const State avoid){
	vector<State> avoids;
	avoids.push_back(avoid);
	return path_find_djikstra(from, to, avoids);
}

Path StateModel::find_path_around(const State from, const State to, initializer_list<State> avoid){
	vector<State> avoids(avoid.begin(), avoid.end());
	return path_find_djikstra(from, to, avoids);
}

Path StateModel::find_path(const State from, const State to){
	vector<State> avoids;
	return path_find_djikstra(from, to, avoids);
}

Path StateModel::find_best_path_over_likelihood(const State from, const State to, double prob_limit){
	vector<State> avoids;
	return path_find_b3(from, to, avoids, prob_limit, numeric_limits<double>::max());
}

Path StateModel::find_best_path_around_over_likelihood(const State from, const State to, 
			const State avoid, double prob_limit){
	vector<State> avoids;
	avoids.push_back(avoid);
	return path_find_b3(from, to, avoids, prob_limit, numeric_limits<double>::max());
}

Path StateModel::find_best_path_around_over_likelihood(const State from, const State to, 
			vector<State> avoid, double prob_limit){
	return path_find_b3(from, to, avoid, prob_limit, numeric_limits<double>::max());
}

Path StateModel::find_best_path_around_over_likelihood(const State from, const State to, 
			initializer_list<State> avoid, double prob_limit){
	vector<State> avoids(avoid.begin(), avoid.end());
	return path_find_b3(from, to, avoids, prob_limit, numeric_limits<double>::max());
}

Path StateModel::find_likeliest_path_under_cost(const State from, const State to, double cost_limit){
	vector<State> avoids;
	return path_find_b3(from, to, avoids, 0, cost_limit);
}

Path StateModel::find_likeliest_path_around_under_cost(const State from, const State to, 
			const State avoid, double cost_limit){
	vector<State> avoids;
	avoids.push_back(avoid);
	return path_find_b3(from, to, avoids, 0, cost_limit);
}

Path StateModel::find_likeliest_path_around_under_cost(const State from, const State to, 
			vector<State> avoid, double cost_limit){
	return path_find_b3(from, to, avoid, 0, cost_limit);
}

Path StateModel::find_likeliest_path_around_under_cost(const State from, const State to, 
			initializer_list<State> avoid, double cost_limit){
	vector<State> avoids(avoid.begin(), avoid.end());
	return path_find_b3(from, to, avoids, 0, cost_limit);
}


vector<Path> StateModel::find_all_paths(const State from, const State to){
	vector<State> avoids;
	return find_all_paths_around(from, to, avoids);
}

vector<Path> StateModel::find_all_paths_around(const State from, const State to, const State avoid){
	vector<State> avoids;
	avoids.push_back(avoid);
	return find_all_paths_around(from, to, avoids);
}

vector<Path> StateModel::find_all_paths_around(const State from, const State to, vector<State> avoid){
	return path_find_dfs(from, to, avoid, false);
}


vector<Path> StateModel::find_all_paths_back(const State x){
	vector<State> avoids;
	return find_all_paths_back_around(x, avoids);
}

vector<Path> StateModel::find_all_paths_back_around(const State x, const State avoid){
	vector<State> avoids;
	avoids.push_back(avoid);
	return find_all_paths_back_around(x, avoid);
}

vector<Path> StateModel::find_all_paths_back_around(const State x, vector<State> avoid){
	return path_find_dfs(x, x, avoid, true);
}


vector<PathWay> StateModel::find_all_pathways(const State from, const State to){
	vector<State> avoids;
	return find_all_pathways_around(from, to, avoids);
}

vector<PathWay> StateModel::find_all_pathways_around(const State from, const State to, const State avoid){
	vector<State> avoids;
	avoids.push_back(avoid);
	return find_all_pathways_around(from, to, avoids);
}

vector<PathWay> StateModel::find_all_pathways_around(const State from, const State to, vector<State> avoid){
	return pathway_find_dfs(from, to, avoid);
}

Path StateModel::path_find_djikstra(const State from, const State to, vector<State> avoid){
	Path path(from);

	if(from.dimension() < to.dimension()){
		return path;
	}else if(to.empty() || from.empty()){
		return path;
	}

	set<State> visited_states;
	set<State> known_states;
	map<State, double> cost_map;

	/* map from each state to the controlled/natural pair of transitions */
	/* the first item in the pair, the transition_bundle type, is controlled */
	/* the second item in the pair, the set of transitions, is natural */
	map<State, move_pair> trans_map;

	known_states.insert(from);
	cost_map[from] = 0;
	State u = from;
	State dest = to;

	while(visited_states != known_states){
		/* find unvisited vertex with min cost */
		double cost = numeric_limits<double>::max();
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
		map<string, set<transition_p>> t_controlled = controlled_transitions_from(u);
		/* see where they go */
		for(const transition_bundle& tb : t_controlled){
			/* determine the cost, probability, and destination of this move */
			double tcost = 0;
			State nu = u;
			bool to_avoid = false;
			for(const transition_p t : tb.second){
				nu = nu.move(t);
				tcost += t->cost;

				/* check if this state needs to be avoided */
				for(State obst : avoid){
					to_avoid |= nu.contains(obst);
				}
			}

			/* TODO check destination reached, although transient? */
			/* this option should be parametrized */

			/* after arriving, apply all non-conflicting natural transitions */
			list<transition_p> t_natural = nonconflicting_natural_transitions_from(nu);
			for(const transition_p nt : t_natural){
				nu = nu.move(nt);
				tcost += nt->cost;

				/* check if this state needs to be avoided */
				for(State obst : avoid){
					to_avoid |= nu.contains(obst);
				}
			}

			if(!to_avoid){
				/* add the neighbors to the known state set */
				known_states.insert(nu);

				/* see if this is a better path to reach nu */
				double ncost = cost + tcost;
				auto it = cost_map.find(nu);
				if(it == cost_map.end() || ncost < it->second){
					cost_map[nu] = ncost;
					trans_map[nu] = make_pair(tb, t_natural);
				}
			}
		}
	}

	/* is the destination reachable? */
	list<transition_p> transitions_in_path;
	if(u == from || trans_map.find(dest) != trans_map.end()){
		u = dest;
		while(u != from){
			pair<transition_bundle, list<transition_p>> ts = trans_map.find(u)->second;
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

Path StateModel::path_find_b3(const State from, const State to, vector<State> avoid, 
				double probability_limit, double cost_limit){
    Path path(from);

	if(from.dimension() < to.dimension()){
		return path;
	}else if(to.empty() || from.empty()){
		return path;
	}

    /* a convenient typedef, order: state, cost-so-far, total probability, path */
    typedef tuple<State, double, double, Path> path_solver_state;

	/* set up a heap to store the boundary states */
	struct {
		bool operator()(const path_solver_state a, const path_solver_state b){
			return get<1>(a) > get<1>(b);
		}
	} cost_comparator;
	struct {
		bool operator()(const path_solver_state a, const path_solver_state b){
			return get<2>(a) < get<2>(b);
		}
	} prob_comparator;
	bool use_cost_comp = cost_limit >= numeric_limits<double>::max();

	/* initialize */
    vector<path_solver_state> neighbour_state_heap;
    neighbour_state_heap.push_back(path_solver_state{from, 0.0, 1.0, path});
	set<State> visited_states;
	map<State, double> lowest_cost_to_reach_state;
	map<State, double> highest_prob_to_reach_state;

    while(!neighbour_state_heap.empty()){

		// /* debugging */
		// for(auto z : neighbour_state_heap){
		// 	cout << get<0>(z) << " " << get<1>(z) << " " << get<2>(z) << endl;
		// }

		/* find unvisited vertex with min cost */
		if(use_cost_comp){
        	pop_heap(neighbour_state_heap.begin(), neighbour_state_heap.end(), cost_comparator);
		}else{
        	pop_heap(neighbour_state_heap.begin(), neighbour_state_heap.end(), prob_comparator);
		}

        path_solver_state next_candidate_info = neighbour_state_heap.back();
        neighbour_state_heap.pop_back();

        State u = get<0>(next_candidate_info);
		double cost_so_far = get<1>(next_candidate_info);
		double prob_so_far = get<2>(next_candidate_info);
		Path path_so_far = get<3>(next_candidate_info);

		/* check if this state is viable */
		bool viable = true;
		if(prob_so_far < probability_limit || cost_so_far > cost_limit){
			/* not viable */
			viable = false;
		}else{
			for(const State& ob : avoid){
				if(u.contains(ob)){
					/* not viable */
					viable = false;
				}
			}

			if(lowest_cost_to_reach_state.find(u) != lowest_cost_to_reach_state.end() && lowest_cost_to_reach_state[u] < cost_so_far &&
				highest_prob_to_reach_state.find(u) != highest_prob_to_reach_state.end() && highest_prob_to_reach_state[u] > prob_so_far){
				/* this path costs more and is less likely to succeed, discard */
				viable = false;
			}
		}

		/* go to the next candidate if not viable */
		if(!viable){
			continue;
		}

		// cout << "B3 state " << u <<  " " << cost_so_far << " " << prob_so_far << endl;

		/* if arrived at destination (set), terminate */
		if(u.contains(to)){
			return path_so_far;
		}

		/* mark as visited */
		visited_states.insert(u);

		/* explore neighbouring states from here */
		/* find available controlled transitions */
		map<string, set<transition_p>> t_controlled = controlled_transitions_from(u);
		/* see where they go */
		for(const transition_bundle& tb : t_controlled){
			/* determine the cost, probability, and destination of this move */
			double cost_after_controlled = cost_so_far;
			double prob_after_controlled = prob_so_far;
			Path path_after_controlled = path_so_far;
			State state_after_controlled = u;
			bool to_avoid = false;

			if(tb.second.empty()){
				transition_p no_action = Transition::create_controlled("", u, u, 0, 0);
				path_after_controlled <<= no_action;
				/* no need to check obstacles here */
			}else{
				for(const transition_p t : tb.second){
					state_after_controlled = state_after_controlled.move(t);
					path_after_controlled <<= t;
					cost_after_controlled += t->cost;
					prob_after_controlled *= t->probability;

					/* check if this state needs to be avoided */
					for(const State& ob : avoid){
						to_avoid |= state_after_controlled.contains(ob);
					}
				}
			}

			/* after arriving, find all possible subsequent natural transitions */
			set<list<transition_p>> all_t_natural = all_potential_natural_transitions_from(state_after_controlled);

			if(all_t_natural.empty()){
				/* no natural transitions from here, just add the state after controlled */
				if(visited_states.find(state_after_controlled) == visited_states.end()){
					if(prob_after_controlled >= probability_limit && cost_after_controlled <= cost_limit){
						path_solver_state x{state_after_controlled, cost_after_controlled, prob_after_controlled, path_after_controlled};
						neighbour_state_heap.push_back(x);
						if(use_cost_comp){
							push_heap(neighbour_state_heap.begin(), neighbour_state_heap.end(), cost_comparator);
						}else{
							push_heap(neighbour_state_heap.begin(), neighbour_state_heap.end(), prob_comparator);
						}
						if(lowest_cost_to_reach_state.find(state_after_controlled) == lowest_cost_to_reach_state.end() ||
							lowest_cost_to_reach_state[state_after_controlled] > cost_after_controlled){
							lowest_cost_to_reach_state[state_after_controlled] = cost_after_controlled;
						}
						if(highest_prob_to_reach_state.find(state_after_controlled) == highest_prob_to_reach_state.end() ||
							highest_prob_to_reach_state[state_after_controlled] < prob_after_controlled){
							highest_prob_to_reach_state[state_after_controlled] = prob_after_controlled;
						}
					}
				}
			}else{
				for(auto t_naturals : all_t_natural){
					State next_state = state_after_controlled;
					Path next_path = path_after_controlled;
					double next_cost = cost_after_controlled;
					double next_prob = prob_after_controlled;

					bool to_avoid = false;
					for(auto nt : t_naturals){
						next_state = next_state.move(nt);
						next_path <<= nt;
						next_cost += nt->cost;
						next_prob *= nt->probability;
						
						/* check if this state needs to be avoided */
						for(const State& ob : avoid){
							to_avoid |= next_state.contains(ob);
						}
					}

					/* at the end, check if this state has already been visited */
					to_avoid |= visited_states.find(next_state) != visited_states.end();

					if(!to_avoid){
						if(next_prob >= probability_limit && next_cost <= cost_limit){
							path_solver_state x{next_state, next_cost, next_prob, next_path};
							neighbour_state_heap.push_back(x);
							if(use_cost_comp){
								push_heap(neighbour_state_heap.begin(), neighbour_state_heap.end(), cost_comparator);
							}else{
								push_heap(neighbour_state_heap.begin(), neighbour_state_heap.end(), prob_comparator);
							}
							if(lowest_cost_to_reach_state.find(next_state) == lowest_cost_to_reach_state.end() ||
								lowest_cost_to_reach_state[next_state] > next_cost){
								lowest_cost_to_reach_state[next_state] = next_cost;
							}
							if(highest_prob_to_reach_state.find(next_state) == highest_prob_to_reach_state.end() ||
								highest_prob_to_reach_state[next_state] < next_prob){
								highest_prob_to_reach_state[next_state] = next_prob;
							}
						}
					}
				}
			}
		}
    }
	/* every path searched, none found */
	return path;
}

vector<Path> StateModel::path_find_dfs(const State from, const State to, vector<State> avoid, bool back){
	vector<Path> all_paths;

	if(from.dimension() < to.dimension()){
		return all_paths;
	}else if(to.empty() || from.empty()){
		return all_paths;
	}else{
		set<State> visited;
		if(!back){
			visited.insert(from);
		}
		all_paths = _recursive_dfs(from, to, avoid, visited, Path(from), back);

		sort(all_paths.begin(), all_paths.end());

		return all_paths;
	}
}


vector<Path> StateModel::_recursive_dfs(const State from, const State to, vector<State> avoid, 
			set<State> visited, Path path_so_far, bool back){

	vector<Path> paths;
	if(!back && from.contains(to)){
		// reached the destination
		// return an vector with the current path
		paths.push_back(path_so_far);
		return paths;
	}else{
		/* find available controlled transitions */
		map<string, set<transition_p>> t_controlled = controlled_transitions_from(from);
		/* see where they go */
		for(const transition_bundle& tb : t_controlled){
			Path inst_path_controlled(from);
			State state_after_controlled = from;

			bool blocked = false;

			/* determine the cost and destination of this move */
			for(const transition_p t : tb.second){
				state_after_controlled = state_after_controlled.move(t);
				inst_path_controlled <<= t;

				/* check if this state needs to be avoided */
				for(State obst : avoid){
					blocked |= state_after_controlled.contains(obst);
				}
			}

			/* TODO check destination reached, although transient? */
			/* this option should be parametrized */
			if(blocked){
				continue;
			}

			/* after arriving, find all possible subsequent natural transitions */
			set<list<transition_p>> all_t_natural = all_potential_natural_transitions_from(state_after_controlled);

			/* if allowed to wait, only do so if there are non-trivial natural transitions afterwards */
			if(tb.second.empty()){
				if(!all_t_natural.empty()){
					transition_p no_action = Transition::create_controlled("", from, from, 0, 0, 1);
					inst_path_controlled <<= no_action;
				}else{
					continue;
				}
			}

			vector<State> next_states;
			vector<Path> next_inst_paths;
			if(all_t_natural.empty()){
				/* no natural transitions from here, just add the state after controlled */
				if(visited.find(state_after_controlled) == visited.end()){
					next_states.push_back(state_after_controlled);
					next_inst_paths.push_back(inst_path_controlled);
				}
			}else{
				for(auto t_naturals : all_t_natural){
					State next = state_after_controlled;
					Path inst_path = inst_path_controlled;

					bool to_avoid = false;
					for(auto nt : t_naturals){
						next = next.move(nt);
						inst_path <<= nt;
						
						/* check if this state needs to be avoided */
						for(State obst : avoid){
							to_avoid |= next.contains(obst);
						}
					}

					/* at the end, check if this state has already been visited */
					for(State rep : visited){
						to_avoid |= (next == rep);
					}

					if(!to_avoid){
						next_states.push_back(next);
						next_inst_paths.push_back(inst_path);
					}
				}
			}

			for(size_t i = 0; i < next_states.size(); i++){
				State next = next_states[i];
				Path inst_path = next_inst_paths[i];

				/* add the neighbors to the known state set */
				set<State> next_visited = visited;
				next_visited.insert(next);

				/* recursion */
				Path next_path = path_so_far << inst_path;
				vector<Path> viable_paths = _recursive_dfs(next, to, avoid, next_visited, next_path, false);

				paths.insert(paths.end(), viable_paths.begin(), viable_paths.end());
			}
		}
		return paths;
	}
}


vector<PathWay> StateModel::pathway_find_dfs(const State from, const State to, vector<State> avoid){
	vector<PathWay> all_pathways;

	if(from.dimension() < to.dimension()){
		return all_pathways;
	}else if(to.empty() || from.empty()){
		return all_pathways;
	}else{
		vector<Path> all_paths = _recursive_dfs(from, to, avoid, set<State>({from}), Path(from), false);
		map<pair<State, vector<State>>, vector<Path>> cycles_memo;

		/** find cycles in existing paths **/
		for(Path path : all_paths){
			PathWay pathway(from);
			vector<State> verboten_states = avoid;
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

				if(!t->controllable){
					/* cannot stop in the middle of a 'turn' */
					pathway.cycles.push_front(vector<Path>());
					continue;
				}else{
					avoid_to = true;
				}

				/* find and add cycles */
				pair<State, vector<State>> key({fs, verboten_states});
				if(cycles_memo.find(key) == cycles_memo.end()){
					cycles_memo[key] = find_all_paths_back_around(fs, verboten_states);
				}
				pathway.cycles.push_front(cycles_memo[key]);
			}

			all_pathways.push_back(pathway);
		}

		sort(all_pathways.begin(), all_pathways.end());

		return all_pathways;
	}
}

map<string, set<transition_p>> StateModel::all_transitions_from(const State from){
	map<string, set<transition_p>> tm;

	for(const transition_p t : transitions){
		/* check transition conditions */
		if(from.contains(t->from) && t->available_at(from)){
			tm[t->label].insert(t);
		}
	}

	return tm;
}

map<string, set<transition_p>> StateModel::natural_transitions_from(const State from){
	map<string, set<transition_p>> tm;

	for(const transition_p t : transitions){
		/* check transition conditions */
		if(!t->controllable && from.contains(t->from) && t->available_at(from)){
			tm[t->label].insert(t);
		}
	}

	return tm;
}

map<string, set<transition_p>> StateModel::controlled_transitions_from(const State from){
	map<string, set<transition_p>> tm;

	for(const transition_p t : transitions){
		/* check transition conditions */
		if(t->controllable && from.contains(t->from) && t->available_at(from)){
			tm[t->label].insert(t);
		}
	}

	if(allow_wait){
		tm.insert({"", set<transition_p>()});
	}

	return tm;
}

list<transition_p> StateModel::nonconflicting_natural_transitions_from(const State from){
	map<string, set<transition_p>> tm = natural_transitions_from(from);
	/** map sorts entries by key, so this is automatically sorted by level */
	map<unsigned int, set<transition_p>> nt_level_map;

	for(auto& kv : tm){
	 	for(const transition_p t : kv.second){
	 		nt_level_map[t->priority].insert(t);
	 	}
	}

	/* these transitions should all be applied, but some may become unavailable during the motion.
	 * successively apply each priority of transitions until all are applied or conflict arises */
	State k(from);
	list<transition_p> ntl;
	for(auto& ntp_kv : nt_level_map){
		for(auto& nt : ntp_kv.second){
			if(nt->available_at(k)){
				ntl.push_back(nt);
				k = k.move(nt);
			}
		}
	}

	return ntl;
}

set<list<transition_p>> StateModel::_recursive_expand_possible_transitions(const State x, list<transition_p> t_to_go){
	set<list<transition_p>> result;

	if(t_to_go.empty()){
		return result;
	}

	const transition_p t = t_to_go.front();
	list<transition_p> next_t_to_go(next(t_to_go.begin()), t_to_go.end());

	if(t->available_at(x)){
		/* add the no-fail case */
		if(t->probability > 0){
			set<list<transition_p>> sub_possibilities = _recursive_expand_possible_transitions(x.move(t), next_t_to_go);
			if(sub_possibilities.empty()){
				list<transition_p> l;
				l.push_back(t);
				result.insert(l);
			}else{
				for(auto lnt : sub_possibilities){
					list<transition_p> l(lnt);
					l.push_front(t);
					result.insert(l);
				}
			}
		}

		/* add the fail case */
		if(t->probability < 1){
			set<list<transition_p>> sub_possibilities = _recursive_expand_possible_transitions(x, next_t_to_go);
			if(sub_possibilities.empty()){
				list<transition_p> l;
				l.push_back(t->fail);
				result.insert(l);
			}else{
				for(auto lnt : sub_possibilities){
					list<transition_p> l(lnt);
					l.push_front(t->fail);
					result.insert(l);
				}
			}
		}

		return result;

	}else{
		return _recursive_expand_possible_transitions(x, next_t_to_go);
	}
}

set<list<transition_p>> StateModel::all_potential_natural_transitions_from(const State from){
	/** some natural transitions can fail, leading to alternatives in terms of nonconflicting
	 * pathways. this function computes all potential paths */
	map<string, set<transition_p>> tm = natural_transitions_from(from);
	/** high priority/low level transitions to be applied first */
	/** map sorts entries by key, so this is automatically sorted by level */
	map<unsigned int, set<transition_p>> nt_level_map;

	for(auto& kv : tm){
	 	for(const transition_p t : kv.second){
	 		nt_level_map[t->priority].insert(t);
	 	}
	}

	list<transition_p> ntl;
	for(auto& ntp_kv : nt_level_map){
		for(auto& nt : ntp_kv.second){
			/** add these in some order to facilitate the recursion **/
			/* TODO what order */
			ntl.push_back(nt);
		}
	}

	return _recursive_expand_possible_transitions(from, ntl);
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

list<State> StateMachine::move(string input){
	list<State> passed_states;

	previous_state = current_state;
	map<string, set<transition_p>> cts = controlled_transitions_from(current_state);
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
	/* TODO this disregards probabilistic nature of transitions */
	list<transition_p> nts = nonconflicting_natural_transitions_from(current_state);
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

void StateMachine::on_enter(const State s, function<void(void)> cb){
	enter_callbacks[s].push_back(cb);
}

void StateMachine::on_remain(const State s, function<void(void)> cb){
	remain_callbacks[s].push_back(cb);
}

void StateMachine::on_exit(const State s, function<void(void)> cb){
	exit_callbacks[s].push_back(cb);
}

Path StateMachine::find_path(const State to){
	return StateModel::find_path(current_state, to);
}
