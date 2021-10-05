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
#include <iostream>
#include <list>

using namespace cerebellum;
using namespace std;

void print_path(Path path){
	size_t i = 0;
	for(auto it = path.transitions.begin(); it != path.transitions.end(); it++){
		transition_p t = *it;
		if(t->type == Transition::CONTROLLED){
			cout << "<" << t->label << ">";
		}else if(t->type == Transition::NATURAL){
			cout << "[" << t->label << "]";
		}else{
			cout << t->label;
		}

		cout << " ";
		i++;
	}
	// cout << " :: probability = " << path.probability << ", cost = " << path.cost;
}

void print_pathway(PathWay pathway){
	size_t i = 0;
	auto cyclesit = pathway.cycles.begin();
	for(auto it = pathway.transitions.begin(); it != pathway.transitions.end(); it++){

		if(!cyclesit->empty()){
			cout << " { ";
			for(Path cycle : *cyclesit){
				cout << "( ";
				print_path(cycle);
				cout << ") ";
			}
			cout << "}";
		}
		cout << " ";

		transition_p t = *it;
		if(t->type == Transition::CONTROLLED){
			cout << "<" << t->label << ">";
		}else if(t->type == Transition::NATURAL){
			cout << "[" << t->label << "]";
		}else{
			cout << t->label;
		}

		cyclesit++;
		i++;
	}
	// cout << " :: probability = " << pathway.probability << ", cost = " << pathway.cost;
}

/** Example 0: a toy example **/
void example0(){
	cout << "Toy example:" << endl;

	astate_p a = AtomicState::create("a");
	astate_p b = AtomicState::create("b");
	astate_p c = AtomicState::create("c");
	astate_p d = AtomicState::create("d");

	transition_p tab = Transition::create_controlled("a_b", a, b);
	transition_p tbc = Transition::create_controlled("b_c", b, c);
	transition_p tca = Transition::create_controlled("c_a", c, a);
	transition_p tac = Transition::create_controlled("a_c", a, c);
	transition_p tbd = Transition::create_controlled("b_d", b, d);
	transition_p tcd = Transition::create_controlled("c_d", c, d);

	StateModel m({tab, tbc, tca, tac, tbd, tcd});

	vector<Path> ps = m.find_all_paths(a, d);
	for(Path p : ps){
		print_path(p);
		cout << endl;
	}

	vector<PathWay> pws = m.find_all_pathways(a, d);
	for(PathWay pw : pws){
		print_pathway(pw);
		cout << endl;
	}
	cout << endl;
}

/** Example 1: another toy example **/
void example1(){
	cout << "Another toy example:" << endl;

	astate_p a = AtomicState::create("a");
	astate_p b = AtomicState::create("b");
	astate_p c = AtomicState::create("c");
	astate_p d = AtomicState::create("d");

	transition_p tab = Transition::create_controlled("a_b", a, b);
	transition_p tba = Transition::create_controlled("b_a", b, a);
	tba->inhibit(d);
	transition_p tcd = Transition::create_natural("c_d", c, d, 0.5);
	tcd->activate(c);

	StateModel m({tab, tba, tcd});

	vector<Path> ps = m.find_all_paths({a, c}, {d});
	for(Path p : ps){
		print_path(p);
		cout << "\t" << p.probability << endl;
	}

	vector<PathWay> pws = m.find_all_pathways({a, c}, d);
	for(PathWay pw : pws){
		print_pathway(pw);
		cout << "\t" << pw.probability_arrival_eventually() << endl;
	}
	cout << endl;
}

/** Example 1: elevator model control **/
void elevator_model(){
	cout << "Elevator model control:" << endl;

	/* floor states, 3 floors */
	DataSource floors = DataModel::create_source("floor", 1, 3);

	/* motion state: moving up, stopped, moving down */
	astate_p mu = AtomicState::create("moving_up");
	astate_p ms = AtomicState::create("not_moving");
	astate_p md = AtomicState::create("moving_down");

	/* door state: closed, ajar/open */
	astate_p dc = AtomicState::create("door_closed");
	astate_p da = AtomicState::create("door_open");

	/* floor transition */
	transition_p climbed = Transition::create_natural("climbed_floor", floors->value_any(), floors->value_change(1));
	climbed->activate(mu);
	transition_p descended = Transition::create_natural("descended_floor", floors->value_any(), floors->value_change(-1));
	descended->activate(md);

	/* motion transitions */
	transition_p msmu = Transition::create_controlled("start_ascending", ms, mu);
	msmu->inhibit(da);
	msmu->inhibit(floors->value_geq(3));
	transition_p mums = Transition::create_controlled("stop", mu, ms);
	transition_p msmd = Transition::create_controlled("start_descending", ms, md);
	msmd->inhibit(da);
	msmd->inhibit(floors->value_leq(1));
	transition_p mdms = Transition::create_controlled("stop", md, ms);
	
	/* privileged motion transitions */
	transition_p msmup = Transition::create_controlled("start_ascending_privileged", ms, mu, 5, 1);
	msmup->inhibit(floors->value_geq(3));
	transition_p msmdp = Transition::create_controlled("start_descending_privileged", ms, md, 5, 1);
	msmdp->inhibit(floors->value_leq(1));

	/* door transitions */
	transition_p dcda = Transition::create_controlled("open_door", dc, da);
	dcda->inhibit(mu);
	dcda->inhibit(md);
	transition_p dadc = Transition::create_controlled("close_door", da, dc);
	dadc->inhibit(mu);
	dadc->inhibit(md);

	/** state model with all atomic states and transitions **/
	StateModel model({climbed, descended, msmu, mums, msmd, mdms, 
		/*mumu, mdmd, */dcda, dadc, msmup, msmdp});

	/** test the path finding algorithm **/

	/* find a path from state floor 1, stopped, door open, to
	 * state floor 3, stopped, door open */
	/* privilege level 0 */
	Path path = model.find_path(State({floors->value_at(1), ms, da}), State({floors->value_at(3), ms, da}), 0);
	/* print */
	cout << "Case 1 : ";
	print_path(path);
	cout << endl;

	/* find a path from state floor 1, stopped, door open, to
	 * state floor 2, moving up, door open */
	/* privilege level 0, no path should exist */
	path = model.find_path(State({floors->value_at(1), ms, da}), State({floors->value_at(2), mu, da}), 0);
	/* print */
	cout << "Case 2 : ";
	print_path(path);
	cout << endl;

	/* find a path from state floor 1, stopped, door open, to
	 * state floor 3, stopped, door open */
	/* privilege level unlimited */
	path = model.find_path(State({floors->value_at(1), ms, da}), State({floors->value_at(3), ms, da}), 0);
	/* print */
	cout << "Case 3 : ";
	print_path(path);
	cout << endl;

	/* find a path from state floor 1, stopped, door open, to
	 * state floor 2, moving up, door open */
	/* privilege level unlimited */
	path = model.find_path(State({floors->value_at(1), ms, da}), State({floors->value_at(2), mu, da}));
	/* print */
	cout << "Case 4 : ";
	print_path(path);
	cout << endl;

	/* find a path from state floor 3, stopped, door open, to
	 * state set floor 1, stopped */
	/* privilege level 0 */
	path = model.find_path(State({floors->value_at(3), ms, da}), State({floors->value_at(1), ms}), 0);
	/* print */
	cout << "Case 5 : ";
	print_path(path);
	cout << endl;

	/* find a path from state floor 3, stopped, door open, to
	 * state set floor 1 */
	/* privilege level 0 */
	path = model.find_path(State({floors->value_at(3), ms, da}), State({floors->value_at(1)}), 0);
	/* print */
	cout << "Case 6 : ";
	print_path(path);
	cout << endl;

	/** test an instance of this model as a state machine **/
	/* initial state floor 1, stopped, door open */
	StateMachine sm(model, State({floors->value_at(1), ms, da}));
	/* find path to state floor 3, stopped, door open */
	path = sm.find_path(State({floors->value_at(3), ms, da}), 0);
	/* apply the inputs to the state machine, print all intermediate states */
	cout << "Case 1 state flow :" << endl;
	for(const string& s : path.inputs){
		cout << sm.get_state() << ": ";
		cout << s << " --> ";
		list<State> xs = sm.move(s);
		for(const State& x : xs){
			cout << x << " ";
		}
		cout << endl;
	}

	/** find all possible non-looping paths from one state to another **/
	vector<Path> all_paths = model.find_all_paths(State({floors->value_at(1), ms, da}), State({floors->value_at(3), ms, da}));
	cout << endl << "\tall paths" << endl;
	for(auto p : all_paths){
		print_path(p);
		cout << endl;
	}
	cout << endl;
}

/** Example 2: UAV flight director **/
void uav_model(){
	cout << "UAV flight director:" << endl;

	/** State definitions **/
	/* flying states */
	astate_p af = AtomicState::create("flying");
	astate_p ag = AtomicState::create("landed");
	astate_p ac = AtomicState::create("crashed");

	/* fuel state */
	DataSource fuel = DataModel::create_source("fuel", 0, 5);

	/* location state */
	astate_p xh = AtomicState::create("at_home"); // home
	astate_p x1 = AtomicState::create("at_wp1"); // waypoint 1
	astate_p x2 = AtomicState::create("at_wp2"); // waypoint 2
	astate_p x3 = AtomicState::create("at_wp3"); // waypoint 3
	astate_p xa = AtomicState::create("at_alt"); // alternate a
	astate_p xd = AtomicState::create("at_dest"); // destination

	/* navigation state */
	astate_p nh = AtomicState::create("to_home");
	astate_p nd = AtomicState::create("to_destination");
	astate_p na = AtomicState::create("to_alternate");

	/** Transitions **/
	/* flying states */
	transition_p land = Transition::create_controlled("land", af, ag);
	land->inhibit(x1); // cannot land at waypoints
	land->inhibit(x2); // unless in an emergency, requiring elevated privilege
	land->inhibit(x3);
	transition_p eland = Transition::create_controlled("emergency_land", af, ag, 30, 1);

	transition_p takeoff = Transition::create_controlled("takeoff", ag, af);
	takeoff->inhibit(fuel->value_leq(0)); // cannot takeoff without fuel

	transition_p crash = Transition::create_natural("crash", af, ac);
	crash->activate(fuel->value_leq(0)); // automatically crash when out of fuel

	/* fuel states */
	transition_p fuel_burn = Transition::create_natural("fuel_burnt", fuel->value_any(), fuel->value_change(-1), 10);
	fuel_burn->activate(af); // fuel consumed while flying

	/* natural location transitions */
	/* map like so
	 *
	 *  H - 1 - 2 - 3 - D
	 *          |
	 *          A
	 */
	transition_p xh_x1 = Transition::create_natural("moved_from_home_to_wp1", xh, x1);
	transition_p x1_xh = Transition::create_natural("moved_from_wp1_to_home", x1, xh);
	xh_x1->activate({nd, af});
	xh_x1->activate({na, af});
	x1_xh->activate({nh, af});

	transition_p x1_x2 = Transition::create_natural("moved_from_wp1_to_wp2", x1, x2);
	transition_p x2_x1 = Transition::create_natural("moved_from_wp2_to_wp1", x2, x1);
	x1_x2->activate({nd, af});
	x1_x2->activate({na, af});
	x2_x1->activate({nh, af});

	transition_p x2_xa = Transition::create_natural("moved_from_wp2_to_alt", x2, xa);
	transition_p xa_x2 = Transition::create_natural("moved_from_alt_to_wp2", xa, x2);
	x2_xa->activate({na, af});
	xa_x2->activate({nh, af});
	xa_x2->activate({nd, af});

	transition_p x2_x3 = Transition::create_natural("moved_from_wp2_to_wp3", x2, x3);
	transition_p x3_x2 = Transition::create_natural("moved_from_wp3_to_wp2", x3, x2);
	x2_x3->activate({nd, af});
	x3_x2->activate({na, af});
	x3_x2->activate({nh, af});

	transition_p x3_xd = Transition::create_natural("moved_from_wp3_to_dest", x3, xd);
	transition_p xd_x3 = Transition::create_natural("moved_from_dest_to_wp3", xd, x3);
	x3_xd->activate({nd, af});
	xd_x3->activate({na, af});
	xd_x3->activate({nh, af});

	/* navigation transitions */
	transition_p nd_nh = Transition::create_controlled("to_home", nd, nh, 2); // from dest
	transition_p na_nh = Transition::create_controlled("to_home", na, nh, 2); // from alt

	transition_p nh_nd = Transition::create_controlled("to_dest", nh, nd, 2); // from home
	transition_p na_nd = Transition::create_controlled("to_dest", na, nd, 2); // from alt

	transition_p nh_na = Transition::create_controlled("to_alt", nh, na, 2); // from home
	transition_p nd_na = Transition::create_controlled("to_alt", nd, na, 2); // from dest

	vector<transition_p> all_transitions({land, takeoff, eland, crash, fuel_burn,
		xh_x1, x1_xh, x1_x2, x2_x1, x2_xa, xa_x2, x2_x3, x3_x2, x3_xd, xd_x3, 
		nd_nh, nh_nd, na_nh, nh_na, na_nd, nd_na});

	StateModel model(all_transitions);

	cout << "Case 1 : ";
	print_path(model.find_path({ag, na, x2, fuel->value_at(2)}, {ag, xd}, 0));
	cout << endl;

	cout << "Case 2 : ";
	print_path(model.find_path({ag, nh, xh, fuel->value_at(4)}, {ag, xd}, 0));
	cout << endl;

	cout << "Case 3 : ";
	print_path(model.find_path({af, nd, x1, fuel->value_at(1)}, {ag}, 0));
	cout << endl;

	cout << "Case 4 : ";
	print_path(model.find_path({af, nd, x2, fuel->value_at(1)}, {ag}, 0));
	cout << endl;

	cout << "Case 5 : ";
	print_path(model.find_path({af, nd, x3, fuel->value_at(1)}, {ag}, 0));
	cout << endl;

	cout << "Case 6 : ";
	print_path(model.find_path({af, nd, x1, fuel->value_at(0)}, {ag}));
	cout << endl;

	cout << "Case 7 : ";
	print_path(model.find_path_around({af, nd, x2, fuel->value_at(4)}, {ag}, {{x3}, {xa}}));
	cout << endl;

	/** test an instance of this model as a state machine **/
	StateMachine uav(model, State({ag, nh, xh, fuel->value_at(4)}));
	uav.on_enter(State({ag, xd}), [](){ cout << "GOAL!" << endl; });
	Path path = uav.find_path(State({ag, xd}), 0);
	cout << "Case 2 state flow :" << endl;
	for(const string& s : path.inputs){
		cout << uav.get_state() << ": ";
		cout << s << " --> ";
		list<State> xs = uav.move(s);
		for(const State& x : xs){
			cout << x << " ";
		}
		cout << endl;
	}

	cout << endl;
}

void prob_test(){
	cout << "Probability test:" << endl;

	astate_p a = AtomicState::create("a");
	astate_p b = AtomicState::create("b");
	astate_p c = AtomicState::create("c");

	astate_p w = AtomicState::create("working");
	astate_p k = AtomicState::create("broken");

	astate_p s = AtomicState::create("stopped");
	astate_p m = AtomicState::create("moving");

	transition_p tab = Transition::create_natural("a_to_b", a, b);
	tab->activate({m, w});
	transition_p tbc = Transition::create_natural("b_to_c", b, c);
	tbc->activate({m, w});

	transition_p twk = Transition::create_natural("breakdown", w, k, 0.3);
	twk->activate(m);

	transition_p tsm = Transition::create_controlled("move", s, m);
	transition_p tms = Transition::create_controlled("stop", m, s);

	StateModel model({tab, tbc, twk, tsm, tms});

	Path p = model.find_path(State({a, s, w}), State(c));
	cout << "Djikstra path-finding ignores failures and returns nothing" << endl;
	print_path(p);
	cout << endl;

	vector<Path> pv = model.find_all_paths(State({a, s, w}), State(c));
	cout << "DFS path-finding does account for failures and probabilities" << endl;
	for(Path p : pv){
		print_path(p);
		cout << " - " << p.probability*100 << "% chance of success" << endl;
	}

	cout << endl;
}

void b3_test(){
	cout << "B3 test:" << endl;

	astate_p a = AtomicState::create("a");
	astate_p b = AtomicState::create("b");
	astate_p c = AtomicState::create("c");

	astate_p w = AtomicState::create("working");
	astate_p k = AtomicState::create("broken");

	astate_p s = AtomicState::create("stopped");
	astate_p f = AtomicState::create("fast");
	astate_p m = AtomicState::create("slow");

	transition_p tab = Transition::create_natural("a_to_b", a, b, 1.0, 3.0);
	tab->activate({m, w});

	transition_p tbc = Transition::create_natural("b_to_c", b, c, 1.0, 3.0);
	tbc->activate({m, w});

	transition_p tabf = Transition::create_natural("a_to_b_fast", a, b, 1.0, 1.0);
	tabf->activate({f, w});

	transition_p tbcf = Transition::create_natural("b_to_c_fast", b, c, 1.0, 1.0);
	tbcf->activate({f, w});

	transition_p twk = Transition::create_natural("breakdown_slow", w, k, 0.1, 1.0, 1);
	twk->activate(m);

	transition_p twkf = Transition::create_natural("breakdown_fast", w, k, 0.3, 1.0, 1);
	twkf->activate(f);

	transition_p tsm = Transition::create_controlled("move_slow", s, m);
	transition_p tms = Transition::create_controlled("stop_slow", m, s);
	transition_p tsf = Transition::create_controlled("move_fast", s, f);
	transition_p tfs = Transition::create_controlled("stop_fast", f, s);

	StateModel model({tab, tbc, twk, tsm, tms, tabf, tbcf, tsf, tfs, twkf});

	Path p = model.find_best_path_over_likelihood(State({a, s, w}), State(c), 0.0);
	cout << "B3 path-finding lowest cost with over 0% likelihood" << endl;
	print_path(p);
	cout << " - " << p.probability*100 << "% chance of success, cost " << p.cost << endl;
	cout << endl;

	p = model.find_best_path_over_likelihood(State({a, s, w}), State(c), 0.5);
	cout << "B3 path-finding lowest cost with over 50% likelihood" << endl;
	print_path(p);
	cout << " - " << p.probability*100 << "% chance of success, cost " << p.cost << endl;
	cout << endl;

	p = model.find_likeliest_path_under_cost(State({a, s, w}), State(c), 10);
	cout << "B3 path-finding most likely with under 10 cost" << endl;
	print_path(p);
	cout << " - " << p.probability*100 << "% chance of success, cost " << p.cost << endl;
	cout << endl;

	p = model.find_likeliest_path_under_cost(State({a, s, w}), State(c), 5);
	cout << "B3 path-finding most likely with under 5 cost" << endl;
	print_path(p);
	cout << " - " << p.probability*100 << "% chance of success, cost " << p.cost << endl;
	cout << endl;
}

int main(){

	example0();

	example1();

	elevator_model();

	uav_model();

	prob_test();

	b3_test();

    return 0;
}