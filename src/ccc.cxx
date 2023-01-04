#include <iostream>
#include <list>
#include <cerebellum>

using namespace std;
using namespace cerebellum;

int main(){
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
	astate_p xf = AtomicState::create("at_fuel"); // refuel station

	/* navigation state */
	astate_p nh = AtomicState::create("to_home");
	astate_p nd = AtomicState::create("to_destination");
	astate_p na = AtomicState::create("to_alternate");
	astate_p nf = AtomicState::create("to_fuel");

	/** Transitions **/
	/* flying states */
	transition_p land = Transition::create_controlled("land", af, ag);
	land->inhibit(x1); // cannot land at waypoints
	land->inhibit(x2); // unless in an emergency, requiring elevated privilege
	land->inhibit(x3);

	transition_p takeoff = Transition::create_controlled("takeoff", ag, af);
	takeoff->inhibit(fuel->value_leq(0)); // cannot takeoff without fuel

	transition_p crash = Transition::create_natural("crash", af, ac);
	crash->activate(fuel->value_leq(0)); // automatically crash when out of fuel

	/* fuel states */
	transition_p fuel_burn = Transition::create_natural("fuel_burnt", fuel->value_any(), fuel->value_change(-1), 10);
	fuel_burn->activate(af); // fuel consumed while flying

	transition_p refuel = Transition::create_controlled("refuel", fuel->value_any(), fuel->value_at(5));
	refuel->activate({ag, xf});

	/* natural location transitions */
	/* map like so
	 *          F
	 *          |
	 *  H - 1 - 2 - 3 - D
	 *          |
	 *          A
	 */
	transition_p xh_x1 = Transition::create_natural("moved_from_home_to_wp1", xh, x1);
	transition_p x1_xh = Transition::create_natural("moved_from_wp1_to_home", x1, xh);
	xh_x1->activate({nd, af});
	xh_x1->activate({na, af});
	xh_x1->activate({nf, af});
	x1_xh->activate({nh, af});

	transition_p x1_x2 = Transition::create_natural("moved_from_wp1_to_wp2", x1, x2);
	transition_p x2_x1 = Transition::create_natural("moved_from_wp2_to_wp1", x2, x1);
	x1_x2->activate({nd, af});
	x1_x2->activate({na, af});
	x1_x2->activate({nf, af});
	x2_x1->activate({nh, af});

	transition_p x2_xa = Transition::create_natural("moved_from_wp2_to_alt", x2, xa);
	transition_p xa_x2 = Transition::create_natural("moved_from_alt_to_wp2", xa, x2);
	x2_xa->activate({na, af});
	xa_x2->activate({nh, af});
	xa_x2->activate({nd, af});
	xa_x2->activate({nf, af});

	transition_p x2_x3 = Transition::create_natural("moved_from_wp2_to_wp3", x2, x3);
	transition_p x3_x2 = Transition::create_natural("moved_from_wp3_to_wp2", x3, x2);
	x2_x3->activate({nd, af});
	x3_x2->activate({na, af});
	x3_x2->activate({nh, af});
	x3_x2->activate({nf, af});

	transition_p x2_xf = Transition::create_natural("moved_from_wp2_to_fs", x2, xf);
	transition_p xf_x2 = Transition::create_natural("moved_from_fs_to_wp2", xf, x2);
	x2_xf->activate({nf, af});
	xf_x2->activate({na, af});
	xf_x2->activate({nh, af});
	xf_x2->activate({nd, af});

	transition_p x3_xd = Transition::create_natural("moved_from_wp3_to_dest", x3, xd);
	transition_p xd_x3 = Transition::create_natural("moved_from_dest_to_wp3", xd, x3);
	x3_xd->activate({nd, af});
	xd_x3->activate({na, af});
	xd_x3->activate({nh, af});
	xd_x3->activate({nf, af});

	/* navigation transitions */
	transition_p nd_nh = Transition::create_controlled("to_home", nd, nh, 2); // from dest
	transition_p na_nh = Transition::create_controlled("to_home", na, nh, 2); // from alt
	transition_p nf_nh = Transition::create_controlled("to_home", nf, nh, 2); // from fuel

	transition_p nh_nd = Transition::create_controlled("to_dest", nh, nd, 2); // from home
	transition_p na_nd = Transition::create_controlled("to_dest", na, nd, 2); // from alt
	transition_p nf_nd = Transition::create_controlled("to_dest", nf, nd, 2); // from fuel

	transition_p nh_na = Transition::create_controlled("to_alt", nh, na, 2); // from home
	transition_p nd_na = Transition::create_controlled("to_alt", nd, na, 2); // from dest
	transition_p nf_na = Transition::create_controlled("to_alt", nf, na, 2); // from fuel

	transition_p nh_nf = Transition::create_controlled("to_fuel", nh, nf, 2); // from home
	transition_p nd_nf = Transition::create_controlled("to_fuel", nd, nf, 2); // from dest
	transition_p na_nf = Transition::create_controlled("to_fuel", na, nf, 2); // from alt
	

	vector<transition_p> all_transitions({land, takeoff, crash, fuel_burn, refuel,
		xh_x1, x1_xh, x1_x2, x2_x1, x2_xa, xa_x2, x2_x3, x3_x2, x3_xd, xd_x3, x2_xf, xf_x2,
		nd_nh, nh_nd, nf_nh, na_nh, nh_na, nf_nd, na_nd, nd_na, nf_na, nh_nf, nd_nf, na_nf});

	StateModel model(all_transitions);

	/** test an instance of this model as a state machine **/
	StateMachine uav(model, State({ag, nh, xh, fuel->value_at(4)}));
	uav.on_enter(State({ag, xd}), [](){ cout << "GOAL!" << endl; exit(0); });
    uav.on_enter(State({ac}), [](){ cout << "GAME OVER" << endl; exit(0); });
	cout << "Time to play a game" << endl;
    cout << uav.get_state() << endl << "> ";
    string input;
	while(getline(cin, input)){
		list<State> xs = uav.move(input);
		for(const State& x : xs){
			cout << x << endl;
		}
		cout << endl << "> ";
	}

    return 0;
}