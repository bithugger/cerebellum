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

void print_path(list<string> path){
	size_t i = 0;
	for(auto it = path.begin(); it != path.end(); it++){
		cout << (*it);
		i++;
		if(i < path.size()){
			cout << ", ";
		}
	}
	cout << endl;
}

int main(){
	/** Example: elevator model control using cerebellum **/

	/* floor states, 3 floors */
	astate_p f1 = AtomicState::create("floor_1");
	astate_p f2 = AtomicState::create("floor_2");
	astate_p f3 = AtomicState::create("floor_3");

	/* motion state: moving up, stopped, moving down */
	astate_p mu = AtomicState::create("moving_up");
	astate_p ms = AtomicState::create("not_moving");
	astate_p md = AtomicState::create("moving_down");

	/* door state: closed, ajar/open */
	astate_p dc = AtomicState::create("door_closed");
	astate_p da = AtomicState::create("door_open");

	/* floor transition */
	transition_p f1f2 = Transition::create_natural("moved_from_floor_1_to_floor_2", f1, f2);
	f1f2->activate(mu);
	transition_p f2f1 = Transition::create_natural("moved_from_floor_2_to_floor_1", f2, f1);
	f2f1->activate(md);
	transition_p f2f3 = Transition::create_natural("moved_from_floor_2_to_floor_3", f2, f3);
	f2f3->activate(mu);
	transition_p f3f2 = Transition::create_natural("moved_from_floor_3_to_floor_2", f3, f2);
	f3f2->activate(md);

	/* motion transitions */
	transition_p msmu = Transition::create_controlled("start_ascending", ms, mu);
	msmu->inhibit(da);
	transition_p mums = Transition::create_controlled("stop_ascending", mu, ms);
	transition_p msmd = Transition::create_controlled("start_descending", ms, md);
	msmd->inhibit(da);
	transition_p mdms = Transition::create_controlled("stop_descending", md, ms);
	transition_p mumu = Transition::create_controlled("continue_ascending", mu, mu);
	transition_p mdmd = Transition::create_controlled("continue_descending", md, md);
	
	/* privileged motion transitions */
	transition_p msmup = Transition::create_controlled("start_ascending_privileged", ms, mu, 1);
	transition_p msmdp = Transition::create_controlled("stop_ascending_privileged", ms, md, 1);

	/* door transitions */
	transition_p dcda = Transition::create_controlled("open_door", dc, da);
	dcda->inhibit(mu);
	dcda->inhibit(md);
	transition_p dadc = Transition::create_controlled("close_door", da, dc);
	dadc->inhibit(mu);
	dadc->inhibit(md);

	/** state model with all atomic states and transitions **/
	StateModel model({f1, f2, f3, mu, ms, md, dc, da}, 
		{f1f2, f2f1, f2f3, f3f2, msmu, mums, msmd, mdms, mumu, mdmd, dcda, dadc,
		 msmup, msmdp});


	/** test the path finding algorithm **/
	list<string> path;

	/* find a path from state floor 1, stopped, door open, to
	 * state floor 3, stopped, door open */
	/* privilege level 0 */
	path = model.find_path(State({f1, ms, da}), State({f3, ms, da}), 0);
	/* print */
	cout << "Case 1 : ";
	print_path(path);

	/* find a path from state floor 1, stopped, door open, to
	 * state floor 2, moving up, door open */
	/* privilege level 0, no path should exist */
	path = model.find_path(State({f1, ms, da}), State({f2, mu, da}), 0);
	/* print */
	cout << "Case 2 : ";
	print_path(path);

	/* find a path from state floor 1, stopped, door open, to
	 * state floor 3, stopped, door open */
	/* privilege level unlimited */
	path = model.find_path(State({f1, ms, da}), State({f3, ms, da}));
	/* print */
	cout << "Case 3 : ";
	print_path(path);

	/* find a path from state floor 1, stopped, door open, to
	 * state floor 2, moving up, door open */
	/* privilege level unlimited */
	path = model.find_path(State({f1, ms, da}), State({f2, mu, da}));
	/* print */
	cout << "Case 4 : ";
	print_path(path);

	/* find a path from state floor 3, stopped, door open, to
	 * state set floor 1, stopped */
	/* privilege level 0 */
	path = model.find_path(State({f3, ms, da}), State({f1, ms}), 0);
	/* print */
	cout << "Case 5 : ";
	print_path(path);

	/* find a path from state floor 3, stopped, door open, to
	 * state set floor 1 */
	/* privilege level 0 */
	path = model.find_path(State({f3, ms, da}), State({f1}), 0);
	/* print */
	cout << "Case 6 : ";
	print_path(path);

	/** test an instance of this model as a state machine **/
	/* initial state floor 1, stopped, door open */
	StateMachine sm(model, State({f1, ms, da}));
	/* find path to state floor 3, stopped, door open */
	path = model.find_path(sm.get_state(), State({f3, ms, da}), 0);
	/* apply the inputs to the state machine, print all intermediate states */
	cout << "Case 1 state flow :" << endl;
	for(const string& s : path){
		cout << sm.get_state() << ": ";
		cout << s << " --> ";
		list<State> xs = sm.move(s);
		for(const State& x : xs){
			cout << x << " ";
		}
		cout << endl;
	}

    return 0;
}