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

#define CATCH_CONFIG_MAIN

#include <cerebellum>
#include <catch.hpp>
#include <iostream>

using namespace std;
using namespace cerebellum;

TEST_CASE("Atomic state logic", "[AtomicState]"){
	SECTION("Atomic state sanity check"){
		astate_p a = AtomicState::create("a");
		REQUIRE( a == a );
		REQUIRE( !(a < a) );
	}

    SECTION("Atomic state labels need not be unique"){
    	astate_p a1 = AtomicState::create("a");
    	astate_p a2 = AtomicState::create("a");
    	REQUIRE( a1->label == a2->label );
    	REQUIRE( a1 != a2 );
    }
}

TEST_CASE("State logic", "[State]"){
	astate_p a1 = AtomicState::create("a1");
	astate_p a2 = AtomicState::create("a2");

	astate_p b1 = AtomicState::create("b1");

	SECTION("State sanity checks"){
		State x({a1, b1});
		REQUIRE( !x.empty() );
		REQUIRE( x.dimension() == 2 );
		REQUIRE( x == x );
		REQUIRE( !(x < x) );
		State y = x;
		REQUIRE( y == x );
		REQUIRE( !(y < x) );
		REQUIRE( !(x < y) );
	}

	SECTION("State comparisons"){
		State x({a1, b1});
		State y({a2, b1});
		REQUIRE( x != y );
		REQUIRE( y != x );
		// states are well-ordered
		REQUIRE( ((x < y) || (y < x)) );

		State z({b1, a1});
		// equality defined in terms of containing the same components
		REQUIRE( x == z );
		REQUIRE( !(x < z) );
		REQUIRE( !(z < x) );

		State w({a2});
		// subset relationships
		REQUIRE( w.dimension() == 1);
		REQUIRE( !(x.contains(w)) );
		REQUIRE( y.contains(w) );
		REQUIRE( y != w );
	}
}

TEST_CASE("Transition logic", "[Transition]"){
	astate_p a1 = AtomicState::create("a1");
	astate_p a2 = AtomicState::create("a2");

	astate_p b1 = AtomicState::create("b1");
	astate_p b2 = AtomicState::create("b2");

	transition_p a12 = Transition::create_natural("a12", a1, a2);
	transition_p a21 = Transition::create_natural("a21", a2, a1);
	a21->activate(b1);

	transition_p b12 = Transition::create_controlled("b12", b1, b2);
	transition_p b21 = Transition::create_controlled("b21", b2, b1);
	b21->inhibit(a2);

	transition_p d12 = Transition::create_natural("d12", State({a1, b1}), State({a2, b2}));
	transition_p x12 = Transition::create_natural("x12", State({a1, b2}), State({a2, b1}));
	d12->activate({a1, b1});
	x12->inhibit({a2, b1});

	SECTION("Transition sanity checks"){
		REQUIRE( a12 == a12 );
		REQUIRE( a12 != a21 );
		REQUIRE( !(a12 < a12) );
		// transitions are well-ordered
		REQUIRE( ((a12 < a21) || (a21 < a12)) );
	}

	State x({a1, b1});
	State y({a1, b2});
	State z({a2, b1});
	State w({a2, b2});

	SECTION("Transition logic check"){
		// a12 is natural and has not been activated
		REQUIRE( !x.accepts(a12) );
		// b12 is controlled
		REQUIRE( x.accepts(b12) );
		// a21 and b21 should not be available at x
		REQUIRE( !x.accepts(a21) );
		REQUIRE( !x.accepts(b21) );
		// d12 is activated and available at x
		REQUIRE( x.accepts(d12) );
		// x12 is not available at x
		REQUIRE( !x.accepts(x12) );

		REQUIRE( x.move(a12) == z );
		REQUIRE( x.move(b12) == y );
		REQUIRE( z.move_back(a12) == x );
		REQUIRE( y.move(a12) == w );
		REQUIRE( x.move(a21) == x );
		REQUIRE( y.move(x12) == z );
		REQUIRE( w.move_back(d12) == x );
	}

	SECTION("Transition conditions check"){
		REQUIRE( !a12->available_at(x) );
		REQUIRE( !a12->available_at(y) );
		REQUIRE( !a12->available_at(z) );
		REQUIRE( !a12->available_at(w) );

		REQUIRE( a21->available_at(x) );
		REQUIRE( !a21->available_at(y) );
		REQUIRE( a21->available_at(z) );
		REQUIRE( !a21->available_at(w) );

		REQUIRE( b12->available_at(x) );
		REQUIRE( b12->available_at(y) );
		REQUIRE( b12->available_at(z) );
		REQUIRE( b12->available_at(w) );

		REQUIRE( b21->available_at(x) );
		REQUIRE( b21->available_at(y) );
		REQUIRE( !b21->available_at(z) );
		REQUIRE( !b21->available_at(w) );

		REQUIRE( d12->available_at(x) );
		REQUIRE( !d12->available_at(y) );
		REQUIRE( !d12->available_at(z) );
		REQUIRE( !d12->available_at(w) );

		REQUIRE( x12->available_at(x) );
		REQUIRE( x12->available_at(y) );
		REQUIRE( !x12->available_at(z) );
		REQUIRE( x12->available_at(w) );
	}
}

TEST_CASE("State model logic", "[StateModel]"){
	// using example of elevator model
	// no data states

	// door states
	astate_p da = AtomicState::create("door_open");
	astate_p dc = AtomicState::create("door_closed");
	transition_p dadc = Transition::create_controlled("close_door", da, dc);
	transition_p dcda = Transition::create_controlled("open_door", dc, da);

	// floor states
	astate_p f1 = AtomicState::create("floor_1");
	astate_p f2 = AtomicState::create("floor_2");
	astate_p f3 = AtomicState::create("floor_3");
	transition_p f1f2 = Transition::create_natural("climbed_floor", f1, f2);
	transition_p f2f3 = Transition::create_natural("climbed_floor", f2, f3);
	transition_p f3f2 = Transition::create_natural("descended_floor", f3, f2);
	transition_p f2f1 = Transition::create_natural("descended_floor", f2, f1);

	// motion states
	astate_p mu = AtomicState::create("moving_up");
	astate_p ms = AtomicState::create("stopped");
	astate_p md = AtomicState::create("moving_down");
	transition_p msmu = Transition::create_controlled("start_climb", ms, mu);
	transition_p mums = Transition::create_controlled("stop_climb", mu, ms);
	transition_p msmd = Transition::create_controlled("start_descend", ms, md);
	transition_p mdms = Transition::create_controlled("stop_descend", md, ms);

	// transition conditions
	dcda->inhibit(mu);	// do not open door while moving
	dcda->inhibit(md);	// do not open door while moving
	msmu->inhibit(da);	// do not start moving while door open
	msmd->inhibit(da);  // do not start moving while door open

	f1f2->activate(mu);	// naturally climb when moving up
	f2f3->activate(mu); // naturally climb when moving up
	f3f2->activate(md); // naturally descend when moving down
	f2f1->activate(md);	// naturally descend when moving down

	StateModel model({dadc, dcda, f1f2, f2f3, f3f2, f2f1, msmu, mums, msmd, mdms});

	SECTION("State model Djikstra pathfinding sanity"){
		// shortest path from a state back to itself
		Path p = model.find_path({da, f1, ms}, {da, f1, ms});
		REQUIRE(p.inputs.empty());		// contains no inputs
		REQUIRE(p.transitions.empty());	// contains no transitions
		REQUIRE(p.states.size() == 1);	// contains only one state
		REQUIRE(*(p.states.begin()) == State({da, f1, ms}));	// which is itself
		REQUIRE(p.probability == 1);	// 100%

		// path to an impossible state
		p = model.find_path({da, f1, ms}, {da, f2, mu});
		REQUIRE(p.inputs.empty());		// contains no inputs
		REQUIRE(p.transitions.empty());	// contains no transitions
		REQUIRE(p.states.empty());		// contains no states
		REQUIRE(p.probability == 0);	// 0%
	}


	SECTION("State model Djikstra pathfinding"){
		// shortest path from one state to another
		Path p = model.find_path({da, f1, ms}, {f3, ms, da});
		REQUIRE(!p.inputs.empty());			// contains inputs
		REQUIRE(!p.transitions.empty());	// contains transitions
		REQUIRE(*(p.states.begin()) == State({da, ms, f1}));	// starts at the inital state
		REQUIRE(*(p.states.rbegin()) == State({da, f3, ms}));	// ends at the final state

		// this path should contain this exact sequence of inputs
		// close_door, start_climb, <wait>, stop_climb, open_door
		list<string> desired({"close_door", "start_climb", "", "stop_climb", "open_door"});
		auto j = desired.begin();
		for(auto i = p.inputs.begin(); i != p.inputs.end(); ++i){
			REQUIRE(*i == *j);
			j++;
		}
		REQUIRE(j == desired.end());

		// this path should contain this exact sequence of states
		// {da, f1, ms}, {dc, f1, ms}, {dc, f1, mu}, {dc, f2, mu}, {dc, f2, mu}, {dc, f3, mu}, {dc, f3, ms}, {da, f3, ms}
		// note {dc, f2, mu} is repeated due to having to wait for the natural transition
		list<State> ds({State({da, f1, ms}), State({dc, f1, ms}), State({dc, f1, mu}), State({dc, f2, mu}),
						State({dc, f2, mu}), State({dc, f3, mu}), State({dc, f3, ms}), State({da, f3, ms})});
		auto k = ds.begin();
		for(auto i = p.states.begin(); i != p.states.end(); ++i){
			REQUIRE(*i == *k);
			k++;
		}
		REQUIRE(k == ds.end());

		// this path should contain this exact sequence of transitions
		// dadc, msmu, f1f2, <wait>, f2f3, mums, dcda
		list<transition_p> dt({dadc, msmu, f1f2, f2f3, mums, dcda});
		auto l = dt.begin();
		for(auto i = p.transitions.begin(); i != p.transitions.end(); ++i){
			if((*i)->label == ""){
				// wait transition, do not increment l
			}else{
				REQUIRE(*i == *l);
				l++;
			}
		}
		REQUIRE(l == dt.end());
	}

	SECTION("State model DFS pathfinding"){
		// TODO
	}
}

TEST_CASE("Data state logic", "[DataState]"){
	SECTION("Data state sanity"){
		DataSource water = DataModel::create_source("water", 0, 10);
		dstate_p water1 = water->value_at(1);
		dstate_p water2 = water->value_at(2);
		dstate_p waters = water->value_lt(2);
		astate_p wc = water1;

		REQUIRE( water1->is_subset(water1) );
		REQUIRE( !water1->is_subset(water2) );
		REQUIRE( water1->is_subset(waters) );
		REQUIRE( !water2->is_subset(waters) );
		REQUIRE( waters->is_subset(waters) );
	}

	SECTION("Data state comparison"){
		DataSource fs = DataModel::create_source("fuel", 0, 10);
		astate_p f2 = fs->value_at(2);
		astate_p f3 = fs->value_at(3);
		astate_p fl3 = fs->value_lt(3);
		astate_p a = AtomicState::create("a");
		astate_p b = AtomicState::create("b");
		State A2({a, f2});
		State A3({a, f3});
		State A({a});
		State Al3({a, fl3});

		REQUIRE( A2.contains(A) );
		REQUIRE( A3.contains(A) );
		REQUIRE( Al3.contains(A) );
		REQUIRE( A2.contains(Al3) );
		REQUIRE( !A3.contains(Al3) );
		REQUIRE( !A3.contains(A2) );
		REQUIRE( !A2.contains(A3) );
		REQUIRE( !A.contains(A3) );
		REQUIRE( !Al3.contains(A3) );
		REQUIRE( !A.contains(A2) );
		REQUIRE( !Al3.contains(A2) );
	}

	SECTION("Data state transitions"){
		DataSource fs = DataModel::create_source("fuel", 0, 5);
		astate_p f0 = fs->value_at(0);
		astate_p a = AtomicState::create("a");
		astate_p b = AtomicState::create("b");
		transition_p fp = Transition::create_controlled("f+", fs->value_any(), fs->value_change(1));
		transition_p tab = Transition::create_controlled("a-b", {a, fs->value_any()}, {b, fs->value_change(-1)});
		tab->inhibit(fs->value_leq(0));

		State A({f0, a});

		// cannot move with no fuel
		REQUIRE(!A.accepts(tab));

		// can fuel up
		REQUIRE(A.accepts(fp));

		// can move once fueled up
		REQUIRE(A.move(fp).accepts(tab));

		// moving burns fuel, so cannot move again
		REQUIRE(!A.move(fp).move(tab).accepts(tab));

		State full = A.move(fp).move(fp).move(fp).move(fp).move(fp);

		// cannot fuel up past full
		REQUIRE(!full.accepts(fp));
		REQUIRE(full.move(fp) == full);
	}
}
