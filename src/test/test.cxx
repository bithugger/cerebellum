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

#include <cerebellum.h>
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
		REQUIRE( ((x < y) || (y < x)) );

		State z({b1, a1});
		REQUIRE( x == z );
		REQUIRE( !(x < z) );
		REQUIRE( !(z < x) );

		State w({a2});
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
		REQUIRE( ((a12 < a21) || (a21 < a12)) );
	}

	State x({a1, b1});
	State y({a1, b2});
	State z({a2, b1});
	State w({a2, b2});

	SECTION("Transition logic check"){
		REQUIRE( x.accepts(a12) );
		REQUIRE( x.accepts(b12) );
		REQUIRE( !x.accepts(a21) );
		REQUIRE( !x.accepts(b21) );
		REQUIRE( x.accepts(d12) );
		REQUIRE( !x.accepts(x12) );

		REQUIRE( x.move(a12) == z );
		REQUIRE( x.move(b12) == y );
		REQUIRE( z.move_back(a12) == x );
		REQUIRE( y.move(a12) == w );
		REQUIRE( x.move(a21).empty() );
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

	SECTION("Data state contains"){
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
}
