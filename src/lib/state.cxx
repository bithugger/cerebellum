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

AtomicState::AtomicState(string label, string output) :
label(label),
output(output)
{

}

astate_p AtomicState::create(string s){
	return astate_p(new AtomicState(s, s));
}

AtomicStateType AtomicState::type() const {
	return AtomicStateType::PURE;
}

ostream& cerebellum::operator<<(ostream& os, const AtomicState& a){
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

State::State(vector<astate_p> components) :
components(components)
{

}

State::State(initializer_list<astate_p> components) :
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
		return find_if(components.begin(), components.end(), [&](const astate_p y){
			if(y->type() == AtomicStateType::INT_DATA){
				DataState& xx = static_cast<DataState&>(*x);
				DataState& yy = static_cast<DataState&>(*y); 
				return yy.is_subset(xx);
			}else{
				return false;
			}
		}) != components.end();
	}else{
		return find(components.begin(), components.end(), x) != components.end();		
	}
}

bool State::contains(vector<astate_p> X) const {
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
		vector<astate_p> a = components;
		vector<astate_p> b = o.components;
		sort(a.begin(), a.end());
		sort(b.begin(), b.end());
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
		vector<pair<astate_p, astate_p>> changes;
		for(int i = 0; i < t->from.dimension(); i++){
			changes.push_back(make_pair(t->from.components[i], t->to.components[i]));
		}

		vector<astate_p> new_components = components;
		for(auto ft : changes){
			if(ft.first->type() == AtomicStateType::INT_DATA){
				DataState& xx = static_cast<DataState&>(*ft.first);
				auto it = find_if(new_components.begin(), new_components.end(), [&](const astate_p y){
					if(y->type() == AtomicStateType::INT_DATA){
						DataState& yy = static_cast<DataState&>(*y); 
						return yy.is_subset(xx);
					}else{
						return false;
					}
				});
				DataState& yy = static_cast<DataState&>(*(*it));
				DataState& zz = static_cast<DataState&>(*ft.second);
				int v = yy.value;
				if(zz.qualifier == DataState::NONE){
					v = yy.value + zz.value;
				}else if(zz.qualifier == DataState::EQUALS){
					v = zz.value;
				}
				v = v < yy.source->lower_bound ? yy.source->lower_bound : v;
				v = v > yy.source->upper_bound ? yy.source->upper_bound : v;
				*it = yy.source->value_at(v);
			}else{
				auto it = find(new_components.begin(), new_components.end(), ft.first);
				*it = ft.second;	
			}
		}
		return State(new_components);
	}else{
		return State(components);
	}
}

State State::move_back(const transition_p t) const {
	if(contains(t->to)){
		vector<pair<astate_p, astate_p>> changes;
		for(int i = 0; i < t->from.dimension(); i++){
			changes.push_back(make_pair(t->from.components[i], t->to.components[i]));
		}

		vector<astate_p> new_components = components;
		for(auto ft : changes){
			if(ft.first->type() == AtomicStateType::INT_DATA){
				DataState& xx = static_cast<DataState&>(*ft.first);
				auto it = find_if(new_components.begin(), new_components.end(), [&](const astate_p y){
					if(y->type() == AtomicStateType::INT_DATA){
						DataState& yy = static_cast<DataState&>(*y); 
						return yy.is_subset(xx);
					}else{
						return false;
					}
				});
				DataState& yy = static_cast<DataState&>(*(*it));
				DataState& zz = static_cast<DataState&>(*ft.second);
				int v = zz.value;
				if(zz.qualifier == DataState::NONE){
					v = yy.value - zz.value;
				}else if(zz.qualifier == DataState::EQUALS){
					v = yy.value;
				}
				v = v < yy.source->lower_bound ? yy.source->lower_bound : v;
				v = v > yy.source->upper_bound ? yy.source->upper_bound : v;
				*it = yy.source->value_at(v);
			}else{
				auto it = find(new_components.begin(), new_components.end(), ft.second);
				*it = ft.first;
			}
		}
		return State(new_components);
	}else{
		return State(components);
	}
}

void State::extend(astate_p a){
	components.push_back(a);
}

void State::extend(vector<astate_p> as){
	for(const astate_p a : as){
		extend(a);
	}
}

void State::extend(initializer_list<astate_p> as){
	for(const astate_p a : as){
		extend(a);
	}
}

ostream& cerebellum::operator<<(ostream& os, const State& s) {
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

vector<string> qualifier_names({"=", "!=", "<", "<=", ">", ">=", "*"});

DataState::DataState(DataSource source, DataState::Qualifier qualifier, int value) :
AtomicState("" + source->name + " " + qualifier_names[qualifier] + " " + to_string(value), 
			"" + source->name + " " + qualifier_names[qualifier] + " " + to_string(value)),
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

DataModel::DataModel(string name, int lb, int ub) :
name(name),
lower_bound(lb),
upper_bound(ub)
{

}

DataSource DataModel::create_source(string name){
	return DataSource(new DataModel(name, numeric_limits<int>::min(), numeric_limits<int>::max()));
}

DataSource DataModel::create_source(string name, int lb, int ub){
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

bool cerebellum::operator==(const astate_p& x, const astate_p& y){
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

bool cerebellum::operator<(const astate_p& x, const astate_p& y){
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