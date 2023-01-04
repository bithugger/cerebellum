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

#include <cerebellum>
#include <algorithm>
#include <sstream>
#include <utility>
#include <vector>
#include <string>
#include <map>

using namespace cerebellum;
using namespace std;

/** trim whitespaces from beginning and end */
string trim_whitespaces(const string& str){
	size_t first = str.find_first_not_of(' ');
	if (string::npos == first){
		return str;
	}
	size_t last = str.find_last_not_of(' ');
	return str.substr(first, (last - first + 1));
}

/** trim whitespaces from beginning and end, as well as remove repeated
 * whitespaces that appear in the middle
 * as well as replace whitespaces that appear within brackets () */
string collapse_whitespaces(const string& str){
	string trimmed = trim_whitespaces(str);
	string result;
	unique_copy(trimmed.begin(), trimmed.end(), back_insert_iterator<string>(result), 
		[](char a, char b){
			return isspace(a) && isspace(b);
		}
	);
	
	int x = 0;
	size_t pos = result.find_first_of("( )");
	while(pos != string::npos){
		char c = result[pos];
		if(c == '('){
			x++;
		}else if(c == ')'){
			x--;
		}else if(x > 0){
			// c == ' ' is implied
			// found a space inside a bracket
			// replace with comma
			result[pos] = ',';
		}
		pos = result.find_first_of("( )", pos+1);
	}

	return result;
}

vector<string> split_by_delimiter(const string& str, const string delim){
	vector<string> splits;
	auto start = 0U;
	auto end = str.find(delim);
	while(end != string::npos){
		string sub = str.substr(start, end - start);
		if(sub.size() > 0){
			splits.push_back(sub);
		}
		start = end + delim.length();
		end = str.find(delim, start);
	}
	splits.push_back(str.substr(start, end));
	return splits;
}

bool equals_ignore_case(const string& a, const string& b){
	size_t sz = a.size();
	if (b.size() != sz){
		return false;
	}

	for (size_t i = 0; i < sz; i++){
		if (tolower(a[i]) != tolower(b[i])){
			return false;
		}
	}

	return true;
}

StateModel cerebellum::parse_model(vector<string> lines){
	map<string, astate_p> states_map;
	map<string, DataSource> source_map;
	vector<transition_p> transitions;
	unsigned int anonymous_transition_count = 0;
	unsigned int line_count = 0;

	for(const string& raw_line : lines){
		line_count++;

		// trim whitespaces
		string line = collapse_whitespaces(raw_line);
		if(line.size() == 0){
			continue;
		}
		
		vector<string> tokens = split_by_delimiter(line, " ");
		// comments begin with a # symbol
		if(tokens.size() == 0 || tokens[0].size() == 0 || tokens[0][0] == '#'){
			continue;
		}

		// there are three kinds of statements to look for
		// 1: regular transitions
		//  <transition_label> <from_state> -> <to_state> <conditions and properties>
		//   or 
		//  <from_state> -> <to_state> <conditions and properties>
		// 
		// 2: data source definitions
		//  <lower_bound> <= <data_source_label> <= <upper_bound>
		//
		// 3: data transitions
		//  <transition_label>: <data_source_label> ['+=', '-=', '='] <value_or_delta> <conditions and properties>
		//   or
		//  <data_source_label> ['+=', '-=', '='] <value_or_delta> <conditions and properties>

		bool is_data_source_def = tokens.size() == 5 && tokens[1] == "<=" && tokens[3] == "<=";
		if(is_data_source_def){
			string lower_bound_str = tokens[0];
			string upper_bound_str = tokens[4];
			string data_source_label = tokens[2];

			int lower_bound = stoi(lower_bound_str);
			int upper_bound = stoi(upper_bound_str);
			DataSource src = DataModel::create_source(data_source_label, lower_bound, upper_bound);
			source_map.insert({data_source_label, src});
		}else{
			bool is_transition_controlled;
			string transition_label;
			string from_state_label;
			string to_state_label;
			string transition_type_str;
			vector<string> flag_strs;

			// see if there is a transition label
			bool is_transition_labelled = tokens[0].size() > 2 
				&& ( (tokens[0][0] == '<' && tokens[0][tokens[0].size() - 1] == '>') 
					|| (tokens[0][0] == '[' && tokens[0][tokens[0].size() - 1] == ']') );
			if(is_transition_labelled && tokens.size() >= 4){
				transition_label = tokens[0].substr(1, tokens[0].size() - 2);
				from_state_label = tokens[1];
				transition_type_str = tokens[2];
				to_state_label = tokens[3];
				for(int j = 4; j < tokens.size(); j++){
					flag_strs.push_back(tokens[j]);
				}
				is_transition_controlled = tokens[0][0] == '<' && tokens[0][tokens[0].size() - 1] == '>';
			}else if(tokens.size() >= 3){
				// no transition label
				transition_label = " t" + to_string(anonymous_transition_count++);
				from_state_label = tokens[0];
				transition_type_str = tokens[1];
				to_state_label = tokens[2];
				for(int j = 3; j < tokens.size(); j++){
					flag_strs.push_back(tokens[j]);
				}
				is_transition_controlled = false;
			}else{
				// ???
				cout << "could not parse line " << line_count << ": " << raw_line << endl;
				continue;
			}

			// determine transition type
			bool is_data_transition;
			if(transition_type_str == "->"){
				is_data_transition = false;
			}else if(transition_type_str == "=" || transition_type_str == "+=" || transition_type_str == "-="){
				is_data_transition = true;
			}else{
				cout << "unrecognized transition type " << transition_type_str << " on line " << raw_line << endl;
				continue;
			}

			// determine to and from states
			astate_p from_state, to_state;
			if(is_data_transition){
				if(source_map.find(from_state_label) == source_map.end()){
					// the data source has not been defined yet
					cout << "undefined data model " << from_state_label << " on line " << raw_line << endl;
					continue;
				}
				DataSource src = source_map.at(from_state_label);
				from_state = src->value_any();
				
				int val = stoi(to_state_label);

				if(transition_type_str == "="){
					to_state = src->value_at(val);
				}else if(transition_type_str == "+="){
					to_state = src->value_change(val);
				}else if(transition_type_str == "-="){
					to_state = src->value_change(-val);
				}
			}else{
				if(states_map.find(from_state_label) == states_map.end()){
					states_map.insert({from_state_label, AtomicState::create(from_state_label)});
				}
				from_state = states_map.at(from_state_label);

				if(states_map.find(to_state_label) == states_map.end()){
					states_map.insert({to_state_label, AtomicState::create(to_state_label)});
				}
				to_state = states_map.at(to_state_label);
			}
			
			// determine transition properties
			double prob = 1.;
			double cost = 1.;
			int priority = 0;
			vector<pair<bool, State>> conditions;
			vector<pair<double, State>> conditional_costs;
			vector<pair<double, State>> conditional_probs;
			for(const string& flag_str : flag_strs){
				if(flag_str[0] == '@' && flag_str.size() > 1){
					// conditions
					bool active_condition = true;
					size_t condition_str_start = 1;
					if(flag_str[1] == '!'){
						active_condition = false;
						if(flag_str.size() > 2){
							condition_str_start = 2;
						}else{
							continue;
						}
					}

					string condition_str = flag_str.substr(condition_str_start, flag_str.size() - condition_str_start);
					if(condition_str[0] == '(' && condition_str[condition_str.size() - 1] == ')'){
						condition_str = condition_str.substr(1, condition_str.size() - 2);
					}

					vector<string> condition_state_strs = split_by_delimiter(condition_str, ",");
					vector<astate_p> condition_states;
					for(const string& cs : condition_state_strs){
						if(cs.size() > 0){
							// check to see if this is a data source condition
							size_t cs_op_pos = cs.find_first_of("<=>");
							if(cs_op_pos != string::npos && cs.size() > cs_op_pos + 1){
								string cs_data_label = cs.substr(0, cs_op_pos);
								if(source_map.find(cs_data_label) == source_map.end()){
									// the data source has not been defined yet
									cout << "undefined data model " << cs_data_label << " on line " << raw_line << endl;
									continue;
								}
								DataSource src = source_map.at(cs_data_label);

								int val;
								if(cs[cs_op_pos] == '<' && cs[cs_op_pos + 1] == '='){
									val = stoi(cs.substr(cs_op_pos + 2));
									condition_states.push_back(src->value_leq(val));
								}else if(cs[cs_op_pos] == '>' && cs[cs_op_pos + 1] == '='){
									val = stoi(cs.substr(cs_op_pos + 2));
									condition_states.push_back(src->value_geq(val));
								}else if(cs[cs_op_pos] == '='){
									val = stoi(cs.substr(cs_op_pos + 1));
									condition_states.push_back(src->value_eq(val));
								}else if(cs[cs_op_pos] == '<'){
									val = stoi(cs.substr(cs_op_pos + 1));
									condition_states.push_back(src->value_lt(val));
								}else if(cs[cs_op_pos] == '>'){
									val = stoi(cs.substr(cs_op_pos + 1));
									condition_states.push_back(src->value_gt(val));
								}

							}else{
								// atomic state condition
								if(states_map.find(cs) == states_map.end()){
									states_map.insert({cs, AtomicState::create(cs)});
								}
								condition_states.push_back(states_map.at(cs));
							}
						}
					}

					conditions.push_back({active_condition, State(condition_states)});
				}else{
					// other properties
					size_t eq_pos = flag_str.find_first_of("=");
					if(eq_pos != string::npos && eq_pos == flag_str.find_last_of("=")){
						string key_str = flag_str.substr(0, eq_pos);
						string val_str = flag_str.substr(eq_pos+1);
						if(key_str.size() > 0 && val_str.size() > 0){
							double value;
							if(val_str[val_str.size() - 1] == '%'){
								value = stod(val_str.substr(0, val_str.size() - 1))/100.;
							}else{
								value = stod(val_str);
							}

							if(equals_ignore_case(key_str, "cost")){
								// base cost
								cost = value;
							}else if(equals_ignore_case(key_str, "prob")){
								// base probability
								prob = value;
							}else if(equals_ignore_case(key_str, "priority")){
								// priority
								priority = (int)value;
							}else{
								// possible conditional properties
								size_t at_pos = key_str.find_first_of("@");
								if(at_pos != string::npos && at_pos == key_str.find_last_of("@")){
									string key_type_str = key_str.substr(0, at_pos);
									string key_cond_str = key_str.substr(at_pos+1);
									if(key_type_str.size() > 0 && key_cond_str.size() > 0){

										if(key_cond_str[0] == '(' && key_cond_str[key_cond_str.size() - 1] == ')'){
											key_cond_str = key_cond_str.substr(1, key_cond_str.size() - 2);
										}

										vector<string> key_cond_state_strs = split_by_delimiter(key_cond_str, ",");
										vector<astate_p> key_cond_states;
										for(const string& cs : key_cond_state_strs){
											if(cs.size() > 0){
												// check to see if this is a data source condition
												size_t cs_op_pos = cs.find_first_of("<=>");
												if(cs_op_pos != string::npos && cs.size() > cs_op_pos + 1){
													string cs_data_label = cs.substr(0, cs_op_pos);
													if(source_map.find(cs_data_label) == source_map.end()){
														// the data source has not been defined yet
														cout << "undefined data model " << cs_data_label << " on line " << raw_line << endl;
														continue;
													}
													DataSource src = source_map.at(cs_data_label);

													int val;
													if(cs[cs_op_pos] == '<' && cs[cs_op_pos + 1] == '='){
														val = stoi(cs.substr(cs_op_pos + 2));
														key_cond_states.push_back(src->value_leq(val));
													}else if(cs[cs_op_pos] == '>' && cs[cs_op_pos + 1] == '='){
														val = stoi(cs.substr(cs_op_pos + 2));
														key_cond_states.push_back(src->value_geq(val));
													}else if(cs[cs_op_pos] == '='){
														val = stoi(cs.substr(cs_op_pos + 1));
														key_cond_states.push_back(src->value_eq(val));
													}else if(cs[cs_op_pos] == '<'){
														val = stoi(cs.substr(cs_op_pos + 1));
														key_cond_states.push_back(src->value_lt(val));
													}else if(cs[cs_op_pos] == '>'){
														val = stoi(cs.substr(cs_op_pos + 1));
														key_cond_states.push_back(src->value_gt(val));
													}
												}else{
													// atomic state condition
													if(states_map.find(cs) == states_map.end()){
														states_map.insert({cs, AtomicState::create(cs)});
													}
													key_cond_states.push_back(states_map.at(cs));
												}
											}
										}

										State key_cond_state(key_cond_states);
										if(equals_ignore_case(key_type_str, "cost")){
											conditional_costs.push_back({value, key_cond_state});
										}else if(equals_ignore_case(key_type_str, "prob")){
											conditional_probs.push_back({value, key_cond_state});
										}else{
											cout << "unknown flag " << key_type_str << " on line " << line_count << endl;
										}
									}else{
										cout << "unknown flag " << key_str << " on line " << line_count << endl;
									}
								}else{
									cout << "unknown flag " << key_str << " on line " << line_count << endl;
								}
							}
						}else{
							cout << "could not parse flags on line " << line_count << ": " << flag_str << endl;
						}
					}else{
						cout << "could not parse flags on line " << line_count << ": " << flag_str << endl;
					}
				}
			}

			// create transition
			transition_p transition;
			if(is_transition_controlled){
				transition = Transition::create_controlled(transition_label, from_state, to_state, cost, prob, priority);
			}else{
				transition = Transition::create_natural(transition_label, from_state, to_state, prob, cost, priority);
			}

			for(auto c : conditions){
				if(c.first){
					// active condition
					transition->activate(c.second);
				}else{
					// inhibit condition
					transition->inhibit(c.second);
				}
			}

			// conditional properties
			for(auto cc : conditional_costs){
				transition->set_cost_at(cc.second, cc.first);
			}
			for(auto cp : conditional_probs){
				transition->set_probability_at(cp.second, cp.first);
			}

			transitions.push_back(transition);
		}
	}

	return StateModel(transitions);
}