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
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <algorithm>

using namespace cerebellum;
using namespace std;

char* get_cmd_option(char** begin, char** end, const string & option)
{
	char** it = find(begin, end, option);
	if (it != end && ++it != end)
	{
		return *it;
	}
	return 0;
}

bool cmd_option_exists(char** begin, char** end, const string& option)
{
	return find(begin, end, option) != end;
}

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

StateModel parse_file(fstream& file){
	vector<string> lines;

	string line;
	while(getline(file, line)) {
		lines.push_back(line);
	}

	return parse_model(lines);
}

void print_path(Path path){
	size_t i = 0;
	for(auto it = path.transitions.begin(); it != path.transitions.end(); it++){
		transition_p t = *it;
		if(t->controllable){
			cout << "<" << t->label << ">";
		}else{
			cout << "[" << t->label << "]";
		}
		cout << " ";
		i++;
	}
	if(i > 0){
		cout << ": prob = " << path.probability << ", cost = " << path.cost << endl;
	}
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
		if(t->controllable){
			cout << "<" << t->label << ">";
		}else{
			cout << "[" << t->label << "]";
		}

		cyclesit++;
		i++;
	}
	if(i > 0){
		cout << ": prob = " << pathway.probability << ", cost = " << pathway.cost << endl;;
	}
}

const string HELP_TEXT = R"(
	path <from> <to> [options]
		- find the best path from state <from> to state <to>
		- 'best' is either based on cost or success rate
		- options include cost limit, success rate limit, and obstacles
	
	allpath <from> <to> [options]
		- find all available paths from state <from> to state <to>
		- options can only include obstacles
		- this can be impossibly slow if the model is non-trivial
	
	print
		- display the model in dot-file format, suitable as input to GraphViz
	
	exit/quit/q
		- exit from the program
)";

const string USAGE_TEXT = R"(
	Usage: cb <model_file.cb>
)";

int main(int argc, char* argv[]){
	if(argc < 2){
		// display usage text and exit
		cout << USAGE_TEXT << endl;
		exit(0);
	}

	char* filename = argv[1];

    if(filename){
		// file mode
		fstream file;
		file.open(filename, ios::in);

		if(!file.is_open()){
			exit(1);
		}

		StateModel model = parse_file(file);

		file.close();
		cout << "Model Loaded" << endl;

		string input;
		bool running = true;
		while(running){
			cout << "\033[100;97mcerebellum>\033[0m ";
			getline(cin, input);

			vector<string> inputs = split_by_delimiter(collapse_whitespaces(input), " ");
			if(inputs.size() > 0){
				string command = inputs[0];
				if(command == "exit" || command == "quit" || command == "q"){
					running = false;
				}else if(command == "print"){
					cout << model.as_dot_file() << endl;
				}else if(command == "path" || command == "allpath"){
					if(inputs.size() < 3){
						cout << "Not enough arguments" << endl;
					}else{
						bool all_path = command == "allpath";

						string from_str = inputs[1];
						string to_str = inputs[2];
						vector<string> flags(inputs.begin() + 3, inputs.end());

						if(from_str.size() > 2 && from_str[0] == '(' && from_str[from_str.size() - 1] == ')'){
							from_str = from_str.substr(1, from_str.size() - 2);
						}

						if(to_str.size() > 2 && to_str[0] == '(' && to_str[to_str.size() - 1] == ')'){
							to_str = to_str.substr(1, to_str.size() - 2);
						}

						State from = model.find_state(split_by_delimiter(from_str, ","));
						if(from.empty()){
							cout << "Error: state " << from_str << " not found in model" << endl;
							continue;
						}

						State to = model.find_state(split_by_delimiter(to_str, ","));
						if(to.empty()){
							cout << "Error: state " << to_str << " not found in model" << endl;
							continue;
						}
						
						vector<State> obstacles;
						double prob_limit = 0.;
						double cost_limit = 0.;
						bool prob_limit_specd = false;
						bool cost_limit_specd = false;

						for(const string& flag : flags){
							// check if any obstacles specified
							if(flag.size() > 1 && flag[0] == '!'){
								string obst_str;
								if(flag.size() > 3 && flag[1] == '(' && flag[flag.size() - 1] == ')'){
									obst_str = flag.substr(2, flag.size() - 3);
								}else{
									obst_str = flag.substr(1, flag.size() - 1);
								}

								State obst = model.find_state(split_by_delimiter(obst_str, ","));
								if(obst.empty()){
									cout << "Warning: obstacle state " << obst_str << " not found in model" << endl;
								}
								obstacles.push_back(obst);
							}else{
								// check for other flags
								size_t eq_pos = flag.find_first_of("<>");
								if(eq_pos != string::npos){
									string key_str = flag.substr(0, eq_pos);
									string val_str = flag.substr(eq_pos+1);
									if(flag[eq_pos] == '<' && equals_ignore_case(key_str, "cost")){
										cost_limit = stod(val_str);
										cost_limit_specd = true;
									}else if(flag[eq_pos] == '>' && equals_ignore_case(key_str, "prob")){
										prob_limit = stod(val_str);
										prob_limit_specd = true;
									}else{
										cout << "Warning: unrecognized argument " << flag << endl;
									}
								}else{
									cout << "Warning: unrecognized argument " << flag << endl;
								}
							}
						}

						if(all_path){
							if(prob_limit_specd || cost_limit_specd){
								cout << "Warning: cost and probability limits have no effect in this mode" << endl;
							}
							vector<Path> all_paths = model.find_all_paths_around(from, to, obstacles);
							for(const Path& path : all_paths){
								print_path(path);
							}
						}else{
							if(prob_limit_specd && cost_limit_specd){
								cout << "Error: cannot specify both probability and cost limits" << endl;
							}else if(cost_limit_specd){
								Path path = model.find_likeliest_path_around_under_cost(from, to, obstacles, cost_limit);
								print_path(path);
							}else{
								// either prob_limit_specd or neither specified
								Path path = model.find_best_path_around_over_likelihood(from, to, obstacles, prob_limit);
								print_path(path);
							}
						}
					}
				}else if(command == "help"){
					cout << HELP_TEXT << endl;
				}
			}
		}
	}

	return 0;
}