#include <iostream>
#include <list>
#include <cerebellum>
#include <fstream>

using namespace std;
using namespace cerebellum;

const string CB_MODEL = R"(
# fuel states
0 <= fuel <= 5
[burn_fuel] fuel -= 1 @af
<refuel> fuel = 5 @(ag xa)

# vehicle states: landed on ground, flying, or crashed
<takeoff> ag -> af @!fuel=0
<land> af -> ag cost@x1=100 cost@x2=100 cost@x3=100
[crash] af -> ac @fuel=0 cost@x1=1000 cost@x2=1000 cost@x3=1000

# navigation states: to_home, to_dest, or to_alt
<to_home> nd -> nh
<to_home> na -> nh
<to_dest> nh -> nd
<to_dest> na -> nd
<to_alt> nd -> na
<to_alt> nh -> na

# location states: home, dest, alt, or one of three waypoints
#
#  H - 1 - 2 - 3 - D
#          |
#          A

xh -> x1 @(af na) @(af nd)
x1 -> x2 @(af na) @(af nd)
x2 -> x3 @(af nd)
x3 -> xd @(af nd)
xd -> x3 @(af nh) @(af na)
x3 -> x2 @(af nh) @(af na)
x2 -> x1 @(af nh)
x1 -> xh @(af nh)
x2 -> xa @(af na)
xa -> x2 @(af nh) @(af nd)
)";

int main(){
	StateModel model = parse_model(CB_MODEL);

	/** test an instance of this model as a state machine **/
	StateMachine uav(model, model.find_state({"ag", "nh", "xh", "fuel=4"}));
	uav.on_enter(model.find_state({"ag", "xd"}), [](){ cout << "GOAL!" << endl; exit(0); });
    uav.on_enter(model.find_state({"ac"}), [](){ cout << "GAME OVER" << endl; exit(0); });
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