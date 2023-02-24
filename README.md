# Cerebellum

Cerebellum is a set of tools created to solve model-based discrete event planning and estimation problems. These kinds of problems typically arise when trying to devise supervisory control of systems and processes that are inherently event-based, and whose behaviour can be modelled as a discrete event system or finite state machine. Broad classes of systems fall into this category: manufacturing processes, vehicles subsystems, robotics, video game AI, etc. Cerebellum contains tools that allow you to model such systems in a discrete form and solve planning and estimation problems in an automated way.

Cerebellum aims to provide solutions to two main kinds of problems
1. Planning and decision under constraints
2. State estimation under uncertainties

Planning problems are problems whose goal is to find a way to bring the system from one state to another using only a certain subset of *controllable* transitions, while taking into account the systems natural responses and obeying restrictions such as forbidden states, cost limits, and success rates. State estimation problems are problems whose goal is to determine the most likely state a system is in based on the history of *observable* states, as well as the known properties of the system itself.

## Motivation

Most real life systems and products of engineering are comprised of many individual components and subsystems, each built to certain specifications and designed to operate within certain limits. Traditionally such products are thought of as tools that a human operator would use to accomplish tasks, and can make sound judgements about how to do so. As the field of autonomous systems develop, more and more of these tasks must be done by computer programs - tasks like planning actions to achieve goals and handling failures. Engineers often approach the design of these programs the same way they approach the design of said tools, i.e. something that must be built to spec and operate within some limits. Often times the extent of the automation within these programs is little more than a composition of simple directives like *if this happens then do that*.

While there is nothing inherently wrong with this approach, programs created this way are often completely ad hoc and frustratingly difficult to scale as the system or operating envelope grows. When the systems grow large enough, such as on the order of autonomous vehicles, the interactions within its various subsystems as well as with the environment become so dense and complicated that ad hoc designs cannot keep up. The only option is to narrowly limit the operating envelope to within what can be feasibly designed for and tested. For example, an autopilot system for an aircraft might be required to operate between 30,000 and 40,000 feet, between airspeeds of 200 and 500 knots, and in temperatures ranging from -40°C to +70°C. But if one is designing a fully autonomous system, what should happen when the aircraft uncontrollably exits that envelope...?

Cerebellum offers an alternative to handcrafted programs when it comes to autonomy. It is a generic program that is given models of the system it is acting on and the environment. It explores the state space through the various interactions available at any point and can provide options for achieving a given goal, as well as evaluate and compare multiple options. By treating all decision problems onboard as dynamic ones and leveraging information provided by the model as a whole, it should be able to make much better decisions than the traditional compositions of independent directives. Autonomous systems making full use of Cerebellum should be able to make the *correct* decision in any circumstance.

## Caution

This project is at a **very early** stage! Many planned features have not been implemented, and many existing features harbour serious issues and lack any documentation. IT IS NOT SUITABLE FOR USE IN ANY PROJECT AT THIS POINT.

That said, if you would like to see some of the ideas in action, read on for some basic instructions.

## Quickstart

Clone the repository and build using cmake
```
(cd into the project dir)
mkdir -p build
cd build
cmake ..
make
```

Run the example program on the simple elevator model
```
./bin/cb ../resources/elevator.cb
```

![Elevator Model](https://g.gravizo.com/svg?
digraph cerebellum {
	newrank=true
	subgraph cluster_floor {
		label=floor
		"floor = 3" [label=3]
		"floor = 2" [label=2]
		"floor = 1" [label=1]
		"floor = 1" -> "floor = 2" [ color=red arrowhead=empty tooltip="activated (mu)\ncost: 1\nprob: 1"]
		"floor = 2" -> "floor = 3" [ color=red arrowhead=empty tooltip="activated (mu)\ncost: 1\nprob: 1"]
		"floor = 2" -> "floor = 1" [ color=red arrowhead=empty tooltip="activated (md)\ncost: 1\nprob: 1"]
		"floor = 3" -> "floor = 2" [ color=red arrowhead=empty tooltip="activated (md)\ncost: 1\nprob: 1"]
	}
	{" do "} -> {" dc "} [ label="close_door" tooltip="cost: 1\nprob: 1"]
	{" dc "} -> {" do "} [ label="open_door" arrowhead=empty tooltip="inhibited (mu) (md)\ncost: 1\nprob: 1"]
	{" ms "} -> {" mu "} [ label="move_up" arrowhead=empty tooltip="inhibited (do)\ncost: 1\nprob: 1"]
	{" mu "} -> {" ms "} [ label="stop" tooltip="cost: 1\nprob: 1"]
	{" ms "} -> {" md "} [ label="move_down" arrowhead=empty tooltip="inhibited (do)\ncost: 1\nprob: 1"]
	{" md "} -> {" ms "} [ label="stop" tooltip="cost: 1\nprob: 1"]
}
)

Elevator model states
- three floor states: floor = 1, floor = 2, and floor = 3
- two door states: door open (do) and door closed (dc)
- three motion states: stopped (ms), moving up (mu), and moving down (md)

Elevator model transitions
- close_door: controllable
- open_door: controllable, inhibited when moving up (mu), inhibited when moving down (md)
- stop: controllable
- move_up: controllable, inhibited when door open (do)
- move_down: controllable, inhibited when door open (do)
- climb: natural, activated when moving up (mu)
- descend: natural, activated when moving down (md)

Find a path from state (floor = 1, door open, stopped) to state (floor = 3, door open, stopped)
```
cerebellum> path (floor=1, do, ms) (floor=3, do, ms)
<close_door> <move_up> [climb] <> [climb] <stop> <open_door> : prob = 1, cost = 6
```

Find all paths from state (floor = 1, door open, stopped) to state (floor = 3, door open)* without entering state (moving down)
```
cerebellum> allpath (floor=1, do, ms) (floor=3, do) !(md)
<close_door> <move_up> [climb] <> [climb] <stop> <open_door> : prob = 1, cost = 6
<close_door> <move_up> [climb] <stop> <move_up> [climb] <stop> <open_door> : prob = 1, cost = 8
```
*no restriction on motion state