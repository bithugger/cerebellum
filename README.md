# Cerebellum

Cerebellum is a set of tools created to solve model-based discrete event planning and estimation problems. These kinds of problems typically arise when trying to devise supervisory control of systems and processes that are inherently event-based, and whose behaviour can be modelled as a discrete event system or finite state machine. Broad classes of systems fall into this category, for example manufacturing processes, vehicles subsystems, robotics, and video game AI. Cerebellum contains tools that allow you to model such systems in a discrete form and solve planning and estimation problems in an automated way.

Cerebellum aims to provide solutions to two main kinds of problems
1. Path planning under constraints
2. State estimation under uncertainties

Path planning problems are problems whose goal is to find a way to bring the system from one state to another using only a certain subset of *controllable* transitions, while taking into account the systems natural responses and obeying restrictions such as forbidden states, cost limits, success rates, and safety thresholds (to be defined). State estimation problems are problems whose goal is to determine the likelihood that the system is in a certain state based on the history of *observable* states, as well as the known properties of the system itself. 

## Motivating example

TODO

## Caution

This project is in a very early stage! Many planned features have not been implemented, and many existing features harbour serious issues and lack proper documentation. IT IS NOT SUITABLE FOR USE IN ANY PROJECT AT THIS POINT.

