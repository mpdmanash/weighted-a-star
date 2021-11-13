# Weighted A Star
## _A light-weight, reusable, and efficient C++17 implementation of Weighted A* planning algorithm_

## Features
- Can be easily added to your project
- Uses modern C++ optimizations
- Efficient and Light-Weight

## How to use it in your project
- The implementation uses C++ templates, so you can have any data-structure that represents a state in the planning problem. The only requirement is that you also provide a hash function for the same data-structure **if it is not available already in C++**. Initialize the planner with custom data-structure that represents state `stateT` and action `actionT`, optionally along with the custom hash function for the state as such `WAstar<stateT, actionT, stateHasher>`.
- You just have to provide the planner with three helper functions based on your domain
    - `double getH(stateT s, stateT goal)`: Your implementation of any heuristic function for your domain which takes in a state `s` and finds the heuristic value to the `goal`. The return type of `double` is fixed in my implementation as it works will every case.
    - `getSuccessors(stateT s)`: A function which my planner will call to get the successors of any state `s`. This allows you to have implicit, explicit or latice-based graphs etc. It returns a more complicated data-structure, so please refer to the code.
    - `bool satisfiesGoal(stateT s, stateT goal)`: Sometimes for your planning problem the goal may be under-represented. Thus, my planner will allow you to specify a custom search termination condition which checks whether a state `s` satisfies the goal condition given by the state `goal`.

## Installation
- Just include the main header: `#include "lib_planner.h"`
- Make sure to build with `-std=c++17` flag and `-O3` would also help.

### Feature Tracker
- [ ] Add demo planning problem 
- [ ] Support for a goal region instead of a single goal state

mpdmanash (at) cmu.edu
