/*
 * @author Manash Pratim Das (mpratimd@andrew.cmu.edu)
*/

#ifndef LIB_PLANNER
#define LIB_PLANNER

#include <functional>     // for hash
#include <queue>          // for OPEN
#include <unordered_map>  // for G values
#include <unordered_set>  // for CLOSED
#include <vector>         // for successors

namespace was {

/* Struct to hold state with f values in Priority Queue */
template <typename T>
struct FState {
    double f;
    T state;
    bool operator<(const FState<T> &rhs) const { return (-f < -rhs.f); }
};

/*
 * Struct to hold an edge comprising of a state and an action
 * It used to populate successors in A* and to return the solved path
 * When used to populate successors, set the valid variable to true,
 * and add the cost of the edge in the cost member variable.
 */
template <typename ActionT, typename StateT>
struct ActionState {
    ActionT action;
    StateT state;
    double cost = 0;
    bool valid = true;
    ActionState() {}
    ActionState(const ActionT &_action, const StateT &_state) {
        action = _action;
        state = _state;
    }
    friend std::ostream &operator<<(std::ostream &os, const ActionState &s) {
        os << s.state << " " << s.action;
        return os;
    }
};

/*
 * WAStar allows the user to specify own types to represent domain dependent
 * State and Action. The State type should have the == operator defined. A
 * HashFunction for the State msut also be provided if std::hash cannot hash the
 * State type.
 */
template <class StateT, class ActionT, class StateHasher = std::hash<StateT> >
class WAStar {
   public:
    /* Copyable class */
    WAStar(const WAStar &other) = default;
    WAStar &operator=(const WAStar &other) = default;

    /*
     * Generic (domain independent) implementation of the weighted A*
     * algorithm. It can also be configured to run in backwards mode and as
     * Dijkstra's algorithm.
     * The class is initialized with binding to three helper functions provided
     * by the user which are dependent on the domain.
     * @param epsilon: The suboptimality factor for weighted A*
     * @param max_neighbors: Provide an upper bound on the maximum neighbors
     * (successors in forward mode or predecessors in backward mode). This will
     * help reserve memory once and prevent dynamic memory allocation during
     * runtime.
     *
     * @param GetH: Bindin to external helper function which should return the
     * heuristic value of a state
     *
     * @param FillNeighbors: Binding to external helper function to fill a
     * std::vector (passed by pointer) with the neighbors. Depending on forward
     * or backwards mode, user should fill successors or predecessors
     * respectively.
     *
     * @param IsGoalCondition: Binding to external helper function which will
     * let the planner know that the goal condition have been met. This allows
     * the user to specify under-defined goals or goal region.
     */
    WAStar(double epsilon, size_t max_neighbors,
           std::function<double(const StateT &state, const StateT &goal)> GetH,
           std::function<
               void(std::vector<ActionState<ActionT, StateT> > *neighbors,
                    const StateT &state, const StateT &goal)>
               FillNeighbors,
           std::function<bool(const StateT &state, const StateT &goal)>
               IsGoalCondition);
    ~WAStar() {}

    /*
     * Call the planner once the class is initialized. Note that if you want to
     * repeatedly call the planner, use Reset in between to reset the planner.
     */
    double Plan(const StateT &s_start, const StateT &s_goal);

    /*
     * Backtrack a path from the given state.
     * @param s_goal: State to backtrack from. Generally the goal.
     * @returns: Vector of ActionState. Use modern c++ compilers to utilize
     * return value optimization.
     */
    std::vector<ActionState<ActionT, StateT> > BackTrackPath(
        const StateT &s_goal);
    inline size_t GetNumStatesExpanded() { return CLOSED_.size(); }

    /* Call this if you want to use the g values as Heuristic for a different
     * planning problem. Used if you want to run this planner as Dijkstra's
     * algorithm. */
    double GetGHeuristic(const StateT &state, const StateT &s2);
    void Reset();

   private:
    double epsilon_;
    std::function<double(const StateT &state, const StateT &goal)> GetH_;
    std::function<void(std::vector<ActionState<ActionT, StateT> > *neighbors,
                       const StateT &state, const StateT &goal)>
        FillNeighbors_;
    std::function<bool(const StateT &state, const StateT &goal)>
        IsGoalCondition_;
    std::unordered_map<StateT, double, StateHasher> g_values_;
    std::priority_queue<FState<StateT> > OPEN_;
    std::unordered_set<StateT, StateHasher> CLOSED_;
    std::unordered_map<StateT, ActionState<ActionT, StateT>, StateHasher>
        backward_transitions_;
    std::vector<ActionState<ActionT, StateT> > neighbors_;
    StateT backtrack_seed_;

    double GetG(const StateT &state);
    bool IsInCLOSED(const StateT &s,
                    const std::unordered_set<StateT, StateHasher> &CLOSED);
};
}  // namespace was

#include "lib_planner.tpp"

#endif
