/*
 * @author Manash Pratim Das (mpratimd@andrew.cmu.edu)
*/

#pragma once

namespace was {
template <class StateT, class ActionT, class StateHasher>
WAStar<StateT, ActionT, StateHasher>::WAStar(
    double epsilon, size_t max_neighbors,
    std::function<double(const StateT &state, const StateT &goal)> GetH,
    std::function<void(std::vector<ActionState<ActionT, StateT> > *neighbors,
                       const StateT &state, const StateT &goal)>
        FillNeighbors,
    std::function<bool(const StateT &state, const StateT &goal)>
        IsGoalCondition) {
    epsilon_ = epsilon;
    GetH_ = GetH;
    FillNeighbors_ = FillNeighbors;
    IsGoalCondition_ = IsGoalCondition;
    neighbors_.resize(max_neighbors);
}

template <class StateT, class ActionT, class StateHasher>
double WAStar<StateT, ActionT, StateHasher>::Plan(const StateT &s_start,
                                                  const StateT &s_goal) {
    g_values_[s_start] = 0.0;
    OPEN_.push(FState<StateT>{0.0, s_start});
    bool goal_expanded = false;
    while (!goal_expanded && OPEN_.size() != 0) {
        StateT this_state = OPEN_.top().state;
        OPEN_.pop();
        CLOSED_.insert(this_state);
        if (IsGoalCondition_(this_state, s_goal)) {
            goal_expanded = true;
            backtrack_seed_ = this_state;

            return this->GetG(this_state);
        }
        double g_this_state = this->GetG(this_state);
        FillNeighbors_(&neighbors_, this_state, s_goal);
        for (const auto &neighbor : neighbors_) {
            if (!neighbor.valid) continue;
            if (!this->IsInCLOSED(neighbor.state, CLOSED_)) {
                double h_neighbor = GetH_(neighbor.state, s_goal);
                double g_neighbor = g_this_state + neighbor.cost;
                if (this->GetG(neighbor.state) > g_neighbor) {
                    g_values_[neighbor.state] = g_neighbor;
                    backward_transitions_[neighbor.state] =
                        ActionState<ActionT, StateT>{neighbor.action,
                                                     this_state};
                    OPEN_.push(FState<StateT>{
                        g_neighbor + epsilon_ * h_neighbor, neighbor.state});
                }
            }
        }
    }
    return this->GetG(s_goal);  // infinity if path to goal is not found
}

template <class StateT, class ActionT, class StateHasher>
void WAStar<StateT, ActionT, StateHasher>::Reset() {
    g_values_.clear();
    OPEN_ = std::priority_queue<FState<StateT> >();
    backward_transitions_.clear();
    CLOSED_.clear();
}

template <class StateT, class ActionT, class StateHasher>
std::vector<ActionState<ActionT, StateT> >
WAStar<StateT, ActionT, StateHasher>::BackTrackPath(const StateT &s_goal) {
    std::vector<ActionState<ActionT, StateT> > path;
    StateT prev_state = backtrack_seed_;
    while (true) {
        auto it = backward_transitions_.find(prev_state);
        if (it != backward_transitions_.end()) {
            path.insert(path.begin(), it->second);
            prev_state = it->second.state;
        } else {
            break;
        }
    }
    return path;
}

template <class StateT, class ActionT, class StateHasher>
inline double WAStar<StateT, ActionT, StateHasher>::GetGHeuristic(
    const StateT &s, const StateT &s2) {
    auto it = g_values_.find(s);
    if (it != g_values_.end())
        return it->second;
    else
        return std::numeric_limits<double>::infinity();
}

template <class StateT, class ActionT, class StateHasher>
inline bool WAStar<StateT, ActionT, StateHasher>::IsInCLOSED(
    const StateT &s, const std::unordered_set<StateT, StateHasher> &CLOSED) {
    return (CLOSED.find(s) != CLOSED.end());
}

template <class StateT, class ActionT, class StateHasher>
inline double WAStar<StateT, ActionT, StateHasher>::GetG(const StateT &s) {
    auto it = g_values_.find(s);
    if (it != g_values_.end())
        return it->second;
    else
        return std::numeric_limits<double>::infinity();
}

}  // namespace was
