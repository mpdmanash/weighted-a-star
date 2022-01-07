#pragma once

template <class stateT, class actionT, class stateHasher>
WAstar<stateT, actionT, stateHasher>::WAstar(double epsilon, 
        std::function<double(const stateT &state, const stateT &goal)> getH,
        std::function<std::vector<Successor<actionT, stateT> >(const stateT &state)> getSuccessors,
        std::function<bool(const stateT &state, const stateT &goal)> satisfiesGoal){
    m_epsilon = epsilon;
    m_getH = getH;
    m_getSuccessors = getSuccessors;
    m_satisfiesGoal = satisfiesGoal;
}

template <class stateT, class actionT, class stateHasher>
double WAstar<stateT, actionT, stateHasher>::Plan(const stateT &s_start, const stateT &s_goal){
    m_g[s_start] = 0.0;
    m_OPEN.push(f_State<stateT>{0.0, s_start});
    bool goal_expanded = false;
    while(!goal_expanded && m_OPEN.size() != 0){
        auto this_state = m_OPEN.top().state;
        m_OPEN.pop();
        m_CLOSED.insert(this_state);
        if(m_satisfiesGoal(this_state, s_goal)){
            goal_expanded=true; 
            backtrackState=this_state; 
            return this->getG(this_state);
        }
        double g_this_state = this->getG(this_state);
        auto successors = m_getSuccessors(this_state);
        for(const auto &successor: successors){
            if(!successor.valid) continue;
            if(!this->isInCLOSED(successor.state,m_CLOSED)){
                double h_successor = m_getH(successor.state, s_goal);
                double g_successor = g_this_state + successor.cost;
                if (this->getG(successor.state) > g_successor){
                    m_g[successor.state] = g_successor;
                    m_transitions[successor.state] = std::make_pair(this_state,successor.action);
                    m_OPEN.push(f_State<stateT>{g_successor + m_epsilon*h_successor, successor.state});
                }
            }
        }
    }
    return this->getG(s_goal); // infinity if path to goal is not found
}

template <class stateT, class actionT, class stateHasher>
void WAstar<stateT, actionT, stateHasher>::Reset(){
    m_g.clear(); m_OPEN = std::priority_queue<f_State<stateT> >(); m_transitions.clear(); m_CLOSED.clear();
}

template <class stateT, class actionT, class stateHasher>
std::vector<actionT> WAstar<stateT, actionT, stateHasher>::BackTrackPath(const stateT &s_goal){
    std::vector<actionT> path;
    stateT prev_state = backtrackState;
    while(true){
        auto it = m_transitions.find(prev_state);
        if (it != m_transitions.end()) {
            path.insert(path.begin(),it->second.second);  
            prev_state = it->second.first;
        }
        else break;
    }
    return path;
}

template <class stateT, class actionT, class stateHasher>
bool WAstar<stateT, actionT, stateHasher>::isInCLOSED(const stateT &s, 
        const std::unordered_set<stateT, stateHasher> &CLOSED){
    return (CLOSED.find(s) != CLOSED.end());
}

template <class stateT, class actionT, class stateHasher>
double WAstar<stateT, actionT, stateHasher>::getG(const stateT &s){
    auto it = m_g.find(s);
    if (it != m_g.end()) return it->second;
    else return std::numeric_limits<double>::infinity();
}