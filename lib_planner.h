#ifndef LIB_PLANNER
#define LIB_PLANNER

#include <functional> // for hash
#include <vector> // for successors
#include <unordered_map> // for G values
#include <unordered_set> // for CLOSED
#include <queue> // for OPEN


template <typename T>
struct f_State{
    double f; T state;
    bool operator<(const f_State<T>& rhs) const
    {return (-f < -rhs.f);}
};

template<typename actionT, typename stateT>
struct Successor{
    actionT action;
    stateT state;
    double cost = 0;
    bool valid = true;
};


template <class stateT, class actionT, class stateHasher = std::hash<stateT> >
class WAstar{
public:
    WAstar(double epsilon, 
           std::function<double(const stateT &state, const stateT &goal)> getH, 
           std::function<std::vector<Successor<actionT, stateT> >(const stateT &state)> getSuccessors,
           std::function<bool(const stateT &state, const stateT &goal)> satisfiesGoal);
    double Plan(const stateT &s_start,const stateT &s_goal);
    std::vector<actionT> BackTrackPath(const stateT &s_goal);
    inline size_t getNumStatesExpanded(){return m_CLOSED.size();};
    void Reset();
private:
    double m_epsilon;
    std::function<double(const stateT &state, const stateT &goal)> m_getH;
    std::function<std::vector<Successor<actionT, stateT> >(const stateT &state)> m_getSuccessors;
    std::function<bool(const stateT &state, const stateT &goal)> m_satisfiesGoal;
    std::unordered_map<stateT, double, stateHasher> m_g;
    std::priority_queue<f_State<stateT> > m_OPEN;
    std::unordered_set<stateT, stateHasher> m_CLOSED;
    std::unordered_map<stateT, std::pair<stateT,actionT>, stateHasher> m_transitions;
    stateT backtrackState;

    double getG(const stateT &state);
    bool isInCLOSED(const stateT &s, const std::unordered_set<stateT, stateHasher> &CLOSED);
};

#include "lib_planner.tpp"

#endif
