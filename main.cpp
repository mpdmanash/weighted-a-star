#include <cstdio>
#include <iostream>
#include <string>
#include <vector>
#include <utility> // for std::pair
#include <fstream>
#include <functional> // for std::bind
#include <chrono>
#include <cmath>
#include "lib_planner.h"


using namespace std;

struct State{
    int x; int y; int t;
    size_t id;

    bool operator==(const State& rhs) const {return id == rhs.id;}

    friend ostream& operator<<(ostream& os, const State& s)
    {
        os << "State: " << s.x << "," << s.y << "," << s.t;
        return os;
    }
};

struct Action{
    float dx, dy, dt, velocity, goal_threshold, timeout, cost;
    size_t id;
    friend ostream& operator<<(ostream& os, const Action& a)
    {
        os << "Action: " << a.dx << "," << a.dy << "," << a.dt;
        return os;
    }
};

class Graph{
public:
    Graph(){}
    void readGraphFile(string filename){
        ifstream myfile (filename);
        size_t sid, spid, mpid, num_vertices, max_actions, num_edges;
        int sx, sy, st, spx, spy, spz;
        float mdx, mdy, mdt, mdv, mgc, mto, mc;
        string line_data;
        myfile >> num_vertices >> max_actions;
        m_max_actions = max_actions;
        m_vertices.resize(num_vertices);
        m_out_edges.resize(num_vertices);
        m_actions.resize(max_actions);
        m_successors_container.resize(max_actions);
        cout << num_vertices << " Num Verts\n";
        for(size_t i = 0; i < num_vertices; i++){
            myfile >> sid >> sx >> sy >> st;
            myfile >> num_edges;
            m_vertices[sid].x = sx; m_vertices[sid].y = sy; m_vertices[sid].t = st; m_vertices[sid].id = sid; 
            for(size_t j=0; j<num_edges; j++){
                myfile >> mpid >> mdx >> mdy >> mdt >> mdv >> mgc >> mto >> mc >> spid >> spx >> spy >> spz;
                m_actions[mpid] = {mdx, mdy, mdt, mdv, mgc, mto, mc, mpid};
                m_out_edges[sid].push_back(make_pair(mpid,spid));
            }
        }
        cout << "Done creating graph\n";
    }
    vector<Successor<Action, State> > getSuccessors(State s){
        size_t num_s = m_out_edges[s.id].size();
        for(size_t i=0; i<num_s; i++){
            size_t mpid = m_out_edges[s.id][i].first;
            size_t spid = m_out_edges[s.id][i].second;
            m_successors_container[i].action = m_actions[mpid];
            m_successors_container[i].state = m_vertices[spid];
            m_successors_container[i].cost = m_actions[mpid].cost;
            m_successors_container[i].valid = true;
        }
        for(size_t i=num_s; i<m_max_actions; i++) m_successors_container[i].valid = false;
        return m_successors_container;
    }
    State getStateByID(size_t id){return m_vertices[id];}
private:
    size_t m_max_actions;
    vector<State> m_vertices;
    vector<Action> m_actions;
    vector<vector<pair<size_t, size_t> > > m_out_edges;
    vector<Successor<Action, State> > m_successors_container;
};

double getH(State state, State goal){return 0;}
// double getH(State s, State g){return sqrt((s.x-g.x)*(s.x-g.x) + (s.y-g.y)*(s.y-g.y)) ;}
bool satisfiesGoal(State state, State goal){return (state.x==goal.x && state.y==goal.y);}
struct StateHasher
{
    size_t operator()(const State& state) const{return state.id;}
};

int main(){
    Graph g;
    g.readGraphFile("full_map.txt");

    using namespace std::placeholders;
    WAstar<State, Action, StateHasher> m_h_planner(1.0, 
                                                   std::bind(getH, _1, _2),
                                                   std::bind(&Graph::getSuccessors, g, _1),
                                                   std::bind(satisfiesGoal, _1, _2) );
    const State start = g.getStateByID(100);
    const State goal  = g.getStateByID(410048);
    
    auto timer_start = std::chrono::high_resolution_clock::now();
    double cost = m_h_planner.Plan(start, goal);
    auto timer_end = std::chrono::high_resolution_clock::now();
    cout << "Planning Time: " << std::chrono::duration<double, std::milli>(timer_end-timer_start).count() << " ms\n";
    cout << "Planner Expanded: " << m_h_planner.getNumStatesExpanded() << " states\n";
    vector<Action> path = m_h_planner.BackTrackPath(goal);

    cout << "Path\n";
    for(auto &action: path) cout << action << " | ";
    cout << "\n" << cost << " path cost\n";
    return 0;
}
