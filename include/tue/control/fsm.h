#ifndef TUE_CONTROL_FSM_H_
#define TUE_CONTROL_FSM_H_

#include <map>

namespace tue
{
namespace control
{

template<typename State, typename Event>
class FSM
{

public:

    FSM() : initialized_(false) {}

    ~FSM() {}

    void setInitialState(const State& s) { current_state_ = s; initialized_ = true; }

    void addTransition(const State& s1, const Event& e, const State& s2)
    {
        Transition& t = transition_map_[s1][e];
        t.state = s2;
    }

    bool step(const Event& e)
    {
        if (!initialized_)
            return false;

        typename std::map<State, std::map<Event, Transition> >::const_iterator it = transition_map_.find(current_state_);
        if (it == transition_map_.end())
            return false;

        const std::map<Event, Transition>& transitions = it->second;

        typename std::map<Event, Transition>::const_iterator it2 = transitions.find(e);
        if (it2 == transitions.end())
            return false;

        const Transition& t = it2->second;
        current_state_ = t.state;

        return true;
    }

    const State& current_state() const { return current_state_; }


private:

    bool initialized_;

    State current_state_;

    struct Transition
    {
        State state;
    };

    std::map<State, std::map<Event, Transition> > transition_map_;

};

} // end namespace control

} // end namespace tue

#endif
