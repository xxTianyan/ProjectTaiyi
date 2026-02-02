//
// Created by tianyan on 1/6/26.
//

#ifndef TAIYI_ISOLVER_H
#define TAIYI_ISOLVER_H

struct State;


class ISolver {
public:
    ISolver() = default;
    virtual ~ISolver() = default;
    virtual void clear() = 0;
    virtual void Step(State& state_in, State& state_out, const Contacts* contacts, float dt) = 0;
};


#endif //TAIYI_ISOLVER_H
