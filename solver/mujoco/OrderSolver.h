//
// Created by tianyan on 2/26/26.
//

#ifndef TAIYI_ORDERSOLVER_H
#define TAIYI_ORDERSOLVER_H

#include "ISolver.h"
#include "Model.h"

class OrderSolver : public ISolver {
public:
    explicit OrderSolver(const MModel &model, const float angular_damping = 0.05f,
        const int update_mass_matrix_interval = 1, const float friction_smoothing = 1.0f) :
        model_(model),
        angular_damping(angular_damping),
        friction_smoothing(friction_smoothing),
        update_mass_matrix_interval(update_mass_matrix_interval) {};

    ~OrderSolver() override = default;

    void clear() override{};

    void Step(State& state_in, State& state_out, const Contacts* contacts, float dt) override{};

private:

    void compute_articulation_indices();

private:

    const MModel&  model_;

    float angular_damping;

    float friction_smoothing;

    int update_mass_matrix_interval;

    // total flattened sizes
    int J_size = 0;
    int M_size = 0;
    int H_size = 0;

    // per-articulation offsets
    std::vector<int> articulation_J_start;
    std::vector<int> articulation_M_start;
    std::vector<int> articulation_H_start;

    // per-articulation matrix shapes
    std::vector<int> articulation_M_rows;   // = 6 * joint_count
    std::vector<int> articulation_H_rows;   // = dof_count
    std::vector<int> articulation_J_rows;   // = 6 * joint_count
    std::vector<int> articulation_J_cols;   // = dof_count

    // mapping to global joint-space arrays
    std::vector<int> articulation_dof_start;
    std::vector<int> articulation_coord_start;


};




#endif //TAIYI_ORDERSOLVER_H
