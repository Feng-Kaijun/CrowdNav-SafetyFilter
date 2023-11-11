#ifndef SAFETY_FILTER_SAFETY_FILTER_H
#define SAFETY_FILTER_SAFETY_FILTER_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include "Polyhedron.h"

using namespace Eigen;

namespace safety_filter {

    class Safety_Filter {
    public:
        Safety_Filter(const MatrixXd A, const MatrixXd B);
        ~Safety_Filter() {};
        void set_state_constrains(const Polyhedron state_constrains);
        void set_input_constrains(const Polyhedron input_constrains);
        void set_rho(const double rho_value);
        Polyhedron get_state_constrains() const;
        Polyhedron get_input_constrains() const;
        double get_rho() const;
        Polyhedron get_ReachableSet(const Polyhedron state_constrains, const Polyhedron input_constrains);
        Polyhedron get_InvariantSet(const int max_iter);
        VectorXd get_NextState(const VectorXd& current_state, const VectorXd& current_input);
        VectorXd get_SafeAction(const VectorXd& current_state, const VectorXd& current_input, const VectorXd& goal_state, const Polyhedron& safe_set);

    private:
        // x_k+1 = A_ * x_k + B_ * u_k
        double rho_;
        MatrixXd A_;
        MatrixXd B_;
        Polyhedron state_constrains_;
        Polyhedron input_constrains_;
    };

}

#endif //SAFETY_FILTER_SAFETY_FILTER_H
