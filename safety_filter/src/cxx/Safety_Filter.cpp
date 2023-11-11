#include "Safety_Filter.h"

using namespace Eigen;

namespace safety_filter {

    Safety_Filter::Safety_Filter(const MatrixXd A, const MatrixXd B):
        A_(A),
        B_(B) {}

    void Safety_Filter::set_state_constrains(const Polyhedron state_constrains) {
        state_constrains_ = state_constrains;
    }

    void Safety_Filter::set_input_constrains(const Polyhedron input_constrains) {
        input_constrains_ = input_constrains;
    }

    void Safety_Filter::set_rho(const double rho_value) {
        rho_ = rho_value;
    }

    Polyhedron Safety_Filter::get_state_constrains() const {
        return state_constrains_;
    }

    Polyhedron Safety_Filter::get_input_constrains() const {
        return input_constrains_;
    }

    double Safety_Filter::get_rho() const {
        return rho_;
    }

    Polyhedron Safety_Filter::get_ReachableSet(const Polyhedron state_constrains, const Polyhedron input_constrains) {
        Polyhedron XU = state_constrains;
        XU = XU.addDimensions(input_constrains);

        MatrixXd T = MatrixXd::Zero(A_.rows() + B_.cols(), A_.cols() + B_.cols());
        T.topLeftCorner(A_.rows(), A_.cols()) = A_;
        T.topRightCorner(B_.rows(), B_.cols()) = B_;
        T.bottomRightCorner(B_.cols(), B_.cols()) = MatrixXd::Identity(B_.cols(), B_.cols());
        XU = XU.invAffineMap(T);

        XU = XU.projection(input_constrains.getDimNums());
        XU = XU.minHRep();
        return XU;
    }

    Polyhedron Safety_Filter::get_InvariantSet(const int max_iter) {
        bool converged = false;
        Polyhedron X0 = state_constrains_;
        Polyhedron X;
        Polyhedron U = input_constrains_;
        int iter = 1;
        while (!converged) {
            X = get_ReachableSet(X0, U);
            X = X.intersect(X0);
            X = X.minHRep();
            if (X.isEqual(X0)) {
                converged = true;
                //cout << "Iterations " << iter << endl;
            }
            else {
                X0 = X;
                if (++iter > max_iter) {
                    //cout << "Reach max iterations, but the invariant set is not optimal!" << endl;
                    break;
                }
            }
        }
        return X;
    }

    VectorXd Safety_Filter::get_NextState(const VectorXd& current_state, const VectorXd& current_input) {
        return A_ * current_state + B_ * current_input;
    }

    VectorXd Safety_Filter::get_SafeAction(const VectorXd& current_state, const VectorXd& current_input, const VectorXd &goal_state, const Polyhedron& safe_set) {
        /*
         *  QP:
         *     min  0.5 X^T H X + g^T X
         *      X
         *     s.t. lb <= AX <= Aub
         *
         */
        // allocate QP problem matrices and vectores
        int n = current_input.size(); // optimal variable nums
        int m = input_constrains_.getConNums() + safe_set.getConNums(); // rows num of A
        MatrixXd H(n, n);              //H: n*n
        VectorXd g(n);                   //g: n*1
        MatrixXd A(m, n);              //A: m*n
        VectorXd Aub(m);                 //Aub: m*1
        VectorXd Alb(m);                 //Alb: m*1
        // H = 2E + 2B^T B
        H = MatrixXd::Identity(n, n) * 2 + 2 * rho_ * B_.transpose() * B_;
        SparseMatrix<double> hessian(n, n);
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                hessian.insert(i, j) = H(i, j);
            }
        }
        // g
        g = 2 * rho_ * B_.transpose() * A_ * current_state - 2 * current_input - 2 * rho_ * B_.transpose() * goal_state;
        // A
        A.topRows(input_constrains_.getConNums()) = input_constrains_.getA();
        A.bottomRows(safe_set.getConNums()) = safe_set.getA() * B_;
        SparseMatrix<double> linearMatrix(m, n);
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                linearMatrix.insert(i, j) = A(i, j);
            }
        }
        // Aub
        Aub.head(input_constrains_.getConNums()) = input_constrains_.getB();
        Aub.tail(safe_set.getConNums()) = safe_set.getB() - safe_set.getA() * A_ * current_state;
        // Alb
        for (int i = 0; i < m; i++) Alb(i) = -1 * OsqpEigen::INFTY;
        // print
    //    cout << "H: " << endl << H << endl;
    //    cout << "g: " << endl << g << endl;
    //    cout << "A: " << endl << A << endl;
    //    cout << "Aub: " << endl << Aub << endl;
    //    cout << "Alb: " << endl << Alb << endl;

        // instantiate the solver
        OsqpEigen::Solver solver;
        // settings
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);

        // set the initial data of the QP solver
        solver.data()->setNumberOfVariables(n);   //变量数n
        solver.data()->setNumberOfConstraints(m); //约束数m
        if (!solver.data()->setHessianMatrix(hessian)) throw "error";
        if (!solver.data()->setGradient(g)) throw "error";
        if (!solver.data()->setLinearConstraintsMatrix(linearMatrix)) throw "error";
        if (!solver.data()->setLowerBound(Alb)) throw "error";
        if (!solver.data()->setUpperBound(Aub)) throw "error";
        if (!solver.initSolver()) throw "error";

        // solve the QP problem
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) throw "error";
        VectorXd QPSolution = solver.getSolution();
//        cout << "QPSolution" << endl << QPSolution << endl;
        return QPSolution;
    }
}