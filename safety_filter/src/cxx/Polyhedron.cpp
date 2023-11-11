#include "Polyhedron.h"

using namespace Eigen;
using namespace std;

namespace safety_filter {

    Polyhedron::Polyhedron() {
        DimNums_ = 0;
        ConNums_ = 0;
    }

    Polyhedron::Polyhedron(const MatrixXd& A, const VectorXd& b) : A_(A), b_(b) {
        DimNums_ = A_.cols();
        ConNums_ = A_.rows();
    }

    Polyhedron::Polyhedron(const MatrixXd& points) {
        MatrixXd A;
        VectorXd b;
        getInequa(points, A, b);
        A_ = A;
        b_ = b;
        DimNums_ = A_.cols();
        ConNums_ = A_.rows();
    }

    MatrixXd Polyhedron::getA() const {
        return A_;
    }

    VectorXd Polyhedron::getB() const {
        return b_;
    }

    int Polyhedron::getDimNums() const {
        return DimNums_;
    }

    int Polyhedron::getConNums() const {
        return ConNums_;
    }

    MatrixXd Polyhedron::getVertices() {
        MatrixXd Vertex;
        getVertex(A_, b_, Vertex);
        return Vertex;
    }

    void Polyhedron::printAB() const {
        cout << endl << "DimNums is " << DimNums_ << "; " << "ConNums is " << ConNums_ << ";" << endl;
        cout << "A is " << endl << A_ << endl;
        cout << "b is " << endl << b_.transpose() << endl;
    }

    Polyhedron Polyhedron::addConstraints(const MatrixXd& A, const VectorXd& b) {
        if (DimNums_ != A.cols()) throw "dims of A should be equal to dims of this polyhedron";
        if (A.rows() != b.rows()) throw "rows of A should be equal to rows of b";
        MatrixXd result_A = A_;
        VectorXd result_b = b_;
        result_A.conservativeResize(result_A.rows() + A.rows(), result_A.cols());
        result_A.bottomRows(A.rows()) = A;
        result_b.conservativeResize(result_b.rows() + b.rows());
        result_b.tail(b.rows()) = b;
        Polyhedron result(result_A, result_b);
        return result;
    }

    Polyhedron Polyhedron::intersect(const Polyhedron& other) {
        if (DimNums_ != other.getDimNums()) throw "dim of P1 and P2 should be equal";
        Polyhedron result(A_, b_);
        result = result.addConstraints(other.getA(), other.getB());
        return result;
    }

    Polyhedron Polyhedron::addDimensions(const Polyhedron& other) {
        /*
         *   this polyhedron: H1*x <= h1, other polyhedron: H2*y <= h2
         *   after appending: [H1, 0; 0, H2] * [x, y]^T <= [h1, h2]^T
         */
        VectorXd new_b(ConNums_ + other.getConNums());
        new_b.head(ConNums_) = b_;
        new_b.tail(other.getConNums()) = other.getB();

        MatrixXd new_A = MatrixXd::Zero(new_b.size(), DimNums_ + other.getDimNums());
        new_A.topLeftCorner(ConNums_, DimNums_) = A_;
        new_A.bottomRightCorner(other.getConNums(), other.getDimNums()) = other.getA();
        Polyhedron result(new_A, new_b);
        return result;
    }

    Polyhedron Polyhedron::invAffineMap(const MatrixXd& T) {
        /*
         *   for a polyhedron P = {x | Ax <= b}, with a linear map, find a backward reachable polyhedron set M = {x | T*x \in P}
         */
        if (DimNums_ != T.rows()) throw "cols of Polyhedron.A should be equal to rows of T";
        Polyhedron result(A_ * T, b_);
        return result;
    }

    Polyhedron Polyhedron::projection(const int el_nums) {
        if (el_nums >= DimNums_) throw "error.";
        dd_MatrixPtr M1 = NULL, M2 = NULL;
        dd_rowset redset, impl_linset;
        dd_rowindex newpos;
        dd_ErrorType err = dd_NoError;
        M2 = Ab_2_MatrixPtr(A_, b_, 'H');
        for (int i = 1; i <= el_nums; i++) {
            M1 = dd_FourierElimination(M2, &err);
            dd_MatrixCanonicalize(&M1, &impl_linset, &redset, &newpos, &err);
            dd_FreeMatrix(M2);
            M2 = M1;
            set_free(redset);
            set_free(impl_linset);
            free(newpos);
        }
        MatrixXd result_A; VectorXd result_b;
        MatrixPtr_2_Ab(M2, result_A, result_b);
        dd_FreeMatrix(M1);
        Polyhedron result(result_A, result_b);
        return result;
    }

    bool Polyhedron::isContain(const VectorXd& point) {
        if (DimNums_ != point.rows()) throw "dim of this point should be equal to dims of this polyhedron";
        VectorXd tmp = A_ * point - b_;
        for (int i = 0; i < tmp.size(); i++) {
            if (tmp(i) > tolerance) return false;
        }
        return true;
    }

    bool Polyhedron::isSubset(const Polyhedron& other) {
        MatrixXd other_A = other.getA();
        VectorXd other_b = other.getB();
        MatrixXd check_A = A_;
        VectorXd check_b = b_;
        check_A.conservativeResize(check_A.rows() + 1, check_A.cols());
        check_b.conservativeResize(check_b.size() + 1);
        for (int i = 0; i < other.getConNums(); i++) {
            check_A.bottomRows(1) = other_A.row(i);
            check_b.tail(1) = other_b.row(i);

            dd_MatrixPtr check_hrep = Ab_2_MatrixPtr(check_A, check_b, 'H');
            dd_Arow cvec; /* certificate */
            dd_rowrange index = check_hrep->rowsize;
            dd_colrange d = check_hrep->colsize;
            dd_ErrorType error = dd_NoError;
            dd_InitializeArow(d, &cvec);
            if (!dd_Redundant(check_hrep, index, cvec, &error)) {
                return false;
            }
            dd_FreeMatrix(check_hrep);
            dd_FreeArow(d, cvec);
        }
        return true;
    }

    bool Polyhedron::isEqual(Polyhedron& other) {
        bool t1 = this->isSubset(other);
        bool t2 = other.isSubset(*this);
        return t1 && t2;
    }

    Polyhedron Polyhedron::minHRep() {
        dd_MatrixPtr hrep = Ab_2_MatrixPtr(A_, b_, 'H');
        dd_MatrixPtr new_hrep = RedundantRows(hrep);
        // set new A and b for result
        MatrixXd result_A;
        VectorXd result_b;
        MatrixPtr_2_Ab(new_hrep, result_A, result_b);
        //dd_WriteMatrix(stdout,new_hrep); printf("\n");
        Polyhedron result(result_A, result_b);
        return result;
    }

}