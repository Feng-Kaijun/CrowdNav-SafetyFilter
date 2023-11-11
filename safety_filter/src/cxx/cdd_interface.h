#ifndef SAFETY_FILTER_CDD_INTERFACE_H
#define SAFETY_FILTER_CDD_INTERFACE_H

#include <cddlib/setoper.h>
#include <cddlib/cdd.h>
#include <iostream>
#include <cmath>

using namespace Eigen;
using namespace std;

namespace {

struct cdd_global_constants_initializer {
    cdd_global_constants_initializer() {
        dd_set_global_constants();
        cout << "Loading cdd global constants" << endl;
    }
    ~cdd_global_constants_initializer() {
        cout << "Freeing cdd global constants" << endl;
        dd_free_global_constants();
    }
};
static cdd_global_constants_initializer cdd_init;

}

void dd_check(dd_ErrorType err) {
    if (err != dd_NoError) {
        throw runtime_error("dd error");
    }
}

namespace safety_filter {

dd_MatrixPtr Ab_2_MatrixPtr (const MatrixXd& A, const VectorXd& b, const char H_or_V) {
    // if hrep, you have Ax <= b, this function will make [A, b] become [b, -A]
    // if vrep, you have some points, this function will make b become ones(), and A
    if (A.rows() != b.rows()) throw "rows of A should be equal to rows of b";
    int dim = A.cols();
    dd_MatrixPtr hrep = dd_CreateMatrix(A.rows(), 1 + dim);
    // [A, b] to [b, -A], b - Ax >= 0
    for (int i = 0; i < A.rows(); i++) {
        double dataB = b(i);
        if (fabs(dataB) <= dd_almostzero) dd_set_d(hrep->matrix[i][0], 0);
        else dd_set_d(hrep->matrix[i][0], dataB);

        for (int j = 0; j < dim; j++) {
            double dataA = A(i, j);
            if (fabs(dataA) <= dd_almostzero)  dd_set_d(hrep->matrix[i][j+1], 0);
            else {
                //dd_set_d(hrep->matrix[i][j+1], -A(i,j));
                if (H_or_V == 'H') dd_set_d(hrep->matrix[i][j+1], -dataA);
                if (H_or_V == 'V') dd_set_d(hrep->matrix[i][j+1], dataA);
            }
        }
        //dd_set_d(hrep->matrix[i][dim + 1], 1);
    }
    if (H_or_V == 'H') hrep->representation = dd_Inequality;
    if (H_or_V == 'V') hrep->representation = dd_Generator;
    hrep->numbtype = dd_Real;

    return hrep;
}

void MatrixPtr_2_Ab (dd_MatrixPtr& hrep, MatrixXd& A, VectorXd& b) {
    auto A_col = hrep->colsize - 1;
    auto A_row = hrep->rowsize;
    A.conservativeResize(A_row, A_col);
    b.conservativeResize(A_row, 1);
    // [b, -A] to [A, b]
    for (int i = 0; i < A_row; i++) {
        auto dataB = dd_get_d(hrep->matrix[i][0]);
        if (fabs(dataB) <= dd_almostzero) b(i) = 0;
        else b(i) = dataB;
    }
    for (int i = 0; i < A_row; i++) {
        for (int j = 0; j < A_col; j++) {
            auto dataA = dd_get_d(hrep->matrix[i][j + 1]);
            if (fabs(dataA) <= dd_almostzero) A(i, j) = 0;
            else  A(i, j) = -dataA;
        }
    }
}

dd_MatrixPtr RedundantRows(dd_MatrixPtr target) {
    dd_ErrorType err = dd_NoError;
    dd_rowset redrows = dd_RedundantRows(target, &err);
    return dd_MatrixSubmatrix(target, redrows);
}

void getInequa(const MatrixXd& points, MatrixXd& A, VectorXd& b) {
    VectorXd Vrep_b = VectorXd::Ones(points.rows(), 1);
    dd_MatrixPtr vrep = Ab_2_MatrixPtr(points, Vrep_b, 'V');
    //dd_WriteMatrix(stdout,vrep); printf("\n");

    //vrep = RedundantRows(vrep);
    //dd_WriteMatrix(stdout,vrep); printf("\n");

    dd_ErrorType err;
    dd_PolyhedraPtr poly = dd_DDMatrix2Poly(vrep, &err);
//    dd_check(err);

    dd_MatrixPtr Inequa = dd_CopyInequalities(poly);
    //dd_WriteMatrix(stdout,Inequa); printf("\n");

    //Inequa = RedundantRows(Inequa);
    //dd_WriteMatrix(stdout,Inequa); printf("\n");

    MatrixPtr_2_Ab(Inequa, A, b);

    dd_FreeMatrix(vrep);
    dd_FreeMatrix(Inequa);
    dd_FreePolyhedra(poly);
}

void getVertex(const MatrixXd& A, const VectorXd& b, MatrixXd& points) {
    dd_MatrixPtr hrep = Ab_2_MatrixPtr(A, b, 'H');
    int dim = A.cols();

    dd_ErrorType err;
    dd_PolyhedraPtr poly = dd_DDMatrix2Poly(hrep, &err);
//    dd_check(err);

    dd_MatrixPtr generators = dd_CopyGenerators(poly);

    // RedundantRows
    //generators = RedundantRows(generators);
    int row_num = 0;
    for (int i = 0; i < generators->rowsize; i++) {
        if (dd_get_d(generators->matrix[i][0]) == 1) {
            row_num++;
        }
    }
    points.conservativeResize(row_num, dim);
    assert(dim + 1== generators->colsize);
    for (int i = 0; i < generators->rowsize; i++) {
        if (dd_get_d(generators->matrix[i][0]) == 1) {
            VectorXd point(dim);
            for (int j = 0; j < dim; j++) {
                double dataV = dd_get_d(generators->matrix[i][j+1]);
                if (fabs(dataV) <= dd_almostzero) point(j) = 0;
                else point(j) = dataV;
            }
            points.row(i) = point;
        }
    }
    dd_FreeMatrix(hrep);
    dd_FreeMatrix(generators);
    dd_FreePolyhedra(poly);
}

}

#endif //SAFETY_FILTER_CDD_INTERFACE_H
