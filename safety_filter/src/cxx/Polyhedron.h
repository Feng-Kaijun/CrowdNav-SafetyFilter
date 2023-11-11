#ifndef SAFETY_FILTER_POLYHEDRON_H
#define SAFETY_FILTER_POLYHEDRON_H

#include <Eigen/Core>
#include <vector>
#include "cdd_interface.h"
#include <chrono>

const double tolerance = 1e-6;

using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;
using namespace Eigen;
using namespace std;

namespace safety_filter {

    class Polyhedron {
        public:
            // init
            Polyhedron();
            Polyhedron(const MatrixXd& A, const VectorXd& b); // Ax <= b
            Polyhedron(const MatrixXd& points); // vrep to hrep
            ~Polyhedron() {};
            // getters
            MatrixXd getA() const;
            VectorXd getB() const;
            int getDimNums() const;
            int getConNums() const;
            MatrixXd getVertices();
            void printAB() const;
            // operation may change the polyhedron
            Polyhedron addConstraints(const MatrixXd& A, const VectorXd& b);
            Polyhedron intersect(const Polyhedron& other);
            Polyhedron addDimensions(const Polyhedron& other);
            Polyhedron invAffineMap(const MatrixXd& T);
            Polyhedron projection(const int el_nums);
            // operation only return true or false
            bool isContain(const VectorXd& point);
            bool isSubset(const Polyhedron& other);
            bool isEqual(Polyhedron& other);
            // minimize the H representation, without changing the polyhedron
            Polyhedron minHRep();

        private:
            int DimNums_;
            int ConNums_;
            MatrixXd A_;
            VectorXd b_;
    };

}

#endif //SAFETY_FILTER_POLYHEDRON_H
