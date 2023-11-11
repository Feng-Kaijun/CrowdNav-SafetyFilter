#include <Polyhedron.h>
#include <Safety_Filter.h>

using namespace std;
using namespace safety_filter;

int main() {
    // test invariant set
    MatrixXd irisA(6, 2);
    irisA << -0.99161, -0.12930,
            0.96328, -0.26849,
            1.00000,  0.00000,
            0.00000,  1.00000,
            -1.00000, -0.00000,
            -0.00000, -1.00000;
    VectorXd irisB(6, 1);
    irisB << 2.33461, 1.91416, 5.00000, 5.00000, 5.00000, 5.00000;
    Polyhedron iris(irisA, irisB);
    iris.printAB();

    MatrixXd points = iris.getVertices();
    MatrixXd AfromV;
    VectorXd bfromV;
    getInequa(points, AfromV, bfromV);
    Polyhedron irisV(AfromV, bfromV);
    irisV.printAB();
}
