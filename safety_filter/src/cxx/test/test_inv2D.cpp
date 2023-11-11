#include <Polyhedron.h>
#include <Safety_Filter.h>
#include <chrono>
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;
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
//    irisA << 0.09793, 0.99519,
//            0.01525, 0.99988,
//            1.00000,  0.00000,
//            0.00000,  1.00000,
//            -1.00000, -0.00000,
//            -0.00000, -1.00000;
    VectorXd irisB(6, 1);
    irisB << 2.33461, 1.91416, 5.00000, 5.00000, 5.00000, 5.00000;
//    irisB << -3.40963, -3.39902, 5.00000, 5.00000, 5.00000, 5.00000;
    Polyhedron iris(irisA, irisB);

    MatrixXd inputA(4, 2);
    inputA << 1, 0, -1, 0, 0, 1, 0, -1;
    VectorXd inputB(4, 1);
    inputB << 1, 1, 1, 1;
    Polyhedron input(inputA, inputB);

    MatrixXd sysA(2, 2);
    sysA << 1, 0, 0, 1;
    MatrixXd sysB(2, 2);
    sysB << 0.25, 0, 0, 0.25;
    Safety_Filter SF(sysA, sysB);
    SF.set_state_constrains(iris);
    SF.set_input_constrains(input);
    SF.set_rho(1);

    high_resolution_clock::time_point beginTime = high_resolution_clock::now();
    Polyhedron safe_set = SF.get_InvariantSet(1000);
    high_resolution_clock::time_point endTime = high_resolution_clock::now();
    double timeInterval = std::chrono::duration<double, std::milli>(endTime - beginTime).count();
    cout << "Running Time：" << timeInterval  << "ms" << endl;
    safe_set.printAB();
}
