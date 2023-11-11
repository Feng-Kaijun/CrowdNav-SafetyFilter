#include <Polyhedron.h>
#include <Safety_Filter.h>
#include <chrono>
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;
using namespace std;
using namespace safety_filter;

int main() {
    // test invariant set, 3D
    MatrixXd irisA(7, 3);
    irisA << -0.00464, -0.99999, 0,
             1, 0, 0,
              0,  1, 0,
             -1, 0, 0,
                   0,  -1, 0,
                   0,       0, 1,
                   0,       0,-1;
//    irisA << -0.8974, -0.4412, 0,
//            -0.8128, -0.5826, 0,
//            0.9080,  0.4190, 0,
//            -0.4046, -0.9145, 0,
//            0,  0.0995, 0,
//            0,       0, 1,
//            0,       0,-1;
    VectorXd irisB(7, 1);
    irisB << -2.59182, 5.00, 5.00, 5.00, 5.00, 3.14159, 3.14159;
//    irisB << 2.6886, 2.3710, 0.7290, 2.8745, 0.9950, 3.1416, 3.1416;
    Polyhedron iris(irisA, irisB);

    MatrixXd inputA(4, 2);
    inputA << 1, 0, -1, 0, 0, 1, 0, -1;
//    inputA << 0, -0.7071, 0.7071, 0, 0, 0.7071, -0.7071, 0;
    VectorXd inputB(4, 1);
    inputB << 1, 1, 1, 1;
//    inputB << 0.7071, 0.7071, 0.7071, 0.7071;
    Polyhedron input(inputA, inputB);

    MatrixXd sysA(3, 3);
    sysA << 1, 0, -0.250,
            0, 1,  0,
            0, 0,      1;
    MatrixXd sysB(3, 2);
    sysB << 0, 0,
            0.25,  0,
                 0,       0.25;
    Safety_Filter SF(sysA, sysB);
    SF.set_state_constrains(iris);
    SF.set_input_constrains(input);
    SF.set_rho(1);

    high_resolution_clock::time_point beginTime = high_resolution_clock::now();
    Polyhedron safe_set = SF.get_InvariantSet(100);
    high_resolution_clock::time_point endTime = high_resolution_clock::now();
    double timeInterval = std::chrono::duration<double, std::milli>(endTime - beginTime).count();
    cout << "Running Time：" << timeInterval  << "ms" << endl;
    safe_set.printAB();
}