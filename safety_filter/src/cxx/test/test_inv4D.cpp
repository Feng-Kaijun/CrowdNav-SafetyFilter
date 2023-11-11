#include <Polyhedron.h>
#include <Safety_Filter.h>
#include <chrono>
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;
using namespace std;
using namespace safety_filter;

int main() {
    // test invariant set, 4D
    MatrixXd state_consA(8, 4);
    state_consA << -1, 0, 0, 0,
                    0, -1, 0, 0,
                    0, 0, -1, 0,
                    0, 0, 0, -1,
                    1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1;
    VectorXd state_consB(8, 1);
    state_consB << 4, 10, 4, 10, 4, 10, 4, 10;
    Polyhedron state_cons(state_consA, state_consB);

    MatrixXd inputA(2, 1);
    inputA << -1, 1;
    VectorXd inputB(2, 1);
    inputB << 0.5, 0.5;
    Polyhedron input(inputA, inputB);

    MatrixXd sysA(4, 4);
    sysA << 0.763, 0.460, 0.115, 0.020,
            -0.899, 0.763, 0.420, 0.115,
            0.115, 0.020, 0.763, 0.460,
            0.420, 0.115, -0.899, 0.763;
    MatrixXd sysB(4, 1);
    sysB << 0.014, 0.063, 0.221, 0.367;
    Safety_Filter SF(sysA, sysB);

    SF.set_state_constrains(state_cons);
    SF.set_input_constrains(input);
    SF.set_rho(1);

    high_resolution_clock::time_point beginTime = high_resolution_clock::now();
    Polyhedron safe_set = SF.get_InvariantSet(1000);
    safe_set.minHRep();
    high_resolution_clock::time_point endTime = high_resolution_clock::now();
    double timeInterval = std::chrono::duration<double, std::milli>(endTime - beginTime).count();
    cout << "Running Time：" << timeInterval  << "ms" << endl;
    safe_set.printAB();
}