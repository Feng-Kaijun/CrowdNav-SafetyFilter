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
    state_consB << 1, 1.5, 0.35, 1, 1, 1.5, 0.35, 1;
    Polyhedron state_cons(state_consA, state_consB);

    MatrixXd inputA(2, 1);
    inputA << -1, 1;
    VectorXd inputB(2, 1);
    inputB << 1, 1;
    Polyhedron input(inputA, inputB);

    MatrixXd sysA(4, 4);
    sysA <<1, 0.1, 0, 0,
           0, 0.9818, 0.2673, 0,
           0, 0, 1, 0.1,
           0, -0.0455, 3.1182, 1;
    MatrixXd sysB(4, 1);
    sysB << 0, 0.1818, 0, 0.4546;
    Safety_Filter SF(sysA, sysB);

    SF.set_state_constrains(state_cons);
    SF.set_input_constrains(input);
    SF.set_rho(1);

    high_resolution_clock::time_point beginTime = high_resolution_clock::now();
    Polyhedron safe_set = SF.get_InvariantSet(1000);
    high_resolution_clock::time_point endTime = high_resolution_clock::now();
    double timeInterval = std::chrono::duration<double, std::milli>(endTime - beginTime).count();
    cout << "Running Time：" << timeInterval  << "ms" << endl;
    safe_set.printAB();
}