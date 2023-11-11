#include <Polyhedron.h>
#include <Safety_Filter.h>
#include <chrono>
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;
using namespace std;
using namespace safety_filter;

int main() {
    MatrixXd irisA(7, 3);
    irisA << 0.08514, 0.99637, 0.00000,
             1.00000, 0.00000, 0.00000,
             0.00000, 1.00000, 0.00000,
             -1.0000, 0.00000, 0.00000,
             0.00000, -1.0000, 0.00000,
             0.00000, 0.00000, 1.00000,
             0.00000, 0.00000, -1.0000;
    VectorXd irisB(7, 1);
    irisB << -1.87249, 5.00, 5.00, 5.00, 5.00, 6.28319, 0;
    Polyhedron iris(irisA, irisB);

    MatrixXd inputA(4, 2);
    inputA << 1, 0, -1, 0, 0, 1, 0, -1;
    VectorXd inputB(4, 1);
    inputB << 1, 0, 3.14159, 3.14159;
    Polyhedron input(inputA, inputB);

    MatrixXd sysA(3, 3);
    sysA << 1, 0, -0.07117,
            0, 1, -0.00748,
            0, 0,      1;
    MatrixXd sysB(3, 2);
    sysB << -0.02613, 0,
            0.24863,  0,
            0,       0.25;
    Safety_Filter SF(sysA, sysB);
    SF.set_state_constrains(iris);
    SF.set_input_constrains(input);
    SF.set_rho(1);

    Polyhedron safe_set = SF.get_InvariantSet(100);
    safe_set.printAB();

    VectorXd current_state(3);
    current_state << -0.48974, -2.56652, 1.67552;

    VectorXd goal_state(3);
    goal_state << 0, 4, 1.67552;

    VectorXd current_input(2);
    current_input << 0.28623, 0.05236;

    high_resolution_clock::time_point beginTime = high_resolution_clock::now();
    VectorXd action = SF.get_SafeAction(current_state, current_input, goal_state, safe_set);
    high_resolution_clock::time_point endTime = high_resolution_clock::now();
    double timeInterval = std::chrono::duration<double, std::milli>(endTime - beginTime).count();
    cout << "Running Time：" << timeInterval  << "ms" << endl;
    cout << "action: " << endl << action << endl;
}
