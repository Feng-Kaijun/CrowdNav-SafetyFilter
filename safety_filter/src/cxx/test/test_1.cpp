#include <polyhedron.h>
#include <LTIsystem.h>

using namespace std;
using namespace safety_filter;

int main() {
    MatrixXd A1(4, 2);
    A1 << 0, 1,
            1, 0,
            0, -1,
            -1, 0;
    VectorXd b1 = VectorXd::Ones(4, 1);
    Polyhedron P1(A1, b1);
    vector<VectorXd> vertexP1 = P1.generateVertex();
    vector<double> x1(vertexP1.size() + 1), y1(vertexP1.size() + 1);
    for (VectorXd v: vertexP1) {
        cout << "x: " << v(0) << "; " << "y: " << v(1) << endl;
        x1.push_back(v(0));
        y1.push_back(v(1));
    }

    Polyhedron P2(A1, 2 * b1);
    vector<VectorXd> vertexP2 = P2.generateVertex();
    vector<double> x2(vertexP2.size() + 1), y2(vertexP2.size() + 1);
    for (VectorXd v: vertexP2) {
        cout << "x: " << v(0) << "; " << "y: " << v(1) << endl;
        x2.push_back(v(0));
        y2.push_back(v(1));
    }
    if (P2.isSubset(P1)) cout << "P1 is a subset of P2" << endl;
    else cout << "P1 is not a subset of P2" << endl;
    //P2.intersect(P1);
    if (P1.isEqual(P1)) cout << "P1 == P2" << endl;
    else cout << "P1 != P2" << endl;

    P2.appendDimensions(P1);
    vector<VectorXd> vertexP3 = P2.generateVertex();
    cout << "appendDimensions: " << endl;
    for (VectorXd v: vertexP3) {
        for (int i = 0; i < v.size(); i++) {
            cout << v(i) << ' ';
        }
        cout << endl;
    }
    vector<int> proj(4, 0);
    proj[0] = 1;
    proj[1] = 1;
    proj[2] = 1;
    P2.projection(proj);
    vector<VectorXd> vertexP4 = P2.generateVertex();
    cout << "projection: " << endl;
    for (VectorXd v: vertexP4) {
        for (int i = 0; i < v.size(); i++) {
            cout << v(i) << ' ';
        }
        cout << endl;
    }

}