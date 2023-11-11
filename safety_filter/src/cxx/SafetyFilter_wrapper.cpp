#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "Polyhedron.h"
#include "Safety_Filter.h"

namespace py = pybind11;
namespace sf = safety_filter;

PYBIND11_MODULE(SafetyFilter_wrapper, m) {
    m.doc() = "python bindings for SafetyFilter";

    py::class_<sf::Polyhedron>(m, "Polyhedron")
        .def(py::init())
        .def(py::init<Eigen::MatrixXd, Eigen::VectorXd>())
        .def(py::init<Eigen::MatrixXd>())
        .def("getA", &sf::Polyhedron::getA)
        .def("getB", &sf::Polyhedron::getB)
        .def("getDimNums", &sf::Polyhedron::getDimNums)
        .def("getConNums", &sf::Polyhedron::getConNums)
        .def("getVertices", &sf::Polyhedron::getVertices)
        .def("printAB", &sf::Polyhedron::printAB)
        .def("addConstraints", &sf::Polyhedron::addConstraints)
        .def("intersect", &sf::Polyhedron::intersect)
        .def("addDimensions", &sf::Polyhedron::addDimensions)
        .def("invAffineMap", &sf::Polyhedron::invAffineMap)
        .def("projection", &sf::Polyhedron::projection)
        .def("isContain", &sf::Polyhedron::isContain)
        .def("isSubset", &sf::Polyhedron::isSubset)
        .def("isEqual", &sf::Polyhedron::isEqual)
        .def("minHRep", &sf::Polyhedron::minHRep)
        ;

    py::class_<sf::Safety_Filter>(m, "Safety_Filter")
        .def(py::init<Eigen::MatrixXd, Eigen::MatrixXd>())
        .def("set_state_constrains", &sf::Safety_Filter::set_state_constrains)
        .def("set_input_constrains", &sf::Safety_Filter::set_input_constrains)
        .def("set_rho", &sf::Safety_Filter::set_rho)
        .def("get_state_constrains", &sf::Safety_Filter::get_state_constrains)
        .def("get_input_constrains", &sf::Safety_Filter::get_input_constrains)
        .def("get_rho", &sf::Safety_Filter::get_rho)
        .def("get_ReachableSet", &sf::Safety_Filter::get_ReachableSet)
        .def("get_InvariantSet", &sf::Safety_Filter::get_InvariantSet)
        .def("get_NextState", &sf::Safety_Filter::get_NextState)
        .def("get_SafeAction", &sf::Safety_Filter::get_SafeAction)
        ;
}

