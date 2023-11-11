add_library(Polyhedron SHARED Polyhedron.cpp Polyhedron.h cdd_interface.h)
target_include_directories(Polyhedron PUBLIC ${CDD_INCLUDE_DIR})
target_link_libraries(Polyhedron PUBLIC ${CDD_LIBRARY})

if(TARGET Eigen3::Eigen)
    target_link_libraries(Polyhedron PUBLIC Eigen3::Eigen)
else()
    target_include_directories(Polyhedron PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()

add_library(SafetyFilter SHARED Safety_Filter.cpp Safety_Filter.h)
target_link_libraries(SafetyFilter PUBLIC Polyhedron OsqpEigen::OsqpEigen)

add_executable(test_inv2D test/test_inv2D.cpp)
target_include_directories(test_inv2D PRIVATE "/home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/src/cxx")
target_link_libraries(test_inv2D PRIVATE SafetyFilter Polyhedron)

add_executable(test_inv3D test/test_inv3D.cpp)
target_include_directories(test_inv3D PRIVATE "/home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/src/cxx")
target_link_libraries(test_inv3D PRIVATE SafetyFilter Polyhedron)

add_executable(test_QP test/test_QP.cpp)
target_include_directories(test_QP PRIVATE "/home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/src/cxx")
target_link_libraries(test_QP PRIVATE SafetyFilter Polyhedron)

add_executable(test_inv4D test/test_inv4D.cpp)
target_include_directories(test_inv4D PRIVATE "/home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/src/cxx")
target_link_libraries(test_inv4D PRIVATE SafetyFilter Polyhedron)

add_executable(test_inv4D_2 test/test_inv4D_2.cpp)
target_include_directories(test_inv4D_2 PRIVATE "/home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/src/cxx")
target_link_libraries(test_inv4D_2 PRIVATE SafetyFilter Polyhedron)

add_executable(test_HtoV test/test_HtoV.cpp)
target_include_directories(test_HtoV PRIVATE "/home/kaijun/code/CrowdNav_SafetyFilter/safety_filter/src/cxx")
target_link_libraries(test_HtoV PRIVATE SafetyFilter Polyhedron)

add_executable(test_cdd1 test/test_cdd1.cpp)
target_include_directories(test_cdd1 PUBLIC "${CDD_INCLUDE_DIR}")
target_link_libraries(test_cdd1 PUBLIC ${CDD_LIBRARIES})

pybind11_add_module(SafetyFilter_wrapper SafetyFilter_wrapper.cpp)
target_include_directories(SafetyFilter_wrapper PRIVATE "${Python3_NumPy_INCLUDE_DIRS}" ${Python3_INCLUDE_DIRS})
target_link_libraries(SafetyFilter_wrapper PRIVATE SafetyFilter Polyhedron)

install(TARGETS SafetyFilter Polyhedron
        EXPORT targets
        INCLUDES DESTINATION ${SAFETY_FILTER_INCLUDE_DIR}
        LIBRARY DESTINATION ${SAFETY_FILTER_LIBRARY_DIR}
        PUBLIC_HEADER DESTINATION ${SAFETY_FILTER_INCLUDE_DIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(TARGETS SafetyFilter_wrapper
        LIBRARY DESTINATION ${SAFETY_FILTER_PYTHON_DIR}/PySafetyFilter
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})