#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <eigenpy/eigenpy.hpp>

#include <triple_contact_planner/solver/qp_solver.h>

BOOST_PYTHON_MODULE(python_wrapper_cpp)
{
    namespace bp = boost::python;
    eigenpy::enableEigenPy();
    
    bp::class_<contact_planner::QpSolver, boost::noncopyable>("QpSolver")
        .def("qp_solve", &contact_planner::QpSolver::qp_solve)
        .def("qp_solve_one_body", &contact_planner::QpSolver::qp_solve_one_body)
        .def("get_result_force", &contact_planner::QpSolver::get_result_force)
        ;
}
