/*
 * Copyright 2014, Shivesh KUMAR, LAAS-CNRS
 *
 * This file is part of rbdlpy.
 * eigenpy is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * eigenpy is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with eigenpy.  If not, see <http://www.gnu.org/licenses/>.
 */

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT 

#include "rbdlpy/rbdlpy.hpp"
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <iomanip> 
using namespace boost::python;
using namespace std;
using namespace eigenpy;

#include "boost/foreach.hpp"


// Thin wrapper function for ForwardDynamics. It returns the matrix QDDot. Please note that the function signature has been modified. However, function name remains the same.
RigidBodyDynamics::Math::VectorNd ForwardDynamics(RigidBodyDynamics::Model model, const RigidBodyDynamics::Math::VectorNd Q, const RigidBodyDynamics::Math::VectorNd QDot, const RigidBodyDynamics::Math::VectorNd Tau, std::vector< RigidBodyDynamics::Math::SpatialVector > f_ext)
{
	RigidBodyDynamics::Math::VectorNd QDDot = RigidBodyDynamics::Math::VectorNd::Zero (model.dof_count);
	std::vector< RigidBodyDynamics::Math::SpatialVector > *fext;
	fext = &f_ext;
	RigidBodyDynamics::ForwardDynamics (model, Q, QDot, Tau, QDDot, fext);
	return QDDot;
}

// Thin wrapper function for ForwardDynamicsLagrangian. It returns the matrix QDDot. Please note that the function signature has been modified. However, function name remains the same.
RigidBodyDynamics::Math::VectorNd ForwardDynamicsLagrangian (RigidBodyDynamics::Model model, const RigidBodyDynamics::Math::VectorNd Q, const RigidBodyDynamics::Math::VectorNd QDot, const RigidBodyDynamics::Math::VectorNd Tau, RigidBodyDynamics::Math::LinearSolver linear_solver, std::vector< RigidBodyDynamics::Math::SpatialVector > f_ext)
{
	RigidBodyDynamics::Math::VectorNd QDDot = RigidBodyDynamics::Math::VectorNd::Zero (model.dof_count);
	std::vector< RigidBodyDynamics::Math::SpatialVector > *fext;
	fext = &f_ext;
	RigidBodyDynamics::ForwardDynamicsLagrangian (model, Q, QDot, Tau, QDDot, linear_solver, fext);
	return QDDot;
}

// Thin wrapper function for InverseDynamics. It returns the matrix Tau. Please note that the function signature has been modified. However, function name remains the same.
RigidBodyDynamics::Math::VectorNd InverseDynamics (RigidBodyDynamics::Model model, const RigidBodyDynamics::Math::VectorNd Q, const RigidBodyDynamics::Math::VectorNd QDot, const RigidBodyDynamics::Math::VectorNd QDDot, std::vector< RigidBodyDynamics::Math::SpatialVector > f_ext)
{
	RigidBodyDynamics::Math::VectorNd Tau = RigidBodyDynamics::Math::VectorNd::Zero (model.dof_count);
	std::vector< RigidBodyDynamics::Math::SpatialVector > *fext;
	fext = &f_ext;
	RigidBodyDynamics::InverseDynamics (model, Q, QDot, QDDot, Tau, fext );
	return Tau;
}

RigidBodyDynamics::Math::MatrixNd CompositeRigidBodyAlgorithm (RigidBodyDynamics::Model model, const RigidBodyDynamics::Math::VectorNd Q, bool update_kinematics)
{
	RigidBodyDynamics::Math::MatrixNd H = RigidBodyDynamics::Math::MatrixNd::Zero (model.dof_count, model.dof_count);
	CompositeRigidBodyAlgorithm (model, Q, H, update_kinematics);
	return H;
}

BOOST_PYTHON_MODULE(libdynamics)
{
	eigenpy::enableEigenPy();

	eigenpy::enableEigenPySpecific<RigidBodyDynamics::Math::Vector3d, Eigen::Vector3d>();
	eigenpy::enableEigenPySpecific<RigidBodyDynamics::Math::Matrix3d, Eigen::Matrix3d>();
	eigenpy::enableEigenPySpecific<RigidBodyDynamics::Math::SpatialMatrix, Eigen::Matrix<double, 6, 6> >();
	eigenpy::enableEigenPySpecific<RigidBodyDynamics::Math::VectorNd, Eigen::VectorXd>();
	eigenpy::enableEigenPySpecific<RigidBodyDynamics::Math::SpatialVector, Eigen::Matrix<double, 6, 1> >();

	class_<RigidBodyDynamics::Math::SpatialTransform>("SpatialTransform", init<>())  
	.def(init<RigidBodyDynamics::Math::Matrix3d, RigidBodyDynamics::Math::Vector3d>());	// exposing the constructor for storing a spatial transform built by rotation matrix and translation vector

	class_<std::vector<RigidBodyDynamics::Math::SpatialVector> >("pyvec_sv")
            .def(vector_indexing_suite<std::vector<RigidBodyDynamics::Math::SpatialVector> >());
	
	  // Exposing the enum
  	enum_<RigidBodyDynamics::Math::LinearSolver>("LinearSolver")
        .value("LinearSolverUnknown", RigidBodyDynamics::Math::LinearSolverUnknown)
        .value("LinearSolverPartialPivLU", RigidBodyDynamics::Math::LinearSolverPartialPivLU)
        .value("LinearSolverColPivHouseholderQR", RigidBodyDynamics::Math::LinearSolverColPivHouseholderQR)
	.value("LinearSolverLast", RigidBodyDynamics::Math::LinearSolverLast)
	.export_values()
	;

	//Exposing the functions of Dynamics module.
	def("ForwardDynamics", ForwardDynamics, args("model", "Q", "Qdot", "Tau", "f_ext"),"Computes forward dynamics with the Articulated Body Algorithm.");
	def("ForwardDynamicsLagrangian", ForwardDynamicsLagrangian, args("model", "Q", "Qdot", "Tau", "linear_solver", "f_ext"),"Computes forward dynamics by building and solving the full Lagrangian equation. ");
	def("InverseDynamics", InverseDynamics, args("model", "Q", "Qdot", "Tau", "f_ext"),"Computes inverse dynamics with the Newton-Euler Algorithm. ");
	def("CompositeRigidBodyAlgorithm", CompositeRigidBodyAlgorithm, args("model", "Q", "update_kinematics"), "Computes the joint space inertia matrix by using the Composite Rigid Body Algorithm.");
}

