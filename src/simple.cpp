/*
 * Copyright 2014, Nicolas Mansard, LAAS-CNRS
 *
 * This file is part of eigenpy.
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

#include "rbdlpy/kinematics.h"

using namespace boost::python;
using namespace std;

// Thin wrapper function for CalcPointJacobian. It returns the matrix G. Please note that the function signature has been modified. However, function name remains the same.
RigidBodyDynamics::Math::MatrixNd CalcPointJacobian(RigidBodyDynamics::Model model, const RigidBodyDynamics::Math::VectorNd Q, unsigned int body_id, const RigidBodyDynamics::Math::Vector3d point_position, bool update_kinematics=true)
{
	RigidBodyDynamics::Math::MatrixNd G = RigidBodyDynamics::Math::MatrixNd::Zero (3, model.dof_count);
	RigidBodyDynamics::CalcPointJacobian (model, Q, body_id, point_position, G, update_kinematics=true);
	return G;
}

BOOST_PYTHON_MODULE(libsimple)
{

	eigenpy::enableEigenPy();
	eigenpy::enableEigenPySpecific<RigidBodyDynamics::Math::Vector3d, Eigen::Vector3d>();
	eigenpy::enableEigenPySpecific<RigidBodyDynamics::Math::Matrix3d, Eigen::Matrix3d>();
	eigenpy::enableEigenPySpecific<RigidBodyDynamics::Math::SpatialMatrix, Eigen::Matrix<double, 6, 6> >();
	eigenpy::enableEigenPySpecific<RigidBodyDynamics::Math::VectorNd, Eigen::VectorXd>();

	class_<RigidBodyDynamics::Math::SpatialTransform>("SpatialTransform", init<>())  
	.def(init<RigidBodyDynamics::Math::Matrix3d, RigidBodyDynamics::Math::Vector3d>());	// exposing the constructor for storing a spatial transform built by rotation matrix and translation vector

	//Exposing the functions of Kinematics module.
	def("UpdateKinematics", &RigidBodyDynamics::UpdateKinematics, args("model", "Q", "Qdot", "QDDot"),"Updates and computes velocities and accelerations of the bodies. ");
	def("UpdateKinematicsCustom", &RigidBodyDynamics::UpdateKinematicsCustom, args("model", "Q", "Qdot", "QDDot"),"Selectively updates model internal states of body positions, velocities and/or accelerations. ");
	def("CalcBodyToBaseCoordinates", &RigidBodyDynamics::CalcBodyToBaseCoordinates, args("model", "Q", "body_id", "body_point_position", "update_kinematics"),"Returns the base coordinates of a point given in body coordinates.");
	def("CalcBaseToBodyCoordinates", &RigidBodyDynamics::CalcBaseToBodyCoordinates, args("model", "Q", "body_id", "base_point_position", "update_kinematics"),"Returns the body coordinates of a point given in base coordinates.");
	def("CalcBodyWorldOrientation", &RigidBodyDynamics::CalcBodyWorldOrientation, args("model", "Q", "body_id", "update_kinematics"),"Returns the orientation of a given body as 3x3 matrix.");
	def("CalcPointJacobian", CalcPointJacobian, args("model", "Q", "body_id", "point_position", "update_kinematics"),"Computes the point jacobian for a point on a body.");
	def("CalcPointVelocity", &RigidBodyDynamics::CalcPointVelocity, args("model", "Q", "QDot", "body_id", "point_position", "update_kinematics"),"Computes the velocity of a point on a body.");
	def("CalcPointAcceleration", &RigidBodyDynamics::CalcPointAcceleration, args("model", "Q", "QDot", "QDDot", "body_id", "point_position", "update_kinematics"),"Computes the acceleration of a point on a body.");
	def("InverseKinematics", InverseKinematics, args("model", "Qinit", "body_id_vector", "body_point_vector", "target_pos_vector", "step_tolerance", "lambda", "max_iter"),"Computes the inverse kinematics iteratively using a damped Levenberg-Marquardt method (also known as Damped Least Squares method)");

}
