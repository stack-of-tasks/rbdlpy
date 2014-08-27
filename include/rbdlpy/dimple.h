/*
 * Copyright 2014, Shivesh KUMAR, LAAS-CNRS(shivesh.mecheng@gmail.com)
   Advisor: Nicolas MANSARD, LAAS-CNRS(nmansard@laas.fr)
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

#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT 

#include "rbdlpy/rbdlpy.hpp"
#include <boost/python.hpp>

// Thin wrapper function for CalcPointJacobian. It returns the matrix G. Please note that the function signature has been modified. However, function name remains the same.
RigidBodyDynamics::Math::MatrixNd CalcPointJacobian(RigidBodyDynamics::Model model, const RigidBodyDynamics::Math::VectorNd Q, unsigned int body_id, const RigidBodyDynamics::Math::Vector3d point_position, bool update_kinematics=true);

// Thin wrapper function for InverseKinematics.
RigidBodyDynamics::Math::VectorNd InverseKinematics(RigidBodyDynamics::Model model, const RigidBodyDynamics::Math::VectorNd Qinit, const std::vector< unsigned int > body_id, const std::vector< RigidBodyDynamics::Math::Vector3d > body_point, const std::vector< RigidBodyDynamics::Math::Vector3d > target_pos, double step_tol=1.0e-12, double lambda=0.01, unsigned int max_iter=50);

#endif /* _KINEMATICS_H */
