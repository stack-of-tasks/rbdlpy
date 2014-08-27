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

#ifndef _DYNAMICS_H
#define _DYNAMICS_H

#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "rbdlpy/rbdlpy.hpp"
#include <rbdl/Dynamics.h>


// Thin wrapper function for ForwardDynamics. It returns the matrix QDDot. Please note that the function signature has been modified. However, function name remains the same.
RigidBodyDynamics::Math::VectorNd ForwardDynamics(RigidBodyDynamics::Model model, const RigidBodyDynamics::Math::VectorNd Q, const RigidBodyDynamics::Math::VectorNd QDot, const RigidBodyDynamics::Math::VectorNd Tau, std::vector< RigidBodyDynamics::Math::SpatialVector > f_ext)
{
	RigidBodyDynamics::Math::VectorNd QDDot = RigidBodyDynamics::Math::VectorNd::Zero (model.dof_count);
	std::vector< RigidBodyDynamics::Math::SpatialVector > *fext;
	fext = &f_ext;
	//RigidBodyDynamics::InverseDynamics (model, Q, QDot, QDDot, Tau);
	RigidBodyDynamics::UpdateKinematicsCustom (model, &Q, &QDot, &QDDot);
	return QDDot;
}

// Thin wrapper function for ForwardDynamicsLagrangian. It returns the matrix QDDot. Please note that the function signature has been modified. However, function name remains the same.
RigidBodyDynamics::Math::VectorNd ForwardDynamicsLagrangian (RigidBodyDynamics::Model model, const RigidBodyDynamics::Math::VectorNd Q, const RigidBodyDynamics::Math::VectorNd QDot, const RigidBodyDynamics::Math::VectorNd Tau, RigidBodyDynamics::Math::LinearSolver linear_solver, std::vector< RigidBodyDynamics::Math::SpatialVector > f_ext);

// Thin wrapper function for InverseDynamics. It returns the matrix Tau. Please note that the function signature has been modified. However, function name remains the same.
RigidBodyDynamics::Math::VectorNd InverseDynamics (RigidBodyDynamics::Model model, const RigidBodyDynamics::Math::VectorNd Q, const RigidBodyDynamics::Math::VectorNd QDot, const RigidBodyDynamics::Math::VectorNd QDDot, std::vector< RigidBodyDynamics::Math::SpatialVector > f_ext);

// Thin wrapper function for CompositeRigidBodyAlgorithm. It returns the matrix H(Inertia matrix). Please note that the function signature has been modified. However, function name remains the same.
RigidBodyDynamics::Math::MatrixNd CompositeRigidBodyAlgorithm (RigidBodyDynamics::Model model, const RigidBodyDynamics::Math::VectorNd Q, bool update_kinematics);

#endif /* _DYNAMICS_H */
