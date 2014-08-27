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
#include <boost/python/suite/indexing/map_indexing_suite.hpp>
//#include "Model.cc"

using namespace boost::python;
using namespace std;
using namespace RigidBodyDynamics;
using namespace eigenpy;

namespace rbdlpy
{
  class ExceptionIndex : public eigenpy::exception
  {
  public:
    int index;
    ExceptionIndex(int index, int imin, int imax) : eigenpy::exception("")
    {
      std::ostringstream oss; 
      oss << "Index " << index << " out of range " << imin << " to "<< imax <<".";
      message = oss.str();
    }
  };
} // namespace



// Curiously Recursive Template Patern (CRTP) for exposing Model class
struct Model_visitor : boost::python::def_visitor<Model_visitor>
{
    friend class def_visitor_access;

    template <class classT>
    void visit(classT& c) const
    {
        c   
            .add_property("lamda", &Model_visitor::get_lambda, &Model_visitor::set_lambda, "The id of the parents body.")	
	    .add_property("mu", &Model_visitor::get_mu, &Model_visitor::set_mu, "Contains the ids of all the children of a given body.")	
	    .add_property("dof_count", &Model_visitor::get_dof_count, &Model_visitor::set_dof_count, "number of degrees of freedoms of the model")
	    .add_property("q_size", &Model_visitor::get_q_size, &Model_visitor::set_q_size, "The size of the q-vector. For models without spherical joints the value is the same as Model::dof_count, otherwise additional values for the w-component of the Quaternion is stored at the end of q. ")
	    .add_property("qdot_size", &Model_visitor::get_qdot_size, &Model_visitor::set_qdot_size, "The size of the qdot, qddot, and tau vector.")
	    .add_property("previously_added_body_id", &Model_visitor::get_previously_added_body_id, &Model_visitor::set_previously_added_body_id, "Id of the previously added body, required for Model::AppendBody()")
	    .add_property("gravity", &Model_visitor::get_gravity, &Model_visitor::set_gravity, "the cartesian vector of the gravity")	
	    .add_property("v", &Model_visitor::get_v, &Model_visitor::set_v, "The spatial velocity of the bodies. ")	
	    .add_property("a", &Model_visitor::get_a, &Model_visitor::set_a, "The spatial acceleration of the bodies. ")	
	    .add_property("S", &Model_visitor::get_S, &Model_visitor::set_S, "The joint axis for joint i. ")	
	    .add_property("FixedJointCount", &Model_visitor::get_FixedJointCount, &Model_visitor::set_FixedJointCount, "The number of fixed joints that have been declared before each joint.")	
	    .add_property("multdof3_S", &Model_visitor::get_multdof3_S, &Model_visitor::set_multdof3_S, "Motion subspace for joints with 3 degrees of freedom.")
	    .add_property("multdof3_U", &Model_visitor::get_multdof3_U, &Model_visitor::set_multdof3_U)
	    .add_property("multdof3_Dinv", &Model_visitor::get_multdof3_Dinv, &Model_visitor::set_multdof3_Dinv)
	    .add_property("multdof3_u", &Model_visitor::get_multdof3_u, &Model_visitor::set_multdof3_u)
	    .add_property("multdof3_w_index", &Model_visitor::get_multdof3_w_index, &Model_visitor::set_multdof3_w_index)
	    .add_property("c", &Model_visitor::get_c, &Model_visitor::set_c, "The velocity dependent spatial acceleration. ")
	    .add_property("IA", &Model_visitor::get_IA, &Model_visitor::set_IA, "The spatial inertia of the bodies.")
	    .add_property("pA", &Model_visitor::get_pA, &Model_visitor::set_pA, "The spatial bias force")
	    .add_property("U", &Model_visitor::get_U, &Model_visitor::set_U, "Temporary variable U_i (RBDA p. 130) ")
	    .add_property("d", &Model_visitor::get_d, &Model_visitor::set_d, "Temporary variable D_i (RBDA p. 130) ")
	    .add_property("u", &Model_visitor::get_u, &Model_visitor::set_u, "Temporary variable u (RBDA p. 130) ")
	    .add_property("f", &Model_visitor::get_f, &Model_visitor::set_f, "Internal forces on the body (used only InverseDynamics()) ")
	    .add_property("hc", &Model_visitor::get_hc, &Model_visitor::set_hc)
	    .add_property("fixed_body_discriminator", &Model_visitor::get_fixed_body_discriminator, &Model_visitor::set_fixed_body_discriminator, "Value that is used to discriminate between fixed and movable bodies. ")
	    .add_property("BodyNameMap", &Model_visitor::get_BodyNameMap, &Model_visitor::set_BodyNameMap, "Human readable names for the bodies.")

	    .def("AddBody", &Model::AddBody, args("parent_id", "joint_frame", "joint", "body", "body_name"),"Connects a given body to the model.")

		// Strange error: Discuss with Nicolas
	   // .def("AddBodySphericalJoint", &Model::AddBodySphericalJoint, args("parent_id", "joint_frame", "joint", "body", "body_name"))

	    .def("AppendBody", &Model::AppendBody, args("joint_frame", "joint", "body", "body_name"),"Adds a Body to the model such that the previously added Body is the Parent.")
	    .def("GetBodyId", &Model::GetBodyId, args("body_name"),"Returns the id of a body that was passed to AddBody()")
	    .def("GetBodyName", &Model::GetBodyName, args("body_id"),"Returns the name of a body for a given body id.")
	    .def("GetJointFrame", &Model::GetJointFrame, args("id"),"Returns the joint frame transformtion, i.e. the second argument to Model::AddBody().")
	    .def("SetJointFrame", &Model::SetJointFrame, args("id", "spatial_transform"),"Sets the joint frame transformtion, i.e. the second argument to Model::AddBody().")
	    .def("GetParentBodyId", &Model::GetParentBodyId, args("id"),"Determines id the actual parent body.")
	    .def("GetQuaternion", &Model::GetQuaternion, args("id", "Q"),"Determines id the actual parent body.")
	    .def("SetQuaternion", &Model::SetQuaternion, args("unsigned int i", "Quaternion quat", "VectorNd Q"),"Determines id the actual parent body.")
	    .def("IsBodyId", &Model::IsBodyId, args("body_id"),"Check whether its a valid body id.")
	    .def("IsFixedBodyId", &Model::IsFixedBodyId, args("body_id"),"Checks whether the body is rigidly attached to another body.")
	    .def("SetFloatingBaseBody", &Model::SetFloatingBaseBody, args("body"),"Specifies the dynamical parameters of the first body and assigns it a 6 DoF joint.")

	    //.def("reassign", &Joint::operator=)

     		.def("getJoint", &Model_visitor::getJoint)
		.def("getBody", &Model_visitor::getBody)
		.def("getX_T", &Model_visitor::getX_T)
		.def("getIc", &Model_visitor::getIc)
		.def("getX_lambda", &Model_visitor::getX_lambda)
		.def("getX_base", &Model_visitor::getX_base)
		.def("getFixedBody", &Model_visitor::getFixedBody)
	    ;
    }

	static Joint getJoint(Model& self, unsigned int index)
	 {
		if ((index<0)||(index>self.mJoints.size()) ) {
			throw rbdlpy::ExceptionIndex(index,0,self.mJoints.size());}
		 return self.mJoints[index];
	 }

	static Body getBody(Model& self, unsigned int index)
	 {
		if ((index<0)||(index>self.mBodies.size()) ) {
			throw rbdlpy::ExceptionIndex(index,0,self.mBodies.size());}
		 return self.mBodies[index];
	 }

	static Math::SpatialTransform getX_T(Model& self, unsigned int index)
	 {
		if ((index<0)||(index>self.X_T.size()) ) {
			throw rbdlpy::ExceptionIndex(index,0,self.X_T.size());}
		 return self.X_T[index];
	 }
	static Math::SpatialRigidBodyInertia getIc(Model& self, unsigned int index)
	 {
		if ((index<0)||(index>self.Ic.size()) ) {
			throw rbdlpy::ExceptionIndex(index,0,self.Ic.size());}
		 return self.Ic[index];
	 }

	static Math::SpatialTransform getX_lambda(Model& self, unsigned int index)
	 {
		if ((index<0)||(index>self.X_lambda.size()) ) {
			throw rbdlpy::ExceptionIndex(index,0,self.X_lambda.size());}
		 return self.X_lambda[index];
	 }

	static Math::SpatialTransform getX_base(Model& self, unsigned int index)
	 {
		if ((index<0)||(index>self.X_base.size()) ) {
			throw rbdlpy::ExceptionIndex(index,0,self.X_base.size());}
		 return self.X_base[index];
	 }

	static RigidBodyDynamics::FixedBody getFixedBody(Model& self, unsigned int index)
	 {
		if ((index<0)||(index>self.mFixedBodies.size()) ) {
			throw rbdlpy::ExceptionIndex(index,0,self.mFixedBodies.size());}
		 return self.mFixedBodies[index];
	 }

	// Getter for lambda
    static std::vector<unsigned int> get_lambda(Model& self)
    { 	
	return self.lambda; 
    }
	// Setter for lambda
    static void set_lambda(Model& self, const std::vector<unsigned int>  lambda) { self.lambda = lambda; }

	// Getter for mu
    static std::vector<std::vector<unsigned int> > get_mu(Model& self)
    { 
	return self.mu; 
    }
	// Setter for mu
    static void set_mu(Model& self, const std::vector<std::vector<unsigned int> > mu) { self.mu = mu; }

	// Getter for dof_count
    static unsigned int get_dof_count(Model& self) { return self.dof_count; }
	// Setter for dof_count
    static void set_dof_count(Model& self, const unsigned int& dof_count) { self.dof_count = dof_count; }

	// Getter for q_size
    static unsigned int get_q_size(Model& self) { return self.q_size; }
	// Setter for q_size
    static void set_q_size(Model& self, const unsigned int& q_size) { self.q_size = q_size; }

	// Getter for qdot_size
    static unsigned int get_qdot_size(Model& self) { return self.qdot_size; }
	// Setter for qdot_size
    static void set_qdot_size(Model& self, const unsigned int& qdot_size) { self.qdot_size = qdot_size; }
		
	// Getter for previously_added_body_id
    static unsigned int get_previously_added_body_id(Model& self) { return self.previously_added_body_id; }
	// Setter for previously_added_body_id
    static void set_previously_added_body_id(Model& self, const unsigned int& previously_added_body_id) { self.previously_added_body_id = previously_added_body_id; }	

	// Getter for gravity
    static RigidBodyDynamics::Math::Vector3d get_gravity(Model& self) { return self.gravity; }
	// Setter for gravity
    static void set_gravity(Model& self, const RigidBodyDynamics::Math::Vector3d& gravity) { self.gravity = gravity; }

	// Getter for v
    static std::vector<Math::SpatialVector> get_v(Model& self) { return self.v; }
	// Setter for v
    static void set_v(Model& self, const std::vector<Math::SpatialVector>& v) { self.v = v; }

	// Getter for a
    static std::vector<Math::SpatialVector> get_a(Model& self) { return self.a; }
	// Setter for a
    static void set_a(Model& self, const std::vector<Math::SpatialVector>& a) { self.a = a; }

	// Getter for S
    static std::vector<Math::SpatialVector> get_S(Model& self) { return self.S; }
	// Setter for S
    static void set_S(Model& self, const std::vector<Math::SpatialVector>& S) { self.S = S; }

	// Getter for FixedJointCount
    static std::vector<unsigned int> get_FixedJointCount(Model& self)
    { 	
	return self.mFixedJointCount; 
    }
	// Setter for FixedJointCount
    static void set_FixedJointCount(Model& self, const std::vector<unsigned int>  FixedJointCount) { self.mFixedJointCount = FixedJointCount; }

	// Getter for multdof3_S
    static std::vector<Math::Matrix63> get_multdof3_S(Model& self) { return self.multdof3_S; }
	// Setter for multdof3_S
    static void set_multdof3_S(Model& self, const std::vector<Math::Matrix63>& multdof3_S) { self.multdof3_S = multdof3_S; }

	// Getter for multdof3_U
    static std::vector<Math::Matrix63> get_multdof3_U(Model& self) { return self.multdof3_U; }
	// Setter for multdof3_U
    static void set_multdof3_U(Model& self, const std::vector<Math::Matrix63>& multdof3_U) { self.multdof3_U = multdof3_U; }

	// Getter for multdof3_Dinv
    static std::vector<Math::Matrix3d> get_multdof3_Dinv(Model& self) { return self.multdof3_Dinv; }
	// Setter for multdof3_Dinv
    static void set_multdof3_Dinv(Model& self, const std::vector<Math::Matrix3d>& multdof3_Dinv) { self.multdof3_Dinv = multdof3_Dinv; }

	// Getter for multdof3_u
    static std::vector<Math::Vector3d> get_multdof3_u(Model& self) { return self.multdof3_u; }
	// Setter for multdof3_u
    static void set_multdof3_u(Model& self, const std::vector<Math::Vector3d>& multdof3_u) { self.multdof3_u = multdof3_u; }

	// Getter for multdof3_w_index
    static std::vector<unsigned int> get_multdof3_w_index(Model& self) { return self.multdof3_w_index; }
	// Setter for multdof3_w_index
    static void set_multdof3_w_index(Model& self, const std::vector<unsigned int>& multdof3_w_index) { self.multdof3_w_index = multdof3_w_index; }

	// Getter for c
    static std::vector<Math::SpatialVector> get_c(Model& self) { return self.c; }
	// Setter for c
    static void set_c(Model& self, const std::vector<Math::SpatialVector>& c) { self.c = c; }

	// Getter for IA
    static std::vector<Math::SpatialMatrix> get_IA(Model& self) { return self.IA; }
	// Setter for IA
    static void set_IA(Model& self, const std::vector<Math::SpatialMatrix>& IA) { self.IA = IA; }

	// Getter for pA
    static std::vector<Math::SpatialVector> get_pA(Model& self) { return self.pA; }
	// Setter for pA
    static void set_pA(Model& self, const std::vector<Math::SpatialVector>& pA) { self.pA = pA; }

	// Getter for U
    static std::vector<Math::SpatialVector> get_U(Model& self) { return self.U; }
	// Setter for U
    static void set_U(Model& self, const std::vector<Math::SpatialVector>& U) { self.U = U; }

	// Getter for d
    static Math::VectorNd get_d(Model& self) { return self.d; }
	// Setter for d
    static void set_d(Model& self, const Math::VectorNd& d) { self.d = d; }

	// Getter for u
    static Math::VectorNd get_u(Model& self) { return self.u; }
	// Setter for u
    static void set_u(Model& self, const Math::VectorNd& u) { self.u = u; }

	// Getter for f
    static std::vector<Math::SpatialVector> get_f(Model& self) { return self.f; }
	// Setter for f
    static void set_f(Model& self, const std::vector<Math::SpatialVector>& f) { self.f = f; }

	// Getter for hc
    static std::vector<Math::SpatialVector> get_hc(Model& self) { return self.hc; }
	// Setter for hc
    static void set_hc(Model& self, const std::vector<Math::SpatialVector>& hc) { self.hc = hc; }

	// Getter for fixed_body_discriminator
    static unsigned int get_fixed_body_discriminator(Model& self) { return self.fixed_body_discriminator; }
	// Setter for fixed_body_discriminator
    static void set_fixed_body_discriminator(Model& self, const unsigned int& fixed_body_discriminator) { self.fixed_body_discriminator = fixed_body_discriminator; }

	// Getter for BodyNameMap
    static std::map<std::string, unsigned int> get_BodyNameMap(Model& self) { return self.mBodyNameMap; }
	// Setter for BodyNameMap
    static void set_BodyNameMap(Model& self, const std::map<std::string, unsigned int>& BodyNameMap) { self.mBodyNameMap = BodyNameMap; }

};

BOOST_PYTHON_MODULE(libmodel)
{
  eigenpy::enableEigenPy();

  eigenpy::enableEigenPySpecific<RigidBodyDynamics::Math::Vector3d, Eigen::Vector3d>();
  eigenpy::enableEigenPySpecific<RigidBodyDynamics::Math::Matrix3d, Eigen::Matrix3d>();
  eigenpy::enableEigenPySpecific<RigidBodyDynamics::Math::SpatialMatrix, Eigen::Matrix<double, 6, 6> >();
  eigenpy::enableEigenPySpecific<RigidBodyDynamics::Math::SpatialVector, Eigen::Matrix<double, 6, 1> >();
  eigenpy::enableEigenPySpecific<RigidBodyDynamics::Math::Matrix63, Eigen::Matrix<double, 6, 3> >();
  eigenpy::enableEigenPySpecific<RigidBodyDynamics::Math::VectorNd, Eigen::VectorXd>();

  class_<RigidBodyDynamics::Math::SpatialTransform>("SpatialTransform", init<>())  
	.def(init<Math::Matrix3d, Math::Vector3d>());	// exposing the constructor for storing a spatial transform built by rotation matrix and translation vector

  class_<RigidBodyDynamics::Math::SpatialRigidBodyInertia>("SpatialRigidBodyInertia", init<>())  
	.def(init<double, const Math::Vector3d&, const Math::Matrix3d&>());	// exposing the constructor for storing a spatial transform built by rotation matrix and translation vector

  class_<std::vector<unsigned int> >("pyvec_uint")
            .def(vector_indexing_suite<std::vector<unsigned int> >());

  class_<std::vector<std::vector<unsigned int> > >("pyvec_uintvec")
            .def(vector_indexing_suite<std::vector<std::vector<unsigned int> > >());

  class_<std::vector<RigidBodyDynamics::Math::Vector3d> >("pyvec_vec3d")
            .def(vector_indexing_suite<std::vector<RigidBodyDynamics::Math::Vector3d> >());

  class_<std::vector<RigidBodyDynamics::Math::Matrix3d> >("pyvec_mat3d")
            .def(vector_indexing_suite<std::vector<RigidBodyDynamics::Math::Matrix3d> >());

  class_<std::vector<RigidBodyDynamics::Math::SpatialVector> >("pyvec_sv")
            .def(vector_indexing_suite<std::vector<RigidBodyDynamics::Math::SpatialVector> >());

  class_<std::vector<RigidBodyDynamics::Math::SpatialMatrix> >("pyvec_sm")
            .def(vector_indexing_suite<std::vector<RigidBodyDynamics::Math::SpatialMatrix> >());

  class_<std::vector<RigidBodyDynamics::Math::Matrix63> >("pyvec_mat63")
            .def(vector_indexing_suite<std::vector<RigidBodyDynamics::Math::Matrix63> >());

  class_<std::map<std::string, unsigned int> >("pymap")
            .def(map_indexing_suite<std::map<std::string, unsigned int> >());

  // Exposing the Joint class
class_<Model>("Model", init<>())
        .def(Model_visitor())		
	;
}

