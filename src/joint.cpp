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

using namespace boost::python;
using namespace std;
using namespace RigidBodyDynamics;
using namespace eigenpy;

// Curiously Recursive Template Patern (CRTP) for exposing Body class
struct Joint_visitor : boost::python::def_visitor<Joint_visitor>
{
    friend class def_visitor_access;

    template <class classT>
    void visit(classT& c) const
    {
        c   
	    .def("JointAxes", &Joint_visitor::get_JointAxes,"Access on the i^th element of the joint axis.")
            .add_property("JointType", &Joint_visitor::get_JointType, &Joint_visitor::set_JointType, "Getter/Setter functions for the JointType")	
	    .add_property("DoFCount", &Joint_visitor::get_DoFCount, &Joint_visitor::set_DoFCount, "Getter/Setter functions for DoFCount")
	    .add_property("q_index", &Joint_visitor::get_q_index, &Joint_visitor::set_q_index, "Getter/Setter functions for q_index")

	    .def(init<const JointType&>(args("JointType"), "Constructor with JointType as argument"))	// exposing the constructor with JointType as argument

	    .def(init<const Joint&>(args("Joint"), "Copy Constructor"))	// exposing the copy constructor

	    .def(init<JointType,Math::Vector3d>(args("Joint Type","Joint Axis"), "Constructor from joint parameters like joint type and joint axis vector")) 	// exposing the Constructor from joint parameters like joint type and joint axis vector
 
	    .def(init<const Math::SpatialVector&>(args("Spatial Vector of axis0"), "Constructor which creates 1dof joint from spatial vector of the axis.")) 	// exposing the constructor which creates 1dof joint from spatial vector of the axis. 

	    .def(init<const Math::SpatialVector&, const Math::SpatialVector&>(args("Spatial Vector of axis0", "Spatial Vector of axis1"), "Constructor which creates 2dof joint from spatial vector of the axis0 and axis1.")) 	// exposing the constructor which creates 2dof joint from spatial vector of the axis0 and axis1. 

	    .def(init<const Math::SpatialVector&, const Math::SpatialVector&, const Math::SpatialVector&>(args("Spatial Vector of axis0", "Spatial Vector of axis1", "Spatial Vector of axis2"), "Constructor which creates 3dof joint from spatial vector of the axis0, axis1 and axis2.")) 	// exposing the constructor which creates 3dof joint from spatial vector of the axis0,axis1 and axis2. 

	    .def(init<const Math::SpatialVector&, const Math::SpatialVector&, const Math::SpatialVector&, const Math::SpatialVector&>(args("Spatial Vector of axis0", "Spatial Vector of axis1", "Spatial Vector of axis2", "Spatial Vector of axis3"), "Constructor which creates 4dof joint from spatial vector of the axis0, axis1, axis2 and axis3.")) 	// exposing the constructor which creates 4dof joint from spatial vector of the axis0, axis1, axis2 and axis3. 

	    .def(init<const Math::SpatialVector&, const Math::SpatialVector&, const Math::SpatialVector&, const Math::SpatialVector&, const Math::SpatialVector&>(args("Spatial Vector of axis0", "Spatial Vector of axis1", "Spatial Vector of axis2", "Spatial Vector of axis3", "Spatial Vector of axis4"), "Constructor which creates 5dof joint from spatial vector of the axis0, axis1, axis2, axis3 and axis4.")) 	// exposing the constructor which creates 5dof joint from spatial vector of the axis0, axis1, axis2, axis3 and axis4. 

	    .def(init<const Math::SpatialVector&, const Math::SpatialVector&, const Math::SpatialVector&, const Math::SpatialVector&, const Math::SpatialVector&, const Math::SpatialVector&>(args("Spatial Vector of axis0", "Spatial Vector of axis1", "Spatial Vector of axis2", "Spatial Vector of axis3", "Spatial Vector of axis4", "Spatial Vector of axis5"), "Constructor which creates 6dof joint from spatial vector of the axis0, axis1, axis2, axis3, axis4 and axis5.")) 	// exposing the constructor which creates 6dof joint from spatial vector of the axis0, axis1, axis2, axis3,axis4 and axis5. 

	    .def("validate_spatial_axis", &Joint_visitor::validate_spatial_axis, args("spatial vector of the axis"),"Checks whether we have pure rotational or translational axis. ")
	    //.def("reassign", &Joint::operator=)
        
;
    }

	// Getter for JointAxes
    static RigidBodyDynamics::Math::SpatialVector get_JointAxes(Joint& self,int index)
    { 
       if(self.mJointAxes == NULL) throw eigenpy::exception("The pointer mJointAxes is null.");
       return self.mJointAxes[index]; 
    }
	// Setter for JointAxes
    static void set_JointAxes(Joint& self, const RigidBodyDynamics::Math::SpatialVector& JointAxes) { *self.mJointAxes = JointAxes; }

	// Getter for JointType
    static RigidBodyDynamics::JointType get_JointType(Joint& self) { return self.mJointType; }
	//Setter for JointType
    static void set_JointType(Joint& self, const RigidBodyDynamics::JointType& JointType) { self.mJointType = JointType; }

	// Getter for DoFCount
    static unsigned int get_DoFCount(Joint& self) { return self.mDoFCount; }
	// Setter for DoFCount
    static void set_DoFCount(Joint& self, const unsigned int DoFCount) { self.mDoFCount = DoFCount; }

	// Getter for q_index
    static unsigned int get_q_index(Joint& self) { 
		if(self.q_index >= 100) throw eigenpy::exception("The q_index is some garbage value.");
		return self.q_index; 
	}
	// Setter for q_index
    static void set_q_index(Joint& self, const unsigned int q_index) { self.q_index = q_index; }
	
	static bool validate_spatial_axis( Joint& self,const RigidBodyDynamics::Math::SpatialVector &v)
	{
		RigidBodyDynamics::Math::SpatialVector copy = v;
		return	self.validate_spatial_axis(copy);
	}
};

BOOST_PYTHON_MODULE(libjoint)
{
  eigenpy::enableEigenPy();

  eigenpy::enableEigenPySpecific<RigidBodyDynamics::Math::Vector3d, Eigen::Vector3d>();
  eigenpy::enableEigenPySpecific<RigidBodyDynamics::Math::Matrix3d, Eigen::Matrix3d>();
  eigenpy::enableEigenPySpecific<RigidBodyDynamics::Math::SpatialMatrix, Eigen::Matrix<double, 6, 6> >();
  eigenpy::enableEigenPySpecific<RigidBodyDynamics::Math::SpatialVector, Eigen::Matrix<double, 6, 1> >();

  // Exposing the enum
  enum_<JointType>("JointType")
        .value("JointTypeUndefined", JointTypeUndefined)
        .value("JointTypeRevolute", JointTypeRevolute)
        .value("JointTypePrismatic", JointTypePrismatic)
	.value("JointTypeSpherical", JointTypeSpherical)
	.value("JointTypeEulerZYX", JointTypeEulerZYX)
        .value("JointType1DoF", JointType1DoF)
        .value("JointType2DoF", JointType2DoF)
	.value("JointType3DoF", JointType3DoF)
	.value("JointType4DoF", JointType4DoF)
        .value("JointType5DoF", JointType5DoF)
	.value("JointType6DoF", JointType6DoF)
	.export_values()
	;
  // Exposing the Joint class
class_<Joint>("Joint", init<>())
        .def(Joint_visitor())		
	;
}

