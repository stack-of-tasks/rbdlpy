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
struct Body_visitor : boost::python::def_visitor<Body_visitor>
{
    friend class def_visitor_access;

    template <class classT>
    void visit(classT& c) const
    {
        c   .add_property("mass", &Body_visitor::get_mMass, &Body_visitor::set_mMass)
            .add_property("centerOfMass", &Body_visitor::get_mCenterOfMass, &Body_visitor::set_mCenterOfMass)
	    .add_property("inertia", &Body_visitor::get_mInertia, &Body_visitor::set_mInertia)
	    .add_property("spatialInertia", &Body_visitor::get_mSpatialInertia, &Body_visitor::set_mSpatialInertia)
	    .add_property("isVirtual", &Body_visitor::get_mIsVirtual, &Body_visitor::set_mIsVirtual)

	    .def(init<const Body&>(args("Body"), "Copy Constructor"))	// exposing the copy constructor
	    .def(init<double,Math::Vector3d,Math::Matrix3d>(args("mass","center of mass","inertia"), "Constructor from physical parameters like mass, com, interia")) 	// exposing the constructor for creating a body from mass, center of mass, and a 3x3 inertia matrix.  
	    .def(init<const double&, const Math::Vector3d&, const Math::Vector3d&>(args("mass","center of mass","radius of gyration"), "Constructor from physical parameters  like mass, com, radii of gyration")) 	// exposing the constructor for creating a body from mass, center of mass and radii of gyration. 
	    .def("Join",&Body::Join,args("spatial_transform","second_body"),"Join a second body <second_body> at the placement specified by <spatial_transform>")
	  //  .def("reassign", &Body::operator=)
        ;
    }
	// Getter for mass
    static double get_mMass(Body& self) { return self.mMass; }
	// Setter for mass
    static void set_mMass(Body& self, const double& mass) { self.mMass = mass; }

	// Getter for COM
    static RigidBodyDynamics::Math::Vector3d get_mCenterOfMass(Body& self) { return self.mCenterOfMass; }
	//Setter for COM
    static void set_mCenterOfMass(Body& self, const Math::Vector3d& com) { self.mCenterOfMass = com; }
	// Getter for inertia
    static RigidBodyDynamics::Math::Matrix3d get_mInertia(Body& self) { return self.mInertia; }
	// Setter for inertia
    static void set_mInertia(Body& self, const Math::Matrix3d& inertia) { self.mInertia = inertia; }

	// Getter for Spatial Inertia
    static RigidBodyDynamics::Math::SpatialMatrix get_mSpatialInertia(Body& self) { return self.mSpatialInertia; }
	// Setter for Spatial Inertia
    static void set_mSpatialInertia(Body& self, const Math::SpatialMatrix spatial_inertia) { self.mSpatialInertia = spatial_inertia; }

	// Getter for mIsVirtual
    static  bool get_mIsVirtual(Body& self){ return self.mIsVirtual; }
	// Setter for mIsVirtual
    static void set_mIsVirtual(Body& self, bool isvirtual){ self.mIsVirtual = isvirtual; }
};

// Curiously Recursive Template Patern (CRTP) for exposing FixedBody class
struct FixedBody_visitor : boost::python::def_visitor<FixedBody_visitor>
{
    friend class def_visitor_access;

    template <class classT>
    void visit(classT& c) const
    {
        c   .add_property("mass", &FixedBody_visitor::get_mMass, &FixedBody_visitor::set_mMass)
            .add_property("centerOfMass", &FixedBody_visitor::get_mCenterOfMass, &FixedBody_visitor::set_mCenterOfMass)
	    .add_property("spatialInertia", &FixedBody_visitor::get_mSpatialInertia, &FixedBody_visitor::set_mSpatialInertia)
	    .add_property("MovableParent", &FixedBody_visitor::get_mMovableParent, &FixedBody_visitor::set_mMovableParent)
	    .add_property("ParentTransform", &FixedBody_visitor::get_mParentTransform, &FixedBody_visitor::set_mParentTransform)
	    .add_property("BaseTransform", &FixedBody_visitor::get_mBaseTransform, &FixedBody_visitor::set_mBaseTransform)

	    .def("CreateFromBody",&FixedBody::CreateFromBody,args("Body"),"Create a fixed body")
        ;
    }
	// Getter for mass
    static double get_mMass(FixedBody& self) { return self.mMass; }
	// Setter for mass
    static void set_mMass(FixedBody& self, const double& mass) { self.mMass = mass; }

	// Getter for COM
    static RigidBodyDynamics::Math::Vector3d get_mCenterOfMass(FixedBody& self) { return self.mCenterOfMass; }
	//Setter for COM
    static void set_mCenterOfMass(FixedBody& self, const Math::Vector3d& com) { self.mCenterOfMass = com; }

	// Getter for Spatial Inertia
    static RigidBodyDynamics::Math::SpatialMatrix get_mSpatialInertia(FixedBody& self) { return self.mSpatialInertia; }
	// Setter for Spatial Inertia
    static void set_mSpatialInertia(FixedBody& self, const Math::SpatialMatrix spatial_inertia) { self.mSpatialInertia = spatial_inertia; }

	// Getter for mIsVirtual
    static  unsigned int get_mMovableParent(FixedBody& self){ return self.mMovableParent; }
	// Setter for mIsVirtual
    static void set_mMovableParent(FixedBody& self, unsigned int mMovableParent){ self.mMovableParent = mMovableParent; }

	// Getter for ParentTransform
    static RigidBodyDynamics::Math::SpatialTransform get_mParentTransform(FixedBody& self) { return self.mParentTransform; }
	// Setter for ParentTransform
    static void set_mParentTransform(FixedBody& self, const Math::SpatialTransform mParentTransform) { self.mParentTransform = mParentTransform; }

	// Getter for BaseTransform
    static RigidBodyDynamics::Math::SpatialTransform get_mBaseTransform(FixedBody& self) 
	{ 
		return self.mBaseTransform; 
	}
	// Setter for BaseTransform
    static void set_mBaseTransform(FixedBody& self, const Math::SpatialTransform mBaseTransform) 		{ 
		self.mBaseTransform = mBaseTransform; 
	}
/*
	static FixedBody CreateFromBody( FixedBody& self,const Body &b)
	{
		Body copy = b;
		return	self.CreateFromBody(copy);
	}
*/
};


/*
class Foo
{
private:
    int _bar;
public:
    Foo& operator=( const Foo& other )
    {
        _bar = other._bar;
        return *this;
    }
};
*/
BOOST_PYTHON_MODULE(libbody)
{
  eigenpy::enableEigenPy();

  eigenpy::enableEigenPySpecific<RigidBodyDynamics::Math::Vector3d, Eigen::Vector3d>();
  eigenpy::enableEigenPySpecific<RigidBodyDynamics::Math::Matrix3d, Eigen::Matrix3d>();
  eigenpy::enableEigenPySpecific<RigidBodyDynamics::Math::SpatialMatrix, Eigen::Matrix<double, 6, 6> >();

  class_<RigidBodyDynamics::Math::SpatialTransform>("SpatialTransform", init<>())  
	.def(init<Math::Matrix3d, Math::Vector3d>());	// exposing the constructor for storing a spatial transform built by rotation matrix and translation vector
  //docstring_options doc_options(true);
  class_<Body>("Body", init<>())
        .def(Body_visitor())		
	;
  class_<FixedBody>("FixedBody")
        .def(FixedBody_visitor())		
	;
 // class_<Foo>("Foo").def("assign",&Foo::operator=);
}

