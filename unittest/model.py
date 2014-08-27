import sys
sys.path.append('/local/skumar/software/rbdlpy/build')
sys.path.append('/local/skumar/devel/lib')


import numpy as np
import libmodel as md
import libjoint as jt
import libbody as bd
import libeigenpy as eigenpy

m = md.Model()

#Test
p = md.pyvec_uint()
p.append(0)	# Add a data entry(type-unsigned int) in the vector
p.append(1)
print p[0], p[1]
##

#Test2
v = md.pyvec_uintvec()
v.append(p)	# Add a data entry(type-unsigned int vector) in the vector
print 'v=', v[0][0], ',', v[0][1]
##

#Test3
v = md.pyvec_vec3d()
v3d1 = np.array([0., 1., 0.])
v3d2 = np.array([1., 0., 0.])
v.append(v3d1)	# Add a data entry(type-unsigned int vector) in the vector
v.append(v3d2)
##

print 'main output'
print m.lamda[0]	
print m.mu.__getitem__(0)	# Okay but how to print this object?
print m.dof_count
print m.q_size
print m.qdot_size
print m.previously_added_body_id
print m.gravity
print m.v	
print m.a	
print m.S

# Note: mJoints is a std::vector of type Joint. getJoint(index) function can help in retreiving the joint at particular index in the std::vector. 
try:
	j = m.getJoint(0)
except eigenpy.exception,e:
	print 'As expected, catch the following exception: ',e.message,' (Dont worry)'
print j.JointType	

# Note: X_T(Transformations from the parent body to the frame of the joint) is a std::vector of type Math::SpatialTransform. getX_T(index) function can help in retreiving the X_T at particular index in the std::vector. 
try:
	xt = m.getX_T(0)
except eigenpy.exception,e:
	print 'As expected, catch the following exception: ',e.message,' (Dont worry)'
#print xt.E	# Cannot print because SpatialTransform class has not yet been exposed.

print m.FixedJointCount
print m.multdof3_S
print m.multdof3_U
print m.multdof3_Dinv
print m.multdof3_u
print m.multdof3_w_index
print m.c
print m.IA
print m.pA
print m.U
print m.d
print m.u
print m.f

# Note: Ic(The spatial inertia of body i) is a std::vector of type Math::SpatialRigidBodyInertia. getIc(index) function can help in retreiving the Ic at particular index in the std::vector. 
try:
	ic = m.getIc(0)
except eigenpy.exception,e:
	print 'As expected, catch the following exception: ',e.message,' (Dont worry)'
#print ic.m	# Cannot print because SpatialRigidBodyInertia class has not yet been exposed.

print m.hc

# Note: X_lambda(Transformation from the parent body to the current body) is a std::vector of type Math::SpatialTransform. getX_lambda(index) function can help in retreiving the X_lambda at particular index in the std::vector. 
try:
	xl = m.getX_lambda(0)
except eigenpy.exception,e:
	print 'As expected, catch the following exception: ',e.message,' (Dont worry)'
#print xl.E	# Cannot print because SpatialTransform class has not yet been exposed.

# Note: X_base(Transformation from the base to bodies reference frame.) is a std::vector of type Math::SpatialTransform. getX_base(index) function can help in retreiving the X_base at particular index in the std::vector. 
try:
	xb = m.getX_base(0)
except eigenpy.exception,e:
	print 'As expected, catch the following exception: ',e.message,' (Dont worry)'
#print xb.E	# Cannot print because SpatialTransform class has not yet been exposed.

'''
# Note: mFixedBodies(All bodies that are attached to a body via a fixed joint.) is a std::vector of type RigidBodyDynamics::FixedBody. getFixedBody(index) function can help in retreiving the FixedBody at particular index in the std::vector. Here, binding is okay but there is no initialization of mFixedBodies variable in the Model() constructor and hence we get the segmentation fault.
try:
	fb = m.getFixedBody(0)
except eigenpy.exception,e:
	print 'As expected, catch the following exception: ',e.message,' (Dont worry)'
print fb.mass
'''

print m.fixed_body_discriminator	# Supposed to be garbage because it is not initialized in constructor

# Note: mBodies is a std::vector of type Body. getBody(index) function can help in retreiving the Body at particular index in the std::vector. 
try:
	b = m.getBody(0)
except eigenpy.exception,e:
	print 'As expected, catch the following exception: ',e.message,' (Dont worry)'
print b.mass	

print m.BodyNameMap

# Defining the first body
mass = 1.0
com = np.array([0.5, 0., 0.])
radiusofgyration = np.array([1., 1., 1.])
body1 = bd.Body(mass, com, radiusofgyration)
# Defining the first joint
jointaxis1 = np.array([0., 1., 0.])
joint1 = jt.Joint(jt.JointType.JointTypeRevolute, jointaxis1)
# Defining the transfomation between base body(rootbody) and body1
rot = np.matrix([[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]])
trans = np.array([0., 0., 0.])
# Exposing the constructor of SpatialTransform class 
st = bd.SpatialTransform(rot, trans)
print m.AddBody(0, st, joint1, body1, "Link1")
print m.AppendBody(st, joint1, body1, "Link2")
print m.GetBodyId("Link1")
print m.GetBodyName(2)
st = m.GetJointFrame(0)
# print st	# For the moment, printing st is not possible. SpatialTransform class has to be exposed.
print m.GetParentBodyId(1)
print m.IsBodyId(1)
print m.IsFixedBodyId(1)
