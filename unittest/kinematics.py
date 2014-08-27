import sys
sys.path.append('/local/skumar/software/rbdlpy/build')
sys.path.append('/local/skumar/devel/lib')

import numpy as np
import libmodel as md
import libjoint as jt
import libbody as bd
import libkinematics as km
import libeigenpy as eigenpy

# This example has been inspired from example named "pendulum" in rbdl.

m = md.Model()

# Set the gravity vector
m.gravity = np.array([0., -9.81, 0.])

# Defining the first body
mass = 1.0
com = np.array([0.5, 0., 0.])
radiusofgyration = np.array([1., 1., 1.])
body1 = bd.Body(mass, com, radiusofgyration)
# Defining the first joint
jointaxis1 = np.array([0., 0., 1.])
joint1 = jt.Joint(jt.JointType.JointTypeRevolute, jointaxis1)
# Defining the transfomation between base body(rootbody) and body1
rot = np.matrix([[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]])
trans = np.array([0., 0., 0.])
# Exposing the constructor of SpatialTransform class 
st = bd.SpatialTransform(rot, trans)

body1_id = m.AddBody(0, st, joint1, body1, "Link1")
print 'Body with Id', body1_id, 'added to the root body.'

# Defining the second body
mass = 1.0
com = np.array([0., 0.5, 0.])
radiusofgyration = np.array([1., 1., 1.])
body2 = bd.Body(mass, com, radiusofgyration)
# Defining the first joint
jointaxis2 = np.array([0., 0., 1.])
joint2 = jt.Joint(jt.JointType.JointTypeRevolute, jointaxis2)
# Defining the transfomation between body1 and body2
rot = np.matrix([[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]])
trans = np.array([1., 0., 0.])
# Exposing the constructor of SpatialTransform class 
st = bd.SpatialTransform(rot, trans)

body2_id = m.AddBody(body1_id, st, joint2, body2, "Link2")
print 'Body with Id', body2_id, 'added to the body with Id', body1_id

# Defining the third body
mass = 0.
com = np.array([0.5, 0., 0.])
radiusofgyration = np.array([1., 1., 1.])
body3 = bd.Body(mass, com, radiusofgyration)
# Defining the first joint
jointaxis3 = np.array([0., 0., 1.])
joint3 = jt.Joint(jt.JointType.JointTypeRevolute, jointaxis3)
# Defining the transfomation between body1 and body2
rot = np.matrix([[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]])
trans = np.array([0., 1., 0.])
# Exposing the constructor of SpatialTransform class 
st = bd.SpatialTransform(rot, trans)

body3_id = m.AddBody(body2_id, st, joint3, body3, "Link3")
print 'Body with Id', body3_id, 'added to the body with Id', body2_id

dof = m.dof_count	# DoF of the model
print 'DoF of the system is', dof
Q = np.zeros(dof)	# Zero vector for joint position
QDot = np.zeros(dof)	# Zero vector for joint velocity
QDDot = np.zeros(dof)	# Zero vector for joint acceleration
'''
km.UpdateKinematics(m, Q, QDot, QDDot)
#km.UpdateKinematicsCustom(m, Q, QDot, QDDot)	# Does not work, not so important to expose anyways!!
print km.CalcBodyToBaseCoordinates(m, Q, 1, np.array([0.5, 0., 0.]), True)
print km.CalcBaseToBodyCoordinates(m, Q, 1, np.array([0.5, 0., 0.]), True)
print km.CalcBodyWorldOrientation (m, Q, 1, True)
'''
# Wrapper function for CalcPointJacobian. It returns the matrix G instead of taking it as an reference argument. Please note that the function signature has been modified. However, function name remains the same. Check RBDL documentation.
print km.CalcPointJacobian(m, Q, 1, np.array([0.5, 0.5, 0.5]), True)
print km.CalcBodyToBaseCoordinates(m, Q, 1, np.array([0.5, 0.5, 0.5]), True)
'''
point_position = np.array([0.5, 0., 0.])
print km.CalcPointVelocity(m, Q, QDot, 1, point_position, True) 
print km.CalcPointAcceleration(m, Q, QDot, QDDot, 1, point_position, True)
'''
Qinit = Q
print 'Qinit vector:', Qinit
body_id = md.pyvec_uint()	# defining a std::vector<unsigned int>
body_id.append(3)	# Add a body_id in the vector for which we want to solve the inverse kinematics(we can add more for a tree type system)

body_point = md.pyvec_vec3d()	# defining a std::vector<Math::Vector3d>
v3d = np.array([0., 1., 0.])	# Add a body_point(or current end effector pos) in local body coordinates in the vector for solving the inverse kinematics(we can add more for a tree type system)
body_point.append(v3d)

target_pos = md.pyvec_vec3d()	# defining a std::vector<Math::Vector3d>
v3d = np.array([1., 0., 0.])	# Add a body_point(or target end effector pos) in local body coordinates in the vector for solving the inverse kinematics(we can add more for a tree type system)
target_pos.append(v3d)

Qres = np.zeros(dof)	# Output joint position vector

step_tol = 1.0e-12
lamda = 0.01
max_iter = 50
# Wrapper function for InverseKinematics. It returns the matrix Qres instead of taking it as an reference argument. Please note that the function signature has been modified. However, function name remains the same. Check RBDL documentation.
Qres = km.InverseKinematics(m, Qinit, body_id, body_point, target_pos, step_tol, lamda, max_iter) 
print Qres

#print km.foo(2,5)
