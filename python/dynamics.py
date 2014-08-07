import sys
sys.path.append('/local/skumar/software/rbdlpy/build')
sys.path.append('/local/skumar/devel/lib')


import numpy as np
import libmodel as md
import libjoint as jt
import libbody as bd
import libdynamics as dm
import libeigenpy as eigenpy

# This example has been inspired from example named "simple" in rbdl.

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
Q = np.zeros(dof)	# Zero vector for joint position
QDot = np.zeros(dof)	# Zero vector for joint velocity
QDDot = np.zeros(dof)	# Zero vector for joint acceleration
Tau = np.zeros(dof)	# Zero vector for joint Torques

# std::vector of spatial_vector type. Contains external forces acting on the system, if any.
fext = np.array([0., 0., 0., 0., 0., 0.])
f_ext = dm.pyvec_sv()
f_ext.append(fext)

# Wrapper function for ForwardDynamics. It returns the vector QDDot instead of taking it as a reference argument. Please note that the function signature has been modified. However, function name remains the same. Check RBDL documentation.
QDDot = dm.ForwardDynamics(m, Q, QDot, Tau, f_ext)
print QDDot
'''
# LinearSolver enum has been exposed. We can choose any LinearSolver as we want.
linsolver = dm.LinearSolver.LinearSolverColPivHouseholderQR

# Wrapper function for ForwardDynamicsLagrangian. It returns the vector QDDot instead of taking it as an reference argument. Please note that the function signature has been modified. However, function name remains the same. Check RBDL documentation.
print dm.ForwardDynamicsLagrangian(m, Q, QDot, Tau, linsolver, f_ext)	# Some numerical problem! Report to Eigenpy.
'''
# Wrapper function for InverseDynamics. It returns the vector Tau instead of taking it as a reference argument. Please note that the function signature has been modified. However, function name remains the same. Check RBDL documentation.
print dm.InverseDynamics (m, Q, QDot, QDDot, f_ext)

# Wrapper function for CompositeRigidBodyAlgorithm. It returns the matrix H instead of taking it as a reference argument. Please note that the function signature has been modified. However, function name remains the same. Check RBDL documentation.
update_kinematics = True
print dm.CompositeRigidBodyAlgorithm (m, Q, update_kinematics)
