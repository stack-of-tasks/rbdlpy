import sys
sys.path.append('/home/skumar/src/rbdlpy/build')
sys.path.append('/home/skumar/src/eigenpy/build')

import numpy as np
import libmodel as md
import libjoint as jt
import libbody as bd
import libdynamics as dm
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

dof = m.dof_count
Q = np.zeros(dof)	# Zero vector for joint position
QDot = np.zeros(dof)	# Zero vector for joint velocity
QDDot = np.zeros(dof)	# Zero vector for joint acceleration
Tau = np.zeros(dof)	# Zero vector for joint Torques
# Wrapper function for ForwardDynamics. It returns the matrix QDDot instead of taking it as an reference argument. Please note that the function signature has been modified. However, function name remains the same. Check RBDL documentation.
QDDot = dm.ForwardDynamics(m, Q, QDot, Tau)
print QDDot
