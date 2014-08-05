import sys
sys.path.append('/home/skumar/src/rbdlpy/build')
sys.path.append('/home/skumar/src/eigenpy/build')

import numpy as np
import libmodel as md
import libjoint as jt
import libbody as bd
import libdynamics as dm
import libeigenpy as eigenpy
import csv

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
com = np.array([0.5, 0., 0.])
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

dof = m.dof_count
Q = np.zeros(dof)	# Zero vector for joint position
QDot = np.zeros(dof)	# Zero vector for joint velocity
QDDot = np.zeros(dof)	# Zero vector for joint acceleration
Tau = np.zeros(dof)	# Zero vector for joint Torques

# std::vector of spatial_vector type. Contains external forces acting on the system, if any.
fext = np.array([0., 0., 0., 0., 0., 0.])
f_ext = dm.pyvec_sv()
f_ext.append(fext)

# Euler Integrator
dt = 0.1;		# Time step 0.1 s
t = 0.0;
Ts = 10.0;		# Total simulation time(Ts)

f = open('/home/skumar/src/rbdlpy/python/test.csv', 'w')
fw = csv.writer(f, delimiter=',')

while t<=Ts:
	QDDot = dm.ForwardDynamics(m, Q, QDot, Tau, f_ext).transpose()
	print t, 'QDDot', QDDot
	QDot = QDot + QDDot * dt
	#print 'QDot', QDot
	Q = Q + QDot * dt
	#print t, ",", 'Q', Q
	t = t + dt
	q = Q.transpose()
	#if q[0]<=100 and q[1]<=100:  
	data = [t, q[0], q[1]]
	fw.writerow(data)
f.close()
