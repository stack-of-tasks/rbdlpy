import sys
sys.path.append('/local/skumar/software/rbdlpy/build')
sys.path.append('/local/skumar/devel/lib')
import math
import numpy as np
import libmodel as md
import libjoint as jt
import libbody as bd
import libkinematics as km
import libdynamics as dm
import libeigenpy as eigenpy
import libluamodel as lua
import csv
from subprocess import call
from cvxopt import matrix, solvers

model_loaded = False

m = md.Model()

model_loaded = lua.LuaModelReadFromFile("/local/skumar/software/meshup/models/double-pendulum.lua", m, False)
assert(model_loaded==True)
print 'DoF of the system:', m.dof_count

dof = m.dof_count
Q = np.zeros((1,dof))	# Zero vector for joint position
QDot = np.zeros((1,dof))	# Zero vector for joint velocity
QDDot = np.zeros((1,dof))	# Zero vector for joint acceleration
Tau = np.zeros((1,dof))	# Zero vector for joint Torques

EE_pos = np.array([0., 0., 0.6])	# End effector position in Body2 coordinates
sphere_pos = np.array([1., 0., 0.])	# Center of sphere obstacle in Base coordinates
rad_sphere = 0.3/2			# Radius of obstacle sphere
rad_EE = 0.15/2				# Radius of EE sphere
sum_of_radius = rad_sphere + rad_EE		# Sum of radii of 2 spheres

# std::vector of spatial_vector type. Contains external forces acting on the system, if any.
# NOTE: There is a root body always present in the model. So we need to enter one extra spatial vector of external force which is not taken into account for calculations but is required to use ForwardDynamics function of RBDL correctly.
fext1 = np.array([0., 0., 0., 0., 0., 0.])
fext2 = np.array([0., 0., 0., 0., 0., 0.])
fext3 = np.array([0., 0., 0., 0., 0., 0.])
f_ext = dm.pyvec_sv()
f_ext.append(fext1)
f_ext.append(fext2)
f_ext.append(fext3)

# Euler Integrator
dt = 0.001;		# Time step 0.1 s
t = 0.0;
Ts = 5.0;		# Total simulation time(Ts)

f = open('/local/skumar/software/meshup/output-double-pendulum.csv', 'w')
fw = csv.writer(f, delimiter=',')
header_data = ['COLUMNS:']
fw.writerow(header_data)
header_data = ['Time', ' Link1:R:y:r', ' Link2:R:y:r']	# Use the convention for creating COLUMNS from https://bitbucket.org/MartinFelis/meshup
fw.writerow(header_data)
header_data = ['DATA:']
fw.writerow(header_data)

while t<=Ts:
	# Solving Direct Geometric Model in Base coordinates
	X = km.CalcBodyToBaseCoordinates(m, Q, 2, EE_pos, True)
	dist = np.linalg.norm(X.transpose()-sphere_pos)
	if (dist-sum_of_radius)<0.0001:
		print 'Collision Detected'
		# Unit vector of Normal in Base coordinates
		nW1 = X.transpose() - sphere_pos
		unit_nW1 = nW1/dist
		print 'Unit Normal vector:', unit_nW1
		# Contact points on 2 surfaces in Base coordinates
		contact_point1 = sphere_pos + (rad_sphere * unit_nW1)	# C - on obstacle
		contact_point2 = X.transpose() - (rad_EE * unit_nW1)	# C' - on EE
		print 'Contact Point(on fixed obstacle):', contact_point1
		print 'Contact Point(on EE sphere):', contact_point2
		# Contact point2(i.e on EE sphere) in body coordinates
		cp2_body = km.CalcBaseToBodyCoordinates(m, Q, 2, contact_point2, True)
		print 'Contact Point(on EE sphere) in Body coordinates', cp2_body
		# Jacobian of predecessor body
		Jp = km.CalcPointJacobian(m, Q, 2, cp2_body, True)
		print 'Jacobian of predecessor body:', Jp, 'and its rank:', np.linalg.matrix_rank(Jp)
		# NOTE: Jacobian of successor body will be zero as it is fixed.
		Js = 0
		J = np.dot(unit_nW1, (Js - Jp))
		print 'J multiplied with normal:', J
		# Joint Space Inertia Matrix(M) of the system computed with Composite Rigid Body Algorithm
		M = dm.CompositeRigidBodyAlgorithm (m, Q, False)
		print 'Joint Space Inertia Matrix of the system:', M
		# Bias force: b = Tau - M*qdd
		b = np.subtract( Tau.transpose(), np.dot(M, QDDot.transpose()) )
		print 'Bias force:', b
		'''
		Consider the following standard quadratic program:
		min_x 0.5*(x.transpose)*A*x + (B.transpose)*x
		subject to C x = d
		           E x <= f

		Our quadratic programming problem:
		min_qdd	0.5*|| qddot - qddot_free ||^2*M
		such that: J*qddot>=0

		where qddot_free = M.inv * (Tau - bias)
		and we know that: M*qddot + bias = Tau + (J.transpose) * f
		Lets say: x = qddot - qddot_free
		Since, qddot = x + qddot_free, we can rewrite the inequality constrain equation as:
		J*(x + qddot_free) >= 0
		or, -J*x <= J*qddot_free

		Hence,	
		A = M
		b = 0
		C = 0
		d = 0
		E = -J
		f = J * qddot_free

		According to the CVXOPT solver, QP problem is formulated as:
		min_x 0.5 * (x.transpose) * P * x + (p.transpose) * x
		subject to A x = b
		           G x <= h
		Function signature in Python:
		sol=solvers.qp(P, p, G, h, A, b)

		Hence, in our case:
		P = M
		p = [0. 0.]
		G = -J
		h = J * qddot_free
		sol=solvers.qp(P, p, G, h)
		There is no equality constraint in our problem. Since, A and b are optional parameters to the function, we can omit them. 
		'''
		# Quadratic Programming Implementation
		P = matrix(M)
		p = matrix([0., 0.])
		G = matrix(-J)
		h = matrix( np.dot(J,QDDot.transpose()) ) 
		print 'h=', h
		sol=solvers.qp(P, p, G, h)	# Neglecting the parameters of equality constraints
		print 'soln:', sol['x']
		# qdd is the joint acceleration after collision and QDDot is the free joint acceleration(i.e. without collision).
		qdd = sol['x'] + QDDot.transpose()
		print 'qdd soln:', qdd
		Tau_c = np.dot(M, qdd) + b - Tau.transpose()
		print 'Torque due to contact:', Tau_c
		print 'Pseudo Inverse of Jacobian', np.linalg.pinv(J.transpose())  
		print 'Contact force:', np.dot(np.linalg.pinv(J.transpose()), Tau_c)
		QDDot = qdd.transpose()	# to see the effect in the environment
		QDot = QDot + QDDot * dt
		Q = Q + QDot * dt
		t = t + dt
		print t, 'QDDot', QDDot
		# Point velocity of C' in Base coordinates
		point_vel = km.CalcPointVelocity(m, Q, QDot, 2, cp2_body, True)
		print 'Velocity of contact point of EE:', point_vel.transpose()
		# Contact seperation velocity
		zeta = np.dot(unit_nW1, point_vel)	# should be zero for a good collision.
		print 'Contact seperation velocity:', zeta
		# Point acceleration of C' in Base coordinates
		point_accn = km.CalcPointAcceleration(m, Q, QDot, QDDot, 2, cp2_body, True)
		print 'Acceleration of contact point of EE:', point_accn.transpose()
		# Contact seperation acceleration
		zeta_dot = np.dot(unit_nW1, point_accn)	# should be >=0 for a good collision.
		print 'Contact seperation acceleration:', zeta_dot
		#break
	else:
		QDDot = dm.ForwardDynamics(m, Q, QDot, Tau, f_ext).transpose()
		print t, 'QDDot', QDDot
		QDot = QDot + QDDot * dt
		Q = Q + QDot * dt
		t = t + dt
	q = Q.transpose()
	data = [t, str(q[0]).replace('[',' ').replace(']',''), str(q[1]).replace('[',' ').replace(']','')]
	fw.writerow(data)
f.close()

# Calling the meshup program in Python to visualize the animation.
call(["meshup", "/local/skumar/software/meshup/models/double-pendulum.lua", "/local/skumar/software/meshup/output-double-pendulum.csv"])


