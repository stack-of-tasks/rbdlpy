import sys
sys.path.append('/local/skumar/software/rbdlpy/build')
sys.path.append('/local/skumar/devel/lib')

import numpy as np
import libmodel as md
import libjoint as jt
import libbody as bd
import libeigenpy as eigenpy
import liburdfreader as urdf
import libdynamics as dm

model_loaded = False

m = md.Model()

model_loaded = urdf.read_urdf_model("./romeo.urdf", m, False)
assert(model_loaded==True)
assert(m.dof_count==67)

dof = m.dof_count
Q = np.zeros(dof)	# Zero vector for joint position
QDot = np.zeros(dof)	# Zero vector for joint velocity
QDDot = np.zeros(dof)	# Zero vector for joint acceleration
Tau = np.zeros(dof)	# Zero vector for joint Torques

# std::vector of spatial_vector type. Contains external forces acting on the system, if any.
# NOTE: There is a root body always present in the model. So we need to enter one extra spatial vector of external force which is not taken into account for calculations but is required to use ForwardDynamics function of RBDL correctly.
fext = np.array([0., 0., 0., 0., 0., 0.])
f_ext = dm.pyvec_sv()
for i in range(0,dof):
	f_ext.append(fext)


print dm.ForwardDynamics(m, Q, QDot, Tau, f_ext).transpose()
