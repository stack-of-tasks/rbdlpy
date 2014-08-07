import sys
sys.path.append('/local/skumar/software/rbdlpy/build')
sys.path.append('/local/skumar/devel/lib')

import numpy as np
import libmodel as md
import libjoint as jt
import libbody as bd
import libeigenpy as eigenpy
import liburdfreader as urdf

model_loaded = False

m = md.Model()

model_loaded = urdf.read_urdf_model("./romeo.urdf", m, False)
assert(model_loaded==True)
assert(m.dof_count==67)
