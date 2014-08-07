import sys
sys.path.append('/local/skumar/software/rbdlpy/build')
sys.path.append('/local/skumar/devel/lib')

import numpy as np
import libbody as bd
#import libeigenpy

verbose = False

# Exposing the default constructor of Body class
x = bd.Body()
print x.mass
print x.centerOfMass
print x.inertia
print x.spatialInertia
print x.isVirtual

# Exposing the copy constructor of Body class
y = bd.Body(x)	# passing the python object of Body class exposed from c++
print y.mass
print y.centerOfMass
print y.inertia
print y.spatialInertia
print y.isVirtual

# Exposing the constructor for creating a body from mass, center of mass, and a 3x3 inertia matrix.
mass = 5.0
com = np.array([5., 8., 9.])	# make sure elements of array are double
inertia = np.matrix([[1., 2., 3.],[1., 2., 3.],[1., 2., 3.]])	# make sure elements of matrix are double
z = bd.Body(mass, com, inertia)
print z.mass
print z.centerOfMass
print z.inertia
print z.spatialInertia
print z.isVirtual

# Exposing the constructor for creating a body from mass, center of mass, and radii of gyration.
mass = 10.0
com = np.array([2., 8., 5.])
radiusofgyration = np.array([1., 1., 1.])
b = bd.Body(mass, com, radiusofgyration)
print b.mass
print b.centerOfMass
print b.inertia
print b.spatialInertia
print b.isVirtual

rot = np.matrix([[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]])
trans = np.array([1., 1., 1.])
# Exposing the constructor of SpatialTransform class 
st = bd.SpatialTransform(rot, trans)
# Exposing the member function Join which Joins inertial parameters of two bodies to create a composite body. 
b.Join(st, x)
print b.mass

# use of Getter and setter functions
b.mass = 56
print 'mass is manually set to', b.mass 

# Fixed Body class
fb = bd.FixedBody()
assert(fb.mass==0.0)
print fb.centerOfMass
print fb.spatialInertia
assert(fb.MovableParent==0)
print fb.ParentTransform
print fb.BaseTransform
p = fb.CreateFromBody(bd.Body())	# Problem here
