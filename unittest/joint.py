import sys
sys.path.append('/local/skumar/software/rbdlpy/build')
sys.path.append('/local/skumar/devel/lib')


import numpy as np
import libjoint as jt
import libeigenpy as eigenpy
'''
def isApprox(a,b,epsilon = 1e-9):
	if a.__class__ == np.ndarray && b.__class__ == np.ndarray:
		return np.allclose(a,b,epsilon)
	elif a.__class__ == b.__class__ == float:
		return abs(a-b)<epsilon
'''
# exposing enum JointType
jtype = jt.JointType()
assert(jtype.JointTypeRevolute==jt.JointType.JointTypeRevolute)

# exposing the default constructor 
j = jt.Joint()
assert(j.JointType==jt.JointType.JointTypeUndefined)
assert(j.DoFCount==0)
assert(j.q_index==0)
try:
  print j.JointAxes(0) 	
except eigenpy.exception,e:
	print 'As expected, catch the following exception: ',e.message,' (null pointer)'

# exposing the constructor which builds joint from joint type
j1 = jt.Joint(jtype.JointTypeSpherical)
assert(j1.JointType==jt.JointType.JointTypeSpherical)
assert(j1.DoFCount==3)
try:
	print j1.q_index
except eigenpy.exception,e:
	print 'As expected, catch the following exception: ',e.message,' (Dont worry)'
print j1.JointAxes(0)
print j1.JointAxes(1)
print j1.JointAxes(2)	

# exposing copy constructor
j2 = jt.Joint(j1)
assert(j2.JointType==jt.JointType.JointTypeSpherical)
assert(j2.DoFCount==3)
try:
	print j2.q_index
except eigenpy.exception,e:
	print 'As expected, catch the following exception: ',e.message,' (Dont worry)'
print j2.JointAxes(0)
print j2.JointAxes(1)
print j2.JointAxes(2)	

# exposing the Constructor from joint parameters like joint type and joint axis vector
jointaxis = np.array([0., 1., 0.])
j3 = jt.Joint(jtype.JointTypeRevolute, jointaxis)
assert(j3.JointType==jt.JointType.JointTypeRevolute)
assert(j3.DoFCount==1)
try:
	print j3.q_index
except eigenpy.exception,e:
	print 'As expected, catch the following exception: ',e.message,' (Dont worry)'
print j3.JointAxes(0)

# exposing the constructor which creates 1dof joint from spatial vector of the axis. 
axis_0 = np.array([1., 0., 0., 0., 0., 0.])
jdof1 = jt.Joint(axis_0)
assert(jdof1.JointType==jt.JointType.JointType1DoF)
assert(jdof1.DoFCount==1)
try:
	print jdof1.q_index
except eigenpy.exception,e:
	print 'As expected, catch the following exception: ',e.message,' (Dont worry)'
print jdof1.JointAxes(0)

# exposing the constructor which creates 2dof joint from spatial vector of the axis0 and axis1. 
axis_1 = np.array([0., 1., 0., 0., 0., 0.])
jdof2 = jt.Joint(axis_0, axis_1)
assert(jdof2.JointType==jt.JointType.JointType2DoF)
assert(jdof2.DoFCount==2)
try:
	print jdof2.q_index
except eigenpy.exception,e:
	print 'As expected, catch the following exception: ',e.message,' (Dont worry)'
print jdof2.JointAxes(0)
print jdof2.JointAxes(1)


# exposing the constructor which creates 3dof joint from spatial vector of the axis0, axis1 and axis2. 
axis_2 = np.array([0., 0., 1., 0., 0., 0.])
jdof3 = jt.Joint(axis_0, axis_1, axis_2)
assert(jdof3.JointType==jt.JointType.JointType3DoF)
assert(jdof3.DoFCount==3)
try:
	print jdof3.q_index
except eigenpy.exception,e:
	print 'As expected, catch the following exception: ',e.message,' (Dont worry)'
print jdof3.JointAxes(0)
print jdof3.JointAxes(1)
print jdof3.JointAxes(2)

# exposing the constructor which creates 4dof joint from spatial vector of the axis0, axis1, axis2 and axis3. 
axis_3 = np.array([0., 0., 0., 1., 0., 0.])
jdof4 = jt.Joint(axis_0, axis_1, axis_2, axis_3)
assert(jdof4.JointType==jt.JointType.JointType4DoF)
assert(jdof4.DoFCount==4)
try:
	print jdof4.q_index
except eigenpy.exception,e:
	print 'As expected, catch the following exception: ',e.message,' (Dont worry)'
print jdof4.JointAxes(0)
print jdof4.JointAxes(1)
print jdof4.JointAxes(2)
print jdof4.JointAxes(3)

# exposing the constructor which creates 5dof joint from spatial vector of the axis0, axis1, axis2, axis3 and axis4. 
axis_4 = np.array([0., 0., 0., 0., 1., 0.])
jdof5 = jt.Joint(axis_0, axis_1, axis_2, axis_3, axis_4)
assert(jdof5.JointType==jt.JointType.JointType5DoF)
assert(jdof5.DoFCount==5)
try:
	print jdof5.q_index
except eigenpy.exception,e:
	print 'As expected, catch the following exception: ',e.message,' (Dont worry)'
print jdof5.JointAxes(0)
print jdof5.JointAxes(1)
print jdof5.JointAxes(2)
print jdof5.JointAxes(3)
print jdof5.JointAxes(4)

# exposing the constructor which creates 6dof joint from spatial vector of the axis0, axis1, axis2, axis3, axis4 and axis5. 
axis_5 = np.array([0., 0., 0., 0., 0., 1.])
jdof6 = jt.Joint(axis_0, axis_1, axis_2, axis_3, axis_4, axis_5)
assert(jdof6.JointType==jt.JointType.JointType6DoF)
assert(jdof6.DoFCount==6)
try:
	print jdof6.q_index
except eigenpy.exception,e:
	print 'As expected, catch the following exception: ',e.message,' (Dont worry)'
print jdof6.JointAxes(0)
print jdof6.JointAxes(1)
print jdof6.JointAxes(2)
print jdof6.JointAxes(3)
print jdof6.JointAxes(4)
print jdof6.JointAxes(5)

# exposing the function to check whether we have pure rotational or translational axis
axis = np.array([1., 0., 0., 0., 0., 0.])
assert(j.validate_spatial_axis(axis))	

