/*
 * Copyright 2014, Nicolas Mansard, LAAS-CNRS
 *
 * This file is part of eigenpy.
 * eigenpy is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * eigenpy is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with eigenpy.  If not, see <http://www.gnu.org/licenses/>.
 */
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT 

#include "rbdlpy/rbdlpy.hpp"
#include <boost/python.hpp>

using namespace boost::python;
using namespace std;

RigidBodyDynamics::Math::VectorNd test(RigidBodyDynamics::Math::VectorNd rand_array)
{
	return rand_array;
}

RigidBodyDynamics::Math::VectorNd test2()
{
	RigidBodyDynamics::Math::VectorNd rand_array = RigidBodyDynamics::Math::VectorNd::Zero (3);
	rand_array[0] = 6.220225614566666666;
	rand_array[1] = 4.7853457158; 
	return rand_array;
}

BOOST_PYTHON_MODULE(libsimple)
{
  eigenpy::enableEigenPy();
  eigenpy::enableEigenPySpecific<RigidBodyDynamics::Math::VectorNd, Eigen::VectorXd>();
  def("test",test);
  def("test2",test2);
}
