/*
 * Copyright 2014, Shivesh KUMAR, LAAS-CNRS
 *
 * This file is part of rbdlpy.
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
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <rbdl/addons/urdfreader/rbdl_urdfreader.h>

using namespace boost::python;
using namespace std;
using namespace eigenpy;
using namespace RigidBodyDynamics;

BOOST_PYTHON_MODULE(liburdfreader)
{	
	def("read_urdf_model",Addons::read_urdf_model, args("path to .urdf file", "model", "verbose"), "Reads a urdf model from file");
}

