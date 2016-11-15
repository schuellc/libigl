// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "read_triangle_mesh.h"

#include <igl/list_to_matrix.h>
#include <igl/readMESH.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/readSTL.h>
#include <igl/readPLY.h>
#include <igl/readWRL.h>
#include <igl/pathinfo.h>
#include <igl/boundary_facets.h>
#include <igl/polygon_mesh_to_triangle_mesh.h>

#include <cstdio>
#include <algorithm>
#include <iostream>


template <typename Scalar, typename Index>
IGL_INLINE bool igl::read_triangle_mesh(
  const std::string str,
  std::vector<std::vector<Scalar> > & V,
  std::vector<std::vector<Index> > & F)
{
  using namespace std;
  // dirname, basename, extension and filename
  string d,b,e,f;
  pathinfo(str,d,b,e,f);
  // Convert extension to lower case
  std::transform(e.begin(), e.end(), e.begin(), ::tolower);
  vector<vector<Scalar> > TC, N, C;
  vector<vector<Index> > FTC, FN;
  if(e == "obj")
  {
    // Annoyingly obj can store 4 coordinates, truncate to xyz for this generic
    // read_triangle_mesh
    bool success = readOBJ(str,V,TC,N,F,FTC,FN);
    for(auto & v : V)
    {
      v.resize(std::min(v.size(),(size_t)3));
    }
    return success;
  }else if(e == "off")
  {
    return readOFF(str,V,F,N,C);
  }
  cerr<<"Error: "<<__FUNCTION__<<": "<<
    str<<" is not a recognized mesh file format."<<endl;
  return false;
}


#ifndef IGL_NO_EIGN
template <typename DerivedV, typename DerivedF>
IGL_INLINE bool igl::read_triangle_mesh(
  const std::string str,
  Eigen::PlainObjectBase<DerivedV>& V,
  Eigen::PlainObjectBase<DerivedF>& F)
{
  std::string _1,_2,_3,_4;
  return read_triangle_mesh(str,V,F,_1,_2,_3,_4);
}

template <typename DerivedV, typename DerivedF>
IGL_INLINE bool igl::read_triangle_mesh(
  const std::string filename,
  Eigen::PlainObjectBase<DerivedV>& V,
  Eigen::PlainObjectBase<DerivedF>& F,
  std::string & dir,
  std::string & base,
  std::string & ext,
  std::string & name)
{
  using namespace std;
  using namespace Eigen;

  // dirname, basename, extension and filename
  pathinfo(filename,dir,base,ext,name);
  // Convert extension to lower case
  transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
  vector<vector<double > > vV,vN,vTC,vC;
  vector<vector<int > > vF,vFTC,vFN;
  if(ext == "mesh")
  {
    // Convert extension to lower case
    MatrixXi T;
    if(!readMESH(filename,V,T,F))
    {
      return 1;
    }
    //if(F.size() > T.size() || F.size() == 0)
    {
      boundary_facets(T,F);
    }
  }else if(ext == "obj")
  {
    if(!readOBJ(filename,vV,vTC,vN,vF,vFTC,vFN))
    {
      return false;
    }
    // Annoyingly obj can store 4 coordinates, truncate to xyz for this generic
    // read_triangle_mesh
    for(auto & v : vV)
    {
      v.resize((std::min)(v.size(),(size_t)3));
    }
  }else if(ext == "off")
  {
    if(!readOFF(filename,vV,vF,vN,vC))
    {
      return false;
    }
  }else if(ext == "ply")
  {
    if(!readPLY(filename,vV,vF,vN,vTC))
    {
      return false;
    }
  }else if(ext == "stl")
  {
    MatrixXd _;
    if(!readSTL(filename,V,F,_))
    {
      return false;
    }
  }else if(ext == "wrl")
  {
    if(!readWRL(filename,vV,vF))
    {
      return false;
    }
  }else
  {
    cerr<<"Error: unknown extension: "<<ext<<endl;
    return false;
  }
  if(vV.size() > 0)
  {
    if(!list_to_matrix(vV,V))
    {
      return false;
    }
    polygon_mesh_to_triangle_mesh(vF,F);
  }
  return true;
}

#endif

#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
// generated by autoexplicit.sh
template bool igl::read_triangle_mesh<Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1> >(std::basic_string<char, std::char_traits<char>, std::allocator<char> >, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> >&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> >&);
template bool igl::read_triangle_mesh<double, int>(std::basic_string<char, std::char_traits<char>, std::allocator<char> >, std::vector<std::vector<double, std::allocator<double> >, std::allocator<std::vector<double, std::allocator<double> > > >&, std::vector<std::vector<int, std::allocator<int> >, std::allocator<std::vector<int, std::allocator<int> > > >&);
template bool igl::read_triangle_mesh<Eigen::Matrix<double, -1, 3, 0, -1, 3>, Eigen::Matrix<int, -1, 3, 0, -1, 3> >(std::string, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, 3, 0, -1, 3> >&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, 3, 0, -1, 3> >&);
template bool igl::read_triangle_mesh<Eigen::Matrix<float, -1, 3, 1, -1, 3>, Eigen::Matrix<unsigned int, -1, 3, 1, -1, 3> >(std::basic_string<char, std::char_traits<char>, std::allocator<char> >, Eigen::PlainObjectBase<Eigen::Matrix<float, -1, 3, 1, -1, 3> >&, Eigen::PlainObjectBase<Eigen::Matrix<unsigned int, -1, 3, 1, -1, 3> >&);
template bool igl::read_triangle_mesh<Eigen::Matrix<double, -1, 3, 1, -1, 3>, Eigen::Matrix<int, -1, 3, 1, -1, 3> >(std::basic_string<char, std::char_traits<char>, std::allocator<char> >, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, 3, 1, -1, 3> >&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, 3, 1, -1, 3> >&);
template bool igl::read_triangle_mesh<Eigen::Matrix<double, -1, 3, 1, -1, 3>, Eigen::Matrix<int, -1, 3, 1, -1, 3> >(std::basic_string<char, std::char_traits<char>, std::allocator<char> >, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, 3, 1, -1, 3> >&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, 3, 1, -1, 3> >&, std::basic_string<char, std::char_traits<char>, std::allocator<char> >&, std::basic_string<char, std::char_traits<char>, std::allocator<char> >&, std::basic_string<char, std::char_traits<char>, std::allocator<char> >&, std::basic_string<char, std::char_traits<char>, std::allocator<char> >&);
template bool igl::read_triangle_mesh<Eigen::Matrix<double, -1, -1, 1, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1> >(std::basic_string<char, std::char_traits<char>, std::allocator<char> >, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 1, -1, -1> >&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> >&);
#endif
