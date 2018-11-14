// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
// Heavily edited and extended for halfedge navigation
// by Christian Schüller <schuellchr@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_HALFEDGEITERATOR_H
#define IGL_HALFEDGEITERATOR_H

#include <Eigen/Core>

#include <vector>
#include <igl/igl_inline.h>

// This file violates many of the libigl style guidelines.

namespace igl
{
  // HalfEdgeIterator - Fake halfedge for fast and easy navigation
  // on triangle meshes with vertex_triangle_adjacency and
  // triangle_triangle adjacency
  //
  // Note: this is different to classical Half Edge data structure.
  //    Instead, it follows cell-tuple in [Brisson, 1989]
  //    "Representing geometric structures in d dimensions: topology and order."
  //    This class can achieve local navigation similar to half edge in OpenMesh
  //    But the logic behind each atom operation is different.
  //    So this should be more properly called TriangleTupleIterator.
  //
  // Each tuple contains information on (face, edge, vertex)
  //    and encoded by (face, edge \in {0,1,2}, bool reverse)
  //
  // Example initialization:
  //   MatX3I FF,FFi;
  //   igl::triangle_triangle_adjacency(F,FF,FFi);
  //   igl::HalfEdgeIterator<MatX3I> heIter(F,FF,FFi,0,0);
  //
  // Inputs:
  //    F #F by 3 list of "faces"
  //    FF #F by 3 list of triangle-triangle adjacency.
  //    FFi #F by 3 list of FF inverse. For FF and FFi, refer to
  //        "triangle_triangle_adjacency.h"
  // Usages:
  //    FlipF/E/V changes solely one actual face/edge/vertex resp.
  //    NextFE iterates through one-ring of a vertex robustly.
  //
  template <
    typename DerivedF,
    typename DerivedFF,
    typename DerivedFFi>
  class HalfEdgeIterator
  {
  public:
	struct State
    {
      int ei;
      int fi;
      bool reverse;
      bool boundary;
    };
  
	// Init the HalfEdgeIterator by specifying Face, Edge Index and Orientation
    IGL_INLINE HalfEdgeIterator(
      const Eigen::PlainObjectBase<DerivedF>& _F,
      const Eigen::PlainObjectBase<DerivedF>& _FF,
      const Eigen::PlainObjectBase<DerivedF>& _FFi,
      int _fi,
      int _ei,
      bool _reverse = false
    );
	
	// Init the HalfEdgeIterator by another
    IGL_INLINE HalfEdgeIterator(
      const Eigen::PlainObjectBase<DerivedF>& _F,
      const Eigen::PlainObjectBase<DerivedF>& _FF,
      const Eigen::PlainObjectBase<DerivedF>& _FFi,
      const HalfEdgeIterator& other)
    );
	
	// Set current face and edge index
    IGL_INLINE bool init(
	  int faceIndex,
	  int edgeIndex,
	  bool reverse = false
	);
	
	IGL_INLINE State getState() const;
	IGL_INLINE void setState(const State& state);

    // Change Face
    IGL_INLINE bool flipF();

    // Change Edge
    IGL_INLINE void flipE();
	
	// Change to other Halfedge
    // Like flipF() but also works for boundary edges
    IGL_INLINE void flipHE();

    // Change Vertex
    IGL_INLINE void flipV();
	
	// Return if vertex is on boundary
    IGL_INLINE bool isBoundaryV() const;
	
	// Return if edge is on boundary
    IGL_INLINE bool isBoundaryE() const;
	
	// Return if halfedge is on boundary
    IGL_INLINE bool isBoundaryHE() const;
	
	// Todo
    //IGL_INLINE int isBoundaryF() const;
	
	// Deprecated: only for backward compatibility functions above
	IGL_INLINE bool isBorder();
	
	// Todo
    //IGL_INLINE std::vector<int> neighbourV() const;

    // Todo
    //IGL_INLINE std::vector<int> bool neighbourF() const;

	// Move to next halfedge such that Vi0 becomes Vi1
    // Can also be used to travel along boundary halfedges
    IGL_INLINE void nextHE();
	
	// Returns the next halfedge around the current vertex v, including boundaries
    IGL_INLINE void iterHE()

    /*!
     * Returns the next edge skipping the border
     *      _________
     *     /\ c | b /\
     *    /  \  |  /  \
     *   / d  \ | / a  \
     *  /______\|/______\
     *          v
     * In this example, if a and d are of-border and the pos is iterating
     counterclockwise, this method iterate through the faces incident on vertex
     v,
     * producing the sequence a, b, c, d, a, b, c, ...
     */
    IGL_INLINE bool NextFE();

	// Get inner triangle vertex index
    IGL_INLINE int Vii() const;
	
    // Get vertex index
    IGL_INLINE int Vi() const;
	
	// Get inner triangle flipped vertex index
    IGL_INLINE int Viif() const;
	
	// Get flipped vertex index
    IGL_INLINE int Vif() const;
	
	// Get inner triangle vertex index at halfedge start
    IGL_INLINE int Vii0() const;
	
	// Get vertex index at halfedge start
    IGL_INLINE int Vi0() const;
	
	// Get inner triangle vertex index at halfedge end
    IGL_INLINE int Vii1() const;
	
	// Get vertex index at halfedge end
    IGL_INLINE int Vi1() const;

    // Get edge index
    IGL_INLINE int Ei() const;
	
	// Get flipped edge index
    IGL_INLINE int Eif() const;
	
	// Get flipped halfedge index
    IGL_INLINE int HEi() const;
	
	// Get face index
    IGL_INLINE int Fi() const;
	
	// Get flipped face index
    IGL_INLINE int Fif() const;
	
	IGL_INLINE HalfEdgeIterator& operator=(const HalfEdgeIterator& p2);

	IGL_INLINE bool operator==(HalfEdgeIterator& p2) const;
	
	IGL_INLINE bool operator!=(HalfEdgeIterator& p2) const;
	
  private:
	State state;

    // All the same type? This is likely to break.
    const Eigen::PlainObjectBase<DerivedF> & F;
    const Eigen::PlainObjectBase<DerivedFF> & FF;
    const Eigen::PlainObjectBase<DerivedFFi> & FFi;
  };

}

#ifndef IGL_STATIC_LIBRARY
#  include "HalfEdgeIterator.cpp"
#endif

#endif
