// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "HalfEdgeIterator.h"

// debug macro
#if defined(IGL_HALFEDGE_ITERATOR_DEBUG_0)
  #define HE_ITER_DEBUG(action) std::cout << action << " (" << Fi() << "/" << Vi0() << "/" << Vi1() << "/" << state.boundary << ")" << std::endl;
#elif defined(IGL_HALFEDGE_ITERATOR_DEBUG_1)
  #define HE_ITER_DEBUG(action) std::cout << action << " (" << state.fi << "/" << state.ei << "/" << state.reverse << "/" << state.boundary << ")" << std::endl;
#else
  #define HE_ITER_DEBUG(action)
#endif

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::HalfEdgeIterator(
  const Eigen::PlainObjectBase<DerivedF>& _F,
  const Eigen::PlainObjectBase<DerivedFF>& _FF,
  const Eigen::PlainObjectBase<DerivedFFi>& _FFi,
  int _fi,
  int _ei,
  bool _reverse
)
: F(_F), FF(_FF), FFi(_FFi)
{
  state.fi = _fi;
  state.ei = _ei;
  state.reverse = _reverse;
  state.boundary = false;
  HE_ITER_DEBUG("Constructor");
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::HalfEdgeIterator(
  const Eigen::PlainObjectBase<DerivedF>& _F,
  const Eigen::PlainObjectBase<DerivedF>& _FF,
  const Eigen::PlainObjectBase<DerivedF>& _FFi,
  const HalfEdgeIterator& other)
  : F(_F),FF(_FF),FFi(_FFi)
{
  state.fi = other.fi;
  state.ei = other.ei;
  state.reverse = other.reverse;
  state.boundary = false;
  HE_ITER_DEBUG("Constructor");
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE bool igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::init(
  int faceIndex,
  int edgeIndex,
  bool reverse)
{
  if(faceIndex >= 0 && faceIndex < F.rows() && edgeIndex >= 0 && edgeIndex <= 2)
  {
	state.fi = faceIndex;
	state.ei = edgeIndex;
	state.reverse = reverse;
	state.boundary = false;

	HE_ITER_DEBUG("Init");
	return true;
  }

  HE_ITER_DEBUG("Init failed");
  return false;
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE State igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::getState() const
{
  return state;
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE void igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::setState(const State& state)
{
  this->state = state;
  HE_ITER_DEBUG("Set state");
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE bool igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::flipF()
{
  int fin = Fif();

  // check if not boundary face
  if(fin != -1)
  {
	state.ei = FFi(state.fi,state.ei);
	state.fi = fin;
	state.reverse = !state.reverse;
	HE_ITER_DEBUG("Flip face");
	return true;
  }

  HE_ITER_DEBUG("Flip face failed - boundary");
  return false;
}


// Change Edge
template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE void igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::flipE()
{
  state.ei = Eif();
  state.reverse = !state.reverse;
  state.boundary = false;
  HE_ITER_DEBUG("Flip edge");
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE void igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::flipHE()
{
  state.boundary = !flipF() && !state.boundary;
  HE_ITER_DEBUG("Flip halfedge");
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE void igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::flipV()
{
  state.reverse = !state.reverse;
  HE_ITER_DEBUG("Flip Vertex");
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE bool igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::isBoundaryV() const
{
  HalfEdgeIterator<DerivedF> iter(F,FF,FFi,0,0);
  HalfEdgeIterator<DerivedF> end(F,FF,FFi,0,0);
  iter.setState(state);
  
  if(state.reverse != state.boundary)
  {
	iter.flipHE();
  }

  bool isBoundary = false;
  end = iter;
  do
  {
	isBoundary = isBoundary || iter.isBoundaryE();
	iter.iterHE();
  } while(iter != end && !isBoundary);

  return isBoundary;
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE bool igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::isBoundaryE() const
{
  return FF(state.fi,state.ei) == -1;
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE bool igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::isBoundaryHE() const
{
  return state.boundary;
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE bool igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::isBorder()
{
  return (FF)(fi,ei) == -1;
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE void igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::nextHE()
{
  if(state.boundary)
  {
	if(state.reverse)
	{
	  flipV();
	  do
	  {
		flipF();
		flipE();
	  } while(!isBoundaryE());
	  flipHE();
	}
	else
	{
	  do
	  {
		flipF();
		flipE();
	  } while(!isBoundaryE());
	  flipHE();
	  flipV();
	}
  }
  else
  {
	state.ei = (state.ei == 2) ? 0 : (state.ei+1);
  }
  HE_ITER_DEBUG("Next halfedge");
}
template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE void igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::iterHE()
{
  if(state.reverse != state.boundary)
  {
	nextHE();
	flipV();
	flipHE();
  }
  else
  {
	flipHE();
	nextHE();
	flipV();
  }
  HE_ITER_DEBUG("Iterate halfedge");
}

/*!
 * Returns the next edge skipping the border
 *      _________
 *     /\ c | b /\
 *    /  \  |  /  \
 *   / d  \ | / a  \
 *  /______\|/______\
 *          v
 * In this example, if a and d are of-border and the pos is iterating counterclockwise, this method iterate through the faces incident on vertex v,
 * producing the sequence a, b, c, d, a, b, c, ...
 */
template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE bool igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::nextFE()
{
  if ( isBoundaryE() ) // we are on a boundary
  {
	do
	{
	  flipF();
	  flipE();
	} while (!isBoundaryE());
	flipE();
	HE_ITER_DEBUG("Next face edge - border");
	return false;
  }
  else
  {
	flipF();
	flipE();
	HE_ITER_DEBUG("Next face edge");
	return true;
  }
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE int igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::Vii() const
{
  return !state.reverse ? state.ei : (state.ei+1)%3;
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE int igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::Vi()
{
  assert(fi >= 0);
  assert(fi < F.rows());
  assert(ei >= 0);
  assert(ei <= 2);
  return F(state.fi,Vii());
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE int igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::Viif() const
{
  return !state.reverse ? (state.ei+1)%3 : state.ei;
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE int igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::Vif() const
{
  assert(state.fi >= 0);
  assert(state.fi < F.rows());
  assert(state.ei >= 0);
  assert(state.ei <= 2);
  return F(state.fi,Viif());
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE int igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::Vii0() const
{
  return !state.boundary ? state.ei : (state.ei+1)%3;
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE int igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::Vi0() const
{
  assert(state.fi >= 0);
  assert(state.fi < F.rows());
  assert(state.ei >= 0);
  assert(state.ei <= 2);
  return F(state.fi,Vii0());
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE int igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::Vii1() const
{
  return !state.boundary ? (state.ei+1)%3 : state.ei;
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE int igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::Vi1() const
{
  assert(state.fi >= 0);
  assert(state.fi < F.rows());
  assert(state.ei >= 0);
  assert(state.ei <= 2);
  return F(state.fi,Vii1());
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE int igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::Ei() const
{
  return state.ei;
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE int igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::Eif() const
{
  return !state.reverse ? (state.ei+2)%3 : (state.ei+1)%3;;
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE int igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::HEi() const
{
  return FFi(state.fi,state.ei);
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE int igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::Fi() const
{
  return state.fi;
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE int igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::Fif() const
{
  return FF(state.fi,state.ei);
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE HalfEdgeIterator& igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::operator=(const HalfEdgeIterator& p2)
{
  assert((F == p2.F) && (FF  == p2.FF) && (FFi == p2.FFi));

  state.fi = p2.state.fi;
  state.ei = p2.state.ei;
  state.reverse = p2.state.reverse;
  state.boundary = p2.state.boundary;

  HE_ITER_DEBUG("Assigment");

  return *this;
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE bool igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::operator==(HalfEdgeIterator& p2) const
{
  return
  (
   (state.fi == p2.state.fi) &&
   (state.ei == p2.state.ei) &&
   (state.reverse == p2.state.reverse) &&
   (state.boundary == p2.state.boundary) &&
   (&F   == &p2.F) &&
   (&FF  == &p2.FF) &&
   (&FFi == &p2.FFi)
   );
}

template <typename DerivedF, typename DerivedFF, typename DerivedFFi>
IGL_INLINE bool igl::HalfEdgeIterator<DerivedF,DerivedFF,DerivedFFi>::operator!=(HalfEdgeIterator& p2) const
{
  return !(*this == p2);
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template instantiation
template      igl::HalfEdgeIterator<Eigen::Matrix<int, -1, 3, 0, -1, 3>  ,Eigen::Matrix<int, -1, 3, 0, -1, 3>  ,Eigen::Matrix<int, -1, 3, 0, -1, 3>   >::HalfEdgeIterator(Eigen::PlainObjectBase<Eigen::Matrix<int, -1, 3, 0, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, 3, 0, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, 3, 0, -1, 3> > const&, int, int, bool);
template igl::HalfEdgeIterator<Eigen::Matrix<int, -1, 3, 0, -1, 3>, Eigen::Matrix<int, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1> >::HalfEdgeIterator(Eigen::PlainObjectBase<Eigen::Matrix<int, -1, 3, 0, -1, 3> > const&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&, int, int, bool);
template bool igl::HalfEdgeIterator<Eigen::Matrix<int, -1, -1, 0, -1, -1>,Eigen::Matrix<int, -1, -1, 0, -1, -1>,Eigen::Matrix<int, -1, -1, 0, -1, -1> >::NextFE();
template int  igl::HalfEdgeIterator<Eigen::Matrix<int, -1, -1, 0, -1, -1>,Eigen::Matrix<int, -1, -1, 0, -1, -1>,Eigen::Matrix<int, -1, -1, 0, -1, -1> >::Ei();
template int  igl::HalfEdgeIterator<Eigen::Matrix<int, -1, 3, 0, -1, 3>  ,Eigen::Matrix<int, -1, -1, 0, -1, -1>,Eigen::Matrix<int, -1, -1, 0, -1, -1> >::Ei();
template int  igl::HalfEdgeIterator<Eigen::Matrix<int, -1, 3, 0, -1, 3>  ,Eigen::Matrix<int, -1, 3, 0, -1, 3>  ,Eigen::Matrix<int, -1, 3, 0, -1, 3>   >::Ei();
template int  igl::HalfEdgeIterator<Eigen::Matrix<int, -1, 3, 0, -1, 3>  ,Eigen::Matrix<int, -1, 3, 0, -1, 3>  ,Eigen::Matrix<int, -1, 3, 0, -1, 3>   >::Fi();
template bool igl::HalfEdgeIterator<Eigen::Matrix<int, -1, 3, 0, -1, 3>  ,Eigen::Matrix<int, -1, 3, 0, -1, 3>  ,Eigen::Matrix<int, -1, 3, 0, -1, 3>   >::NextFE();
template int  igl::HalfEdgeIterator<Eigen::Matrix<int, -1, -1, 0, -1, -1>,Eigen::Matrix<int, -1, -1, 0, -1, -1>,Eigen::Matrix<int, -1, -1, 0, -1, -1> >::Vi();
template      igl::HalfEdgeIterator<Eigen::Matrix<int, -1, -1, 0, -1, -1>,Eigen::Matrix<int, -1, -1, 0, -1, -1>,Eigen::Matrix<int, -1, -1, 0, -1, -1> >::HalfEdgeIterator(Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&, int, int, bool);
template int  igl::HalfEdgeIterator<Eigen::Matrix<int, -1, -1, 0, -1, -1>,Eigen::Matrix<int, -1, -1, 0, -1, -1>,Eigen::Matrix<int, -1, -1, 0, -1, -1> >::Fi();
template void igl::HalfEdgeIterator<Eigen::Matrix<int, -1, -1, 0, -1, -1>,Eigen::Matrix<int, -1, -1, 0, -1, -1>,Eigen::Matrix<int, -1, -1, 0, -1, -1> >::flipE();
template void igl::HalfEdgeIterator<Eigen::Matrix<int, -1, -1, 0, -1, -1>,Eigen::Matrix<int, -1, -1, 0, -1, -1>,Eigen::Matrix<int, -1, -1, 0, -1, -1> >::flipF();
template void igl::HalfEdgeIterator<Eigen::Matrix<int, -1, -1, 0, -1, -1>,Eigen::Matrix<int, -1, -1, 0, -1, -1>,Eigen::Matrix<int, -1, -1, 0, -1, -1> >::flipV();
template bool igl::HalfEdgeIterator<Eigen::Matrix<int, -1, -1, 0, -1, -1>,Eigen::Matrix<int, -1, -1, 0, -1, -1>,Eigen::Matrix<int, -1, -1, 0, -1, -1> >::operator==(igl::HalfEdgeIterator<Eigen::Matrix<int, -1, -1, 0, -1, -1>,Eigen::Matrix<int, -1, -1, 0, -1, -1>,Eigen::Matrix<int, -1, -1, 0, -1, -1> >&);
template int igl::HalfEdgeIterator<Eigen::Matrix<int, -1, 3, 0, -1, 3>, Eigen::Matrix<int, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1> >::Fi();
template bool igl::HalfEdgeIterator<Eigen::Matrix<int, -1, 3, 0, -1, 3>, Eigen::Matrix<int, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1> >::NextFE();
#endif
