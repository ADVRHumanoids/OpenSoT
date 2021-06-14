#ifndef __SOTH_ASET__
#define __SOTH_ASET__

#include <vector>
#include "Algebra.hpp"
#include "Bound.hpp"
#include "api.hpp"

namespace soth {
/* ActiveSet is a invertible map that gives the row where an active
 * constraint is stored, ie MAP such as J(MAP,:) == WMLY. The map is
 * invertible, which means that it is possible to acces to the constraint
 * reference of a given row of WMLY. It also stores the type of the active
 * constraint (+/-/=), and the free/occupied lines of WMLY.  Take care: WMLY
 * is supposed to be an indirect matrix build on W_MLY, in which case the
 * active set is not aware of the indirection (ie: to access to the
 * constraint of WMLY(row), mapInv should be call with W.indirectRow(row),
 * and not directly with row). Similarly, the row where an active constraint
 * is stored in WMLY will need to inverse the W.indirectRow map, which is not
 * done in the class.
 */
template <typename Indirect>
class SOTH_EXPORT ActiveSet {
  typedef typename Indirect::Index Index;

 public: /* --- Construction --- */
  ActiveSet(Index nr)
      : cstMap(nr),
        cstMapInv(nr),
        freerow(nr),
        freezed(nr),
        activated(nr),
        dampingInf(nr),
        dampingSup(nr),
        nba(0) {
    reset();
  }

  void reset(void) {
    std::fill(cstMap.begin(), cstMap.end(), CONSTRAINT_VOID);
    std::fill(cstMapInv.begin(), cstMapInv.end(), size());
    std::fill(freerow.begin(), freerow.end(), true);
    std::fill(freezed.begin(), freezed.end(), false);
    activated.fill(Bound::BOUND_NONE);
    dampingInf.setZero();
    dampingSup.setZero();

    nba = 0;
  }

 public: /* --- Active set management --- */
  /* Active the constraint <ref> (ie in J(ref,:)) at line <row> (ie in
   * ML(row,:)) with bound type. */
  void active(Index ref, Bound::bound_t type, const Index row) {
    assert(ref < size());
    assert((cstMap[ref].type == Bound::BOUND_NONE) &&
           "Constraint has not been properly unactivated.");
    assert(row < size());
    assert(freerow[row]);
    assert(cstMapInv[row] == size());
    assert(type != Bound::BOUND_NONE);

    cstMap[ref] = ConstraintRef(row, type);
    cstMapInv[row] = ref;
    freerow[row] = false;
    nba++;

    if (activated[ref] == Bound::BOUND_NONE)
      activated[ref] = type;
    else {
      // assert( activated[ref]!=type );
      assert(type == Bound::BOUND_INF || type == Bound::BOUND_SUP);
      activated[ref] = Bound::BOUND_DOUBLE;
    }
  }

  /* Active the given constraint at any free row of ML, and return the
   * position of the selected free row. */
  Index activeRow(Index ref, Bound::bound_t type) {
    const Index row = getAFreeRow();
    assert(row < size());
    active(ref, type, row);
    return row;
  }

  inline Index activeRow(const ConstraintRef& cst) {
    return activeRow(cst.row, cst.type);
  }
  /* Unactive the constraint at line <row> of ML and frees the corresponding
   * line. */
  void unactiveRow(Index row) {
    assert(row < size());

    const Index& cst = cstMapInv[row];
    assert(cst < size());
    assert(cstMap[cst].type != Bound::BOUND_NONE);

    cstMap[cst] = CONSTRAINT_VOID;
    freeARow(row);
    cstMapInv[row] = size();
    nba--;
  }

  /* Pass the constraint to a twin mode. */
  void freeze(Index ref) {
    assert(ref < size());
    assert(cstMap[ref].type != Bound::BOUND_NONE);
    assert(cstMap[ref].type != Bound::BOUND_TWIN);
    assert(!freezed[ref]);

    freezed[ref] = true;
  }

 public: /* --- Accessors --- */
  /* Return the number of active constraint. */
  inline Index nbActive(void) const { return nba; }
  inline Index size(void) const { return (Index)cstMap.size(); }
  bool isFreezed(Index ref) const {
    assert(isActive(ref));
    return (cstMap[ref].type == Bound::BOUND_TWIN) || (freezed[ref]);
  }
  bool isActive(Index ref) const {
    assert(ref < size());
    return (cstMap[ref].type != Bound::BOUND_NONE);
  }

  bool wasActive(Index ref, const Bound::bound_t type) const {
    assert(ref < size());
    assert(activated.size() == (int)size());
    assert(type == Bound::BOUND_INF || type == Bound::BOUND_SUP);
    assert(activated[ref] == Bound::BOUND_INF ||
           activated[ref] == Bound::BOUND_SUP ||
           activated[ref] == Bound::BOUND_NONE ||
           activated[ref] == Bound::BOUND_DOUBLE);
    if (type == Bound::BOUND_INF)
      return activated[ref] == Bound::BOUND_INF ||
             activated[ref] == Bound::BOUND_DOUBLE;
    else if (type == Bound::BOUND_SUP)
      return activated[ref] == Bound::BOUND_SUP ||
             activated[ref] == Bound::BOUND_DOUBLE;
    else
      return (false);
  }

  Bound::bound_t whichBound(Index ref, bool checkActive = false) const {
    assert(isActive(ref));
    const Bound::bound_t& res = cstMap[ref].type;
    if (checkActive) {
      assert((res != Bound::BOUND_NONE) && (res != Bound::BOUND_DOUBLE));
    }
    return res;
  }

  double sign(Index ref) const {
    assert(isActive(ref));
    assert(cstMap[ref].type != Bound::BOUND_DOUBLE);
    return (cstMap[ref].type == Bound::BOUND_INF) ? -1 : +1;
  }

  /* Map: from the cst ref, give the row of J.
   * Map inversion: give the reference of the constraint (ie line in J) located
   * at row <row> of the working space ML. */
  Index map(Index ref) const {
    assert(isActive(ref));
    return cstMap[ref].row;
  }

  Index mapInv(Index row) const {
    assert(row < size());
    assert((cstMapInv[row] < size()) && "The requested row is not active");
    return cstMapInv[row];
  }

  Matrix<Index, Dynamic, 1> getIndirection(void) const {
    if (nba == 0) return Matrix<Index, Dynamic, 1>();

    Matrix<Index, Dynamic, 1> res(nba);
    int row = 0;
    for (Index i = 0; i < size(); ++i)
      if (!freerow[i]) res(row++) = whichConstraint(i);
    return res;
  }

  /* For compatibility */
  inline Index where(Index ref) const { return map(ref); }
  inline Index whichConstraint(Index row) const { return mapInv(row); }

  /* Damping */
  void dampBoundValue(const ConstraintRef& cst, const double& value) {
    assert(cst.type == Bound::BOUND_INF || cst.type == Bound::BOUND_SUP);
    const Index& i = cst.row;
    if (cst.type == Bound::BOUND_INF && +value > dampingInf[i])
      dampingInf[i] = +value;
    if (cst.type == Bound::BOUND_SUP && -value > dampingSup[i])
      dampingSup[i] = -value;
  }

  std::pair<double, double> getBoundDamping(const Index& cst) {
    return std::make_pair(dampingInf[cst], dampingSup[cst]);
  }

 public: /* --- Display --- */
  /* Return a compact of the active line, ordered by row values. */
  void disp(std::ostream& os, bool classic = true) const {
    if (classic) {
      os << " [ ";
      for (unsigned int r = 0; r < size(); ++r) {
        if (freerow[r]) continue;
        const Index cst = mapInv(r);
        if (whichBound(cst) == Bound::BOUND_INF)
          os << "-";
        else if (whichBound(cst) == Bound::BOUND_SUP)
          os << "+";
        os << r << " ";
      }
      os << " ]";
    } else {
      for (unsigned int i = 0; i < size(); ++i) {
        os << i << ": ";
        if (isActive(i))
          os << cstMap[i].type;
        else
          os << "Unactive";
        os << std::endl;
      }
      for (unsigned int i = 0; i < freerow.size(); ++i) {
        os << (freerow[i] ? "0" : "1");
      }
      os << std::endl;
    }
  }

 public: /* --- Deprecated --- */
  /* DEPRECATED*/ void permuteRows(const VectorXi& P);

 protected
     : /* TODO: change that for using the ConstraintRef def in Bound.hpp. */
  typedef std::vector<Index> mapinv_vector_t;

 protected:
  cstref_vector_t cstMap;
  mapinv_vector_t cstMapInv;
  std::vector<bool> freerow, freezed;
  Matrix<Bound::bound_t, Dynamic, 1> activated;
  VectorXd dampingInf, dampingSup;
  Index nba;

 protected: /* Internal management */
  bool isRowFree(Index row) { return freerow[row]; }

  Index getAFreeRow(void) {
    /* TODO: it is possible to store the first freeline. */
    assert(nba < size());
    for (Index row = 0; row < size(); ++row) {
      if (freerow[row]) return row;
    }
    assert(false && "Could never happen.");
    return -1;
  }

  void freeARow(Index row) {
    assert(!freerow[row]);
    freerow[row] = true;
  }

 public:
  inline operator Matrix<Index, Dynamic, 1>(void) const {
    return getIndirection();
  }
  //    SOTH_EXPORT friend std::ostream& operator<< ( std::ostream & os,const
  //    ActiveSet<typename Indirect> & as );
};

template <typename Indirect>
SOTH_EXPORT std::ostream& operator<<(std::ostream& os,
                                     const ActiveSet<Indirect>& as);

/* The previous class is not aware of the indirection built upon WMLY. This
 * indirection is added in this derivation, to make it transparent to the user.
 */
template <typename AS, typename Indirect>
class SubActiveSet : protected AS {
  typedef typename Indirect::Index Index;

 public:
  SubActiveSet(Index nr);
  SubActiveSet(Index nr, Indirect& idx);
  SubActiveSet(const SubActiveSet& clone);

  inline bool ownIndirection(void) const { return &self_indirect == &indirect; }

 public:
  void reset(void);
  Index activeRow(Index ref, Bound::bound_t type);
  inline Index activeRow(const ConstraintRef& cst) {
    return activeRow(cst.row, cst.type);
  }
  void unactiveRow(Index row);
  Index mapInv(Index row) const;

  Index map(Index ref) const;
  Bound::bound_t whichBoundInv(Index row) const;
  /* For compatibility */
  inline Index whichConstraint(Index row) const { return mapInv(row); }
  inline Index where(Index cst) const { return map(cst); }
  Matrix<Index, Dynamic, 1> getIndirection(void) const;
  void disp(std::ostream& os, bool classic = true) const;
  inline operator Matrix<Index, Dynamic, 1>(void) const {
    return getIndirection();
  }

  void defrag(void);
  void setInitialActivation(const AS& as0);

 public:
  using AS::dampBoundValue;
  using AS::freeze;
  using AS::getBoundDamping;
  using AS::isActive;
  using AS::isFreezed;
  using AS::nbActive;
  using AS::sign;
  using AS::size;
  using AS::wasActive;
  using AS::whichBound;

 protected
     : /* Forbidden to avoid ambiguities on which 'row' the arg refers to. */
  void active(Index ref, Bound::bound_t type, Index row);
  Index pushIndirectBack(Index rowup);

 protected:
  Indirect self_indirect;
  Indirect& indirect;
  bool isEmpty;
  using AS::nba;
};

template <typename AS, typename Indirect>
std::ostream& operator<<(std::ostream& os,
                         const SubActiveSet<AS, Indirect>& as);

/* --- HEAVY CODE --------------------------------------------------------- */
/* --- HEAVY CODE --------------------------------------------------------- */
/* --- HEAVY CODE --------------------------------------------------------- */
template <typename AS, typename Indirect>
SubActiveSet<AS, Indirect>::SubActiveSet(Index nr)
    : AS(nr), self_indirect(1), indirect(self_indirect), isEmpty(true) {}

template <typename AS, typename Indirect>
SubActiveSet<AS, Indirect>::SubActiveSet(Index nr, Indirect& idx)
    : AS(nr), self_indirect(1), indirect(idx), isEmpty(true) {}

template <typename AS, typename Indirect>
SubActiveSet<AS, Indirect>::SubActiveSet(const SubActiveSet& clone)
    : AS((const AS&)clone),
      self_indirect(clone.self_indirect),
      indirect(clone.ownIndirection() ? self_indirect : clone.indirect) {}

template <typename AS, typename Indirect>
void SubActiveSet<AS, Indirect>::reset(void) {
  AS::reset();
  isEmpty = true;
  indirect.resize(0);
}

template <typename AS, typename Indirect>
typename Indirect::Index SubActiveSet<AS, Indirect>::activeRow(
    Index ref, Bound::bound_t type) {
  assert((isEmpty && (nba == 0)) || (int(nba) == indirect.size()));
  Index rowup = AS::activeRow(ref, type);
  return pushIndirectBack(rowup);
}
template <typename AS, typename Indirect>
typename Indirect::Index SubActiveSet<AS, Indirect>::pushIndirectBack(
    typename Indirect::Index rowup) {
  assert(rowup < size());
  if (isEmpty) {
    indirect.resize(1);
    indirect(0) = rowup;
    isEmpty = false;
  } else {
    indirect.conservativeResize(nba);
    indirect(nba - 1) = rowup;
  }
  assert(nba > 0);
  return nba - 1;
}
template <typename AS, typename Indirect>
void SubActiveSet<AS, Indirect>::unactiveRow(typename Indirect::Index rowrm) {
  assert(Index(nba) == indirect.size());
  assert(rowrm < nba);  // nba>0

  const typename Indirect::Index internalRowrm = indirect(rowrm);
  if (nba == 1) {
    isEmpty = true;
    indirect.resize(0);
  } else {
    const typename Indirect::Index s = nba - rowrm - 1;
    indirect.segment(rowrm, s) = indirect.tail(s);
    indirect.conservativeResize(nba - 1);
  }
  AS::unactiveRow(internalRowrm);
}
template <typename AS, typename Indirect>
typename Indirect::Index SubActiveSet<AS, Indirect>::mapInv(
    typename Indirect::Index row) const {
  assert(row < nba);
  return AS::mapInv(indirect(row));
}

template <typename AS, typename Indirect>
typename Indirect::Index SubActiveSet<AS, Indirect>::map(
    typename Indirect::Index cst) const {
  const typename Indirect::Index row_ = AS::map(cst);
  for (typename Indirect::Index row = 0; row < nba; ++row) {
    if (indirect(row) == row_) return row;
  }
  assert(false && "This could not happen.");
  return -1;
}
template <typename AS, typename Indirect>
Bound::bound_t SubActiveSet<AS, Indirect>::whichBoundInv(
    typename Indirect::Index row) const {
  return whichBound(mapInv(row));
}

template <typename AS, typename Indirect>
void SubActiveSet<AS, Indirect>::setInitialActivation(const AS& as0) {
  assert(&as0 != this);
  reset();
  typename Indirect::Index row = 0;
  for (typename Indirect::Index i = 0; i < as0.size(); ++i) {
    if (as0.isActive(i)) {
      AS::active(i, as0.whichBound(i), row++);
    }
  }
  if (nba > 0) {
    isEmpty = false;
    if (nba == 1) {
      indirect.resize(1);
      indirect[0] = 0;
    } else
      indirect = VectorXi::LinSpaced(nba, 0, nba - 1);
  }
  assert(row == nba);
}

template <typename AS, typename Indirect>
void SubActiveSet<AS, Indirect>::disp(std::ostream& os, bool classic) const {
  if (classic) {
    os << " [ ";
    for (typename Indirect::Index row = 0; row < nba; ++row) {
      const typename Indirect::Index cst = mapInv(row);
      if (whichBound(cst) == Bound::BOUND_INF)
        os << "-";
      else if (whichBound(cst) == Bound::BOUND_SUP)
        os << "+";
      os << cst << " ";
    }
    os << " ];";
  } else {
    AS::disp(os, classic);
    os << "Indirect = " << indirect << std::endl;
  }
}
template <typename AS, typename Indirect>
std::ostream& operator<<(std::ostream& os,
                         const SubActiveSet<AS, Indirect>& as) {
  as.disp(os);
  return os;
}

/*: Return a compact of the active line, ordered by row values. */
template <typename AS, typename Indirect>
Matrix<typename Indirect::Index, Dynamic, 1>
SubActiveSet<AS, Indirect>::getIndirection(void) const {
  if (nba == 0) return Matrix<typename Indirect::Index, Dynamic, 1>();

  Matrix<typename Indirect::Index, Dynamic, 1> res(nba);
  for (unsigned int r = 0; r < nba; ++r) {
    res[r] = mapInv(r);
  }
  return res;
}

/* --- Protected --- */

template <typename AS, typename Indirect>
void SubActiveSet<AS, Indirect>::active(typename Indirect::Index ref,
                                        Bound::bound_t type,
                                        typename Indirect::Index row) {
  AS::active(ref, type, row);
  pushIndirectBack(row);
}

template <typename AS, typename Indirect>
void SubActiveSet<AS, Indirect>::defrag(void) {
  Matrix<typename Indirect::Index, Dynamic, 1> csts = *this;
  cstref_vector_t cstrefs(csts.size());
  for (int i = 0; i < csts.size(); ++i) {
    cstrefs[i] = ConstraintRef(csts[i], whichBound(csts[i]));
  }
  reset();
  for (int i = 0; i < csts.size(); ++i) {
    activeRow(cstrefs[i]);
  }
}

}  // namespace soth

#endif
