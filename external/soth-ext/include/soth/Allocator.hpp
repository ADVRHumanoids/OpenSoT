#ifndef __SOTH_ALLOCATOR__
#define __SOTH_ALLOCATOR__

#include <iostream>
#include <list>
#include "api.hpp"

#include <Eigen/Core>

namespace soth {
class SOTH_EXPORT AllocatorML {
  typedef Eigen::MatrixXd::Index Index;
  typedef std::list<Index> resource_t;
  typedef resource_t::iterator resource_iter_t;

  resource_t resource;
  Index max;

 public:
  AllocatorML(Index max) : resource(), max(max) {}

  void reset();
  void resetTop(Index min);
  Index get();
  void put(const Index& token);
  void disp(std::ostream& os) const;

  SOTH_EXPORT friend std::ostream& operator<<(std::ostream& os,
                                              const AllocatorML& aml);
};

}  // namespace soth

#endif  // #ifndef __SOTH_ALLOCATOR__
