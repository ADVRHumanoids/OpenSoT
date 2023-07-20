#include <assert.h>
#include "../include/soth/Allocator.hpp"

namespace soth {
/* Replace all the tokens in the resource. */
void AllocatorML::reset() { resetTop(0); }

/* Replace the tokens [min,max[ in the ressource. */
void AllocatorML::resetTop(Index min) {
  assert(min <= max);
  resource.resize(max - min);
  Index inc = min;
  for (resource_iter_t iter = resource.begin(); iter != resource.end();
       ++iter) {
    (*iter) = inc++;
  }
  assert(resource.size() == 0 || resource.back() == max - 1);
}

typename AllocatorML::Index AllocatorML::get() {
  assert(resource.size() > 0);
  const Index token = resource.front();
  resource.pop_front();
  return token;
}

void AllocatorML::put(const Index& token) {
  resource.push_front(token);
  assert((Index)resource.size() <= max);
}

void AllocatorML::disp(std::ostream& os) const {
  typedef resource_t::const_iterator resource_citer_t;
  os << "[ ";
  for (resource_citer_t iter = resource.begin(); iter != resource.end();
       ++iter) {
    os << *iter << " ";
  }
  os << "  ];";
}

std::ostream& operator<<(std::ostream& os, const AllocatorML& aml) {
  aml.disp(os);
  return os;
}

}  // namespace soth
