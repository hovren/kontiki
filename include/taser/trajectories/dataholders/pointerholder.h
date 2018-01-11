#ifndef TASERV2_POINTERHOLDER_H
#define TASERV2_POINTERHOLDER_H

#include "dataholder.h"

namespace taser {
namespace trajectories {
namespace dataholders {

// Data holder that holds a reference to raw pointers as returned by ceres-solver
template<typename T>
class PointerHolder : public DataHolderBase<T> {
 public:
  PointerHolder(T const* const* data) : data_(data) {};

  T* Parameter(size_t i) const override {
    return (T*) data_[i];
  }

  std::shared_ptr<DataHolderBase<T>> Slice(size_t start, size_t size) const {
    T const* const* ptr = &data_[start];
    auto slice = std::make_shared<PointerHolder<T>>(ptr);
    return slice;
  }

 protected:
  T const* const* data_;
};

} // namespace dataholders
} // namespace trajectories
} // namespace taser
#endif //TASERV2_POINTERHOLDER_H
