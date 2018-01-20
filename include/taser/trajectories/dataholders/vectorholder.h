#ifndef TASERV2_VECTORHOLDER_H
#define TASERV2_VECTORHOLDER_H

#include "dataholder.h"

#include <vector>

namespace taser {
namespace trajectories {
namespace dataholders {

// Mutable data holder that wraps a std::vector
template<typename T>
class VectorHolder : public MutableDataHolderBase<T> {
 public:
  ~VectorHolder() {
    for (auto ptr : data_) {
      delete[] ptr;
    }
  }

  T* Parameter(size_t i) const override {
    return data_[i];
  }

  std::shared_ptr<DataHolderBase<T>> Slice(size_t start, size_t size) const {
    auto slice = std::make_shared<VectorHolder<T>>();
    // FIXME: bounds checks
    slice->data_.assign(this->data_.begin() + start, this->data_.begin() + start + size - 1);
    return slice;
  }

  size_t Size() const override {
    return data_.size();
  }

  size_t AddParameter(size_t ndims) override {
    auto ptr = new T[ndims];
    data_.push_back(ptr);
    return data_.size() - 1;
  }

 protected:
  std::vector<T*> data_;
};

} // namespace dataholders
} // namespace trajectories
} // namespace taser

#endif //TASERV2_VECTORHOLDER_H
