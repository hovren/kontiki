#ifndef TASERV2_VECTORHOLDER_H
#define TASERV2_VECTORHOLDER_H

#include "dataholder.h"

#include <vector>

namespace taser {
namespace trajectories {
namespace dataholders {

namespace detail {

template<typename T>
class VectorSliceHolder : public DataHolderBase<T> {
  using ConstIterator = typename std::vector<T *>::const_iterator;
 public:


  VectorSliceHolder(ConstIterator begin, size_t length) :
      DataHolderBase<T>(),
      begin_(begin),
      length_(length) { }

  T* Parameter(size_t i) const override {
    return *(begin_ + i);
  }

  std::shared_ptr<DataHolderBase<T>> Slice(size_t start, size_t size) const {
    return std::make_shared<VectorSliceHolder<T>>(begin_ + start, size);
  }

 protected:
  ConstIterator begin_;
  size_t length_;
};

} // namespace detail

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
    return std::make_shared<detail::VectorSliceHolder<T>>(this->data_.begin() + start, size);
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
