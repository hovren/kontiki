#ifndef TASERV2_MULTIHOLDER_H
#define TASERV2_MULTIHOLDER_H

#include "dataholder.h"

namespace taser {
namespace trajectories {
namespace dataholders {

template <typename T, int N>
class MultiHolder : public MutableDataHolderBase<T> {
 public:

  MultiHolder() {};

  MultiHolder(std::initializer_list<std::shared_ptr<MutableDataHolderBase<T>>> holder_list) {
    Initialize(holder_list);
  }

  void Initialize(std::initializer_list<std::shared_ptr<MutableDataHolderBase<T>>> holder_list) {
    if(holder_list.size() != N) {
      throw std::length_error("Wrong number of arguments");
    }

    int i=0;
    for (auto h : holder_list) {
      holders_[i] = h;
      i += 1;
    }
  }

  T* Parameter(size_t i) const override {
    int j = 0;
    int hi = 0;
    for (auto h : holders_) {
      const size_t n = h->Size();
      if ((j + n) > i) {
        int subi = i - j;
        return h->Parameter(subi);
      }
      else {
        j += n;
      }

      hi += 1;
    }

    throw std::length_error("Parameter index out of range");
  }

  std::shared_ptr<DataHolderBase<T>> Slice(size_t start, size_t size) const override {
    int j = 0;
    for (auto h : holders_) {
      const size_t n = h->Size();
      // Only accept slices on exact boundaries
      if (j == start) {
        return h;
      }
      else if ((j + n) <= start) {
        j += n;
      }
      else {
        break;
      }
    }

    throw std::runtime_error("Can only slice on exact holder boundaries");
  }

  size_t AddParameter(size_t ndims) override {
    throw std::runtime_error("Use the specific MultiHolder interface instead!");
  }

  size_t Size() const override {
    size_t sz = 0;
    for (auto h : holders_) {
      sz += h->Size();
    }

    return sz;
  }

  std::shared_ptr<MutableDataHolderBase<T>> GetHolder(size_t i) {
    return holders_[i];
  }

 protected:
  std::array<std::shared_ptr<MutableDataHolderBase<T>>, N> holders_; // FIXME: Ownage of the holder pointers is unclear
};

} // namespace dataholders
} // namespace trajectories
} // namespace taser
#endif //TASERV2_MULTIHOLDER_H
