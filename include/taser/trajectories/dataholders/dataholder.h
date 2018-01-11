#ifndef TASERV2_DATAHOLDER_H
#define TASERV2_DATAHOLDER_H

#include <memory>

namespace taser {
namespace trajectories {
namespace dataholders {

  // Immutable data holder base class
  template<typename T>
  class DataHolderBase {
   public:
    // Get the i:th parameter address
    virtual T* Parameter(size_t i) const = 0;

    // Return data holder object spanning these parameters
    virtual std::shared_ptr<DataHolderBase<T>> Slice(size_t start, size_t size) const = 0;
  };

  // mutable data holder base class
  template<typename T>
  class MutableDataHolderBase : public DataHolderBase<T> {
   public:
    // Add parameter and return its index
    virtual size_t AddParameter(size_t ndims) = 0;

    // Return number of parameters
    virtual size_t Size() const = 0;
  };

} // namespace dataholders
} // namespace trajectories
} // namespace taser
#endif //TASERV2_DATAHOLDER_H
