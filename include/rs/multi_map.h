#pragma once

#include <iterator>
#include <limits>
#include <vector>

#include "builder.h"
#include "radix_spline.h"

namespace rs {

// A drop-in replacement for std::multimap. Internally creates a sorted copy of
// the data.
template <class RsKeyType, class RsValueType>
class MultiMap {
 public:
  // Member type definitions.
  using key_type = RsKeyType;
  using mapped_type = RsValueType;
  using value_type = std::pair<RsKeyType, RsValueType>;
  using size_type = std::size_t;
  using iterator = typename std::vector<value_type>::iterator;
  using const_iterator = typename std::vector<value_type>::const_iterator;

  // Constructor, creates a copy of the data.
  template <class BidirIt>
  MultiMap(BidirIt first, BidirIt last, size_t num_radix_bits = 18,
           size_t max_error = 32);

  // Lookup functions, like in std::multimap.
  const_iterator find(RsKeyType key) const;
  const_iterator lower_bound(RsKeyType key) const;

  // Iterators.
  const_iterator begin() const { return data_.begin(); }
  const_iterator end() const { return data_.end(); }

  // Size.
  std::size_t size() const { return data_.size(); }

 private:
  std::vector<value_type> data_;
  RadixSpline<RsKeyType> rs_;
};

template <class RsKeyType, class RsValueType>
template <class BidirIt>
MultiMap<RsKeyType, RsValueType>::MultiMap(BidirIt first, BidirIt last,
                                       size_t num_radix_bits,
                                       size_t max_error) {
  // Empty spline.
  if (first == last) {
    rs::Builder<RsKeyType> rsb(std::numeric_limits<RsKeyType>::min(),
                             std::numeric_limits<RsKeyType>::max(),
                             num_radix_bits, max_error);
    rs_ = rsb.Finalize();
    return;
  }

  // Copy data and check if sorted.
  bool is_sorted = true;
  RsKeyType previous_key = first->first;
  for (auto current = first; current != last; ++current) {
    is_sorted &= current->first >= previous_key;
    previous_key = current->first;
    data_.push_back(*current);
  }

  // Sort if necessary.
  if (!is_sorted) {
    std::sort(data_.begin(), data_.end(),
              [](const value_type& lhs, const value_type& rhs) {
                return lhs.first < rhs.first;
              });
  }

  // Create spline builder.
  const auto min_key = data_.front().first;
  const auto max_key = data_.back().first;
  rs::Builder<RsKeyType> rsb(min_key, max_key, num_radix_bits, max_error);

  // Build the radix spline.
  for (const auto& iter : data_) {
    rsb.AddKey(iter.first);
  }
  rs_ = rsb.Finalize();
}

template <class RsKeyType, class RsValueType>
typename MultiMap<RsKeyType, RsValueType>::const_iterator
MultiMap<RsKeyType, RsValueType>::lower_bound(RsKeyType key) const {
  SearchBound bound = rs_.GetSearchBound(key);
  return std::lower_bound(data_.begin() + bound.begin,
                          data_.begin() + bound.end, key,
                          [](const value_type& lhs, const RsKeyType& rhs) {
                            return lhs.first < rhs;
                          });
}

template <class RsKeyType, class RsValueType>
typename MultiMap<RsKeyType, RsValueType>::const_iterator
MultiMap<RsKeyType, RsValueType>::find(RsKeyType key) const {
  auto iter = lower_bound(key);
  return iter != data_.end() && iter->first == key ? iter : data_.end();
}

}  // namespace rs