//
// Copyright (c) 2020 Terry Chen <ty6chen@uwaterloo.ca>
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
// 
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the
// Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
// Boston, MA  02110-1301, USA.
//

#ifndef SGMWSMCPP_INDEXER_H
#define SGMWSMCPP_INDEXER_H

#include <initializer_list>
#include <iostream>
#include <vector>
#include <utility>
#include <boost/container_hash/hash.hpp>

#ifdef NDEBUG
#include <parallel_hashmap/phmap.h>
#else
#include <unordered_map>
#endif

namespace sgm
{

template<typename T,
         typename Hash = boost::hash<T>>
class Indexer
{
public:
    using object_type = T;
    using index_type = size_t;
#ifdef NDEBUG
    using map_type = phmap::flat_hash_map<object_type, index_type, Hash>;
#else
    using map_type = std::unordered_map<object_type, index_type, Hash>;
#endif

private:
    std::vector<object_type> m_i2o;
    map_type m_o2i;

public:
    Indexer() = default;

    template <typename ForwardIterator>
    Indexer(ForwardIterator begin, ForwardIterator end) : m_i2o(begin, end) {
        auto i = 0;
        for (auto it = begin; it != end; ++it) {
            m_o2i[*it] = i;
            ++i;
        }
    }

    Indexer(std::initializer_list<object_type> init) : Indexer(init.begin(), init.end()) {}

    template <typename InitList>
    explicit Indexer(InitList &&init) : Indexer(init.begin(), init.end()) {}

    Indexer(std::initializer_list<typename map_type::value_type> init): m_o2i(init), m_i2o(init.size()) {
        auto i = 0;
        for (auto it = init.begin(); it != init.end(); ++it) {
            m_i2o[i] = *it;
            ++i;
        }
    }

    const T &i2o(index_type i) const {
        return m_i2o.at(i);
    }

    index_type o2i(const T &o) const {
        return m_o2i.at(o);
    }

    template <typename ForwardIterator>
    void add_all(ForwardIterator begin, ForwardIterator end) {
        auto i = m_i2o.size() - 1;
        for (auto it = begin; it != end; ++it) {
            m_i2o.push_back(*it);
            m_o2i[*it] = i;
            ++i;
        }
    }

    size_t count_index(index_type i) const {
        return i >= 0 && i < m_i2o.size();
    }

    size_t count_object(object_type o) const {
        return m_o2i.count(o);
    }

    size_t size() const {
        return m_i2o.size();
    }
};
}

#endif //SGMWSMCPP_INDEXER_H
