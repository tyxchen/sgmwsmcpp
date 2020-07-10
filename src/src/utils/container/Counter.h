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

#ifndef SGMWSMC_COUNTER_H
#define SGMWSMC_COUNTER_H

#include <algorithm>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <vector>
#include <utility>
#include <boost/container_hash/hash.hpp>

#include <parallel_hashmap/phmap.h>

namespace sgm
{

namespace detail
{
template<typename T, typename V>
bool compare(const std::pair<T, V> *lhs, const std::pair<T, V> *rhs) {
    return lhs->second > rhs->second;
}
}

/**
 * A map from objects to doubles. Includes convenience methods for getting,
 * setting, and incrementing element counts. Objects not in the counter will
 * return a count of zero. The counter is backed by a HashMap.
 *
 * If the provided object type is not hashable, this map stores pointers to the objects.
 * To store the objects themselves, you can define hash<T>() yourself, or pass in a hash
 * function to the template argument Hash.
 *
 * Transformed from briefj.collections.Counter. The original author is Dan Klein.
 *
 * @author Terry Chen <ty6chen@uwaterloo.ca>
 */

template<typename T,
         typename Count = double,
         typename Hash = boost::hash<T>>
class Counter
{
public:
    using key_type = T;
    using mapped_type = Count;
    using value_type = std::pair<const T &, mapped_type>;
    using map_type = phmap::flat_hash_map<key_type, mapped_type, Hash>;

private:
    size_t m_current_mod_count = 1;
    mutable size_t m_sorted_pairs_mod_count = 0;
    mutable size_t m_total_count_mod_count = 0;
    mutable std::vector<const std::pair<const key_type, mapped_type> *> m_cache_sorted_pairs;
    mutable mapped_type m_cache_total_count = 0.0;

    map_type m_entries;

public:
    Counter() = default;

    template <typename ForwardIterator>
    Counter(ForwardIterator begin, ForwardIterator end) : m_entries(begin, end) {}

    Counter(std::initializer_list<typename map_type::value_type> init) : m_entries(init) {}

    template <typename InitList>
    explicit Counter(const InitList &init) {
        m_entries.reserve(init.size());
        for (const auto &thing : init) {
            m_entries[thing] = 0;
        }
    }

    /**
     * Clears the container.
     */
    void clear() {
        m_current_mod_count = 1;
        m_sorted_pairs_mod_count = 0;
        m_entries.clear();
    }

    /**
     * Returns the number of elements in the container.
     */
    [[nodiscard]]
    size_t size() const {
        return m_entries.size();
    }

    /**
     * Returns true if the container is empty, false otherwise.
     */
    [[nodiscard]]
    bool empty() const {
        return m_entries.empty();
    }

    /**
     * Returns the number of instances of the given key stored in the container. If this container is backed by a map,
     * count will always return either 0 or 1.
     * @param key
     * @return
     */
    [[nodiscard]]
    size_t count(const key_type &key) const {
        return m_entries.count(key);
    }

    mapped_type erase(const key_type &key) {
        auto key_count = get(key);
        m_entries.erase(key);
        ++m_current_mod_count;
        return key_count;
    }

    [[nodiscard]]
    mapped_type get(const key_type &key, mapped_type default_count = 0) const {
        if (count(key) == 0) return default_count;
        // We have to call unordered_map::at here as operator[] is non-const
        return m_entries.at(key);
    }

    void set(const key_type &key, mapped_type count) {
        m_entries[key] = count;
        ++m_current_mod_count;
    }

    void increment(const key_type &key, mapped_type count = 1) {
        set(key, get(key) + count);
    }

    template<typename ForwardIterator>
    void increment_all(ForwardIterator begin, ForwardIterator end, mapped_type count = 1) {
        for (const auto &it = begin; it != end; ++it) {
            increment(*it, count);
        }
    }

    template<typename S>
    void increment_all(const Counter<S> &other) {
        for (const auto &p : other) {
            increment(p.first, p.second);
        }
    }

    [[nodiscard]]
    mapped_type total_count() const {
        if (m_current_mod_count != m_total_count_mod_count) {
            auto total = 0.0;
            for (const auto &p : m_entries) {
                total += p.second;
            }
            m_cache_total_count = total;
            m_total_count_mod_count = m_current_mod_count;
        }
        return m_cache_total_count;
    }

    void normalize() {
        auto count = total_count();
        for (auto &p : m_entries) {
            p.second /= count;
        }
        ++m_current_mod_count;
    }

private:
    const typename map_type::value_type &entry_max() const {
        auto max_count = -std::numeric_limits<mapped_type>::infinity();
        auto max_entry = m_entries.cbegin();
        for (auto it = m_entries.cbegin(); it != m_entries.cend(); ++it) {
            if (it->second > max_count) {
                max_entry = it;
                max_count = it->second;
            }
        }
        return *max_entry;
    }

    const typename map_type::value_type &entry_min() const {
        auto min_count = std::numeric_limits<mapped_type>::infinity();
        auto min_entry = m_entries.cbegin();
        for (auto it = m_entries.cbegin(); it != m_entries.cend(); ++it) {
            if (it->second < min_count) {
                min_entry = it;
                min_count = it->second;
            }
        }
        return *min_entry;
    }

public:
    [[nodiscard]]
    const key_type &arg_max() const {
        return entry_max().first;
    }

    [[nodiscard]]
    mapped_type max() const {
        return entry_max().second;
    }

    [[nodiscard]]
    const key_type &arg_min() const {
        return entry_min().first;
    }

    [[nodiscard]]
    mapped_type min() const {
        return entry_min().second;
    }

public:
    using iterator = typename map_type::const_iterator;

    iterator begin() const { return m_entries.cbegin(); }

    iterator end() const { return m_entries.cend(); }

    /**
     * Returns a vector of the
     * @return
     */
    std::vector<key_type> keys() const {
        std::vector<key_type> keys;
        keys.reserve(m_entries.size());
        for (const auto &p : m_entries) {
            keys.emplace_back(p.first);
        }
        return keys;
    }

private:
    void sort_into_cache() const {
        if (m_current_mod_count != m_sorted_pairs_mod_count) {
            std::vector<const std::pair<const key_type, mapped_type> *> vec(m_entries.size());
            auto i = 0;
            for (const auto &p : m_entries) {
                vec[i] = &p;
                ++i;
            }
            m_cache_sorted_pairs = std::move(vec);
            std::sort(m_cache_sorted_pairs.begin(), m_cache_sorted_pairs.end(),
                      detail::compare<const key_type, mapped_type>);
            m_sorted_pairs_mod_count = m_current_mod_count;
        }
    }

public:
    friend std::ostream &operator<<(std::ostream &out, const Counter<T, Count, Hash> &counter) {
        counter.sort_into_cache();
        out << "[";
        for (auto it = counter.m_cache_sorted_pairs.begin();;) {
            out << (*it)->first;
            out << ":" << (*it)->second;
            ++it;
            if (it != counter.m_cache_sorted_pairs.end()) {
                out << ", ";
            } else {
                break;
            }
        }
        out << "]";
        return out;
    }
};
}

#endif //SGMWSMC_COUNTER_H
