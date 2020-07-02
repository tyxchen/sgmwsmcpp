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
#include <type_traits>
#include <utility>
#include <boost/container_hash/hash.hpp>

#include <parallel_hashmap/phmap.h>

namespace
{
template<typename T, typename V>
bool compare(const std::pair<T, V> *lhs, const std::pair<T, V> *rhs) {
    return lhs->second > rhs->second;
}

}

namespace sgm
{

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
         typename Hash = void>
class Counter
{
    using hasher = std::conditional_t<std::is_void_v<Hash>, boost::hash<T>, Hash>;
public:
    using key_type = T;
    using mapped_type = Count;
    using value_type = std::pair<const T &, mapped_type>;
    using map_type = phmap::flat_hash_map<key_type, mapped_type, hasher>;

private:
    size_t current_mod_count = 1;
    mutable size_t sorted_pairs_mod_count = 0;
    mutable size_t total_count_mod_count = 0;
    mutable std::vector<const std::pair<const key_type, mapped_type> *> cache_sorted_pairs;
    mutable mapped_type cache_total_count = 0.0;

    map_type entries;

public:
    Counter() = default;

    Counter(std::initializer_list<typename map_type::value_type> init) : entries(init) {}

    /**
     * Clears the container.
     */
    void clear() {
        current_mod_count = 1;
        sorted_pairs_mod_count = 0;
        entries.clear();
    }

    /**
     * Returns the number of elements in the container.
     */
    [[nodiscard]]
    size_t size() const {
        return entries.size();
    }

    /**
     * Returns true if the container is empty, false otherwise.
     */
    [[nodiscard]]
    bool empty() const {
        return entries.empty();
    }

    /**
     * Returns the number of instances of the given key stored in the container. If this container is backed by a map,
     * count will always return either 0 or 1.
     * @param key
     * @return
     */
    [[nodiscard]]
    size_t count(const key_type &key) const {
        return entries.count(key);
    }

    mapped_type erase(const key_type &key) {
        auto key_count = get(key);
        entries.erase(key);
        ++current_mod_count;
        return key_count;
    }

    [[nodiscard]]
    mapped_type get(const key_type &key, mapped_type default_count = 0.0) const {
        if (count(key) == 0) return default_count;
        // We have to call unordered_map::at here as operator[] is non-const
        return entries.at(key);
    }

    void set(const key_type &key, mapped_type count) {
        entries[key] = count;
        ++current_mod_count;
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
        if (current_mod_count != total_count_mod_count) {
            auto total = 0.0;
            for (const auto &p : entries) {
                total += p.second;
            }
            cache_total_count = total;
            total_count_mod_count = current_mod_count;
        }
        return cache_total_count;
    }

    void normalize() {
        auto count = total_count();
        for (auto &p : entries) {
            p.second /= count;
        }
        ++current_mod_count;
    }

private:
    const typename map_type::value_type &entry_max() const {
        auto max_count = -std::numeric_limits<mapped_type>::infinity();
        auto max_entry = entries.cbegin();
        for (auto it = entries.cbegin(); it != entries.cend(); ++it) {
            if (it->second > max_count) {
                max_entry = it;
                max_count = it->second;
            }
        }
        return *max_entry;
    }

    const typename map_type::value_type &entry_min() const {
        auto min_count = std::numeric_limits<mapped_type>::infinity();
        auto min_entry = entries.cbegin();
        for (auto it = entries.cbegin(); it != entries.cend(); ++it) {
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

    iterator begin() const { return entries.cbegin(); }

    iterator end() const { return entries.cend(); }

private:
    void sort_into_cache() const {
        if (current_mod_count != sorted_pairs_mod_count) {
            std::vector<const std::pair<const key_type, mapped_type> *> vec(entries.size());
            auto i = 0;
            for (const auto &p : entries) {
                vec[i] = &p;
                ++i;
            }
            cache_sorted_pairs = std::move(vec);
            std::sort(cache_sorted_pairs.begin(), cache_sorted_pairs.end(),
                      ::compare<const key_type, mapped_type>);
            sorted_pairs_mod_count = current_mod_count;
        }
    }

public:
    friend std::ostream &operator<<(std::ostream &out, const Counter<T, Count, Hash> &counter) {
        counter.sort_into_cache();
        out << "[";
        for (auto it = counter.cache_sorted_pairs.begin();;) {
            out << (*it)->first;
            out << ":" << (*it)->second;
            ++it;
            if (it != counter.cache_sorted_pairs.end()) {
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
