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

#ifndef SGMWSMCPP_VECTORLIST_H
#define SGMWSMCPP_VECTORLIST_H

#include <cstring>
#include <cstddef>
#include <cstdlib>
#include <iterator>
#include <type_traits>
#include <utility>

namespace sgm
{

namespace _vector_list_detail
{

template <typename T>
void _destroy_nodes(T *array, size_t n) {
    for (auto i = 0ul; i < n; ++i) {
        (array + i)->~T();
    }
}

template <typename T>
void _uninitialized_move(size_t *start, size_t *end, T * dest, size_t *ptr_src, T *src) {
    auto progress = 0u;
    try {
        for (; start + progress != end; ++progress) {
            new (dest + progress) T(std::move(src[ptr_src[progress]]));
            start[progress] = progress;
        }
    } catch (...) {
        _destroy_nodes(dest, progress);
        throw;
    }
}

template <typename T>
void _uninitialized_copy(size_t *start, size_t *end, T * dest, size_t *ptr_src, T *src) {
    auto progress = 0u;
    try {
        for (; start + progress != end; ++progress) {
            new (dest + progress) T(src[ptr_src[progress]]);
            start[progress] = progress;
        }
    } catch (...) {
        _destroy_nodes(dest, progress);
        throw;
    }
}

template <typename T, typename It>
void _uninitialized_copy(size_t *start, size_t *end, T *dest, It src) {
    auto progress = 0u;
    try {
        for (; start + progress != end; ++progress, ++src) {
            new (dest + progress) T(*src);
            start[progress] = progress;
        }
    } catch (...) {
        _destroy_nodes(dest, progress);
        throw;
    }
}

}

/**
 * A cache-friendly data structure similar to boost::stable_vector that offers fast insertion and deletion by only
 * modifying an index array on those operations, thus avoiding expensive re-initialization of objects. This comes
 * with a memory cost as the internal storage array grows in size much quicker than in a normal vector, and we also
 * must store an additional array.
 *
 * Internally, the memory layout of this vector may initially look like this:
 *
 *                      --------------------------
 *     storage vector:  | 12 | 4  | -1 | 36 | 12 |
 *                      --------------------------
 *                        ^    ^    ^    ^    ^
 *                      --------------------------
 *     index vector:    | 0  | 1  | 2  | 3  | 4  |
 *                      --------------------------
 *                        ^    ^    ^    ^    ^
 *     index:             0    1    2    3    4
 *
 * Suppose we call v.erase(2), erasing the element storing -1. Where traditional vector implementations would perform
 * an expensive shift on the storage vector, re-initializing O(n) objects, we simply shift the index vector:
 *
 *                      --------------------------
 *     storage vector:  | 12 | 4  |    | 36 | 12 |
 *                      --------------------------
 *                        ^    ^      ~~~^ ~~~^
 *                        |    |     /    /
 *                      ---------------------
 *     index vector:    | 0  | 1  | 3  | 4  |
 *                      ---------------------
 *                        ^    ^    ^    ^
 *     index:             0    1    2    3
 *
 * Similarly, if we call v.insert(0, 5), thus inserting 5 into the front of the list, we shift the index vector and
 * perform a cheap add to the end of the storage vector:
 *
 *                      -------------------------------
 *     storage vector:  | 12 | 4  |    | 36 | 12 | 5  |
 *                      -------------------------------
 *                        ^~~~ ^~~~      ^    ^    ^
 *                         ___\____\_____|____|___/
 *                        /    \    \    |    |
 *                      --------------------------
 *     index vector:    | 5  | 0  | 1  | 3  | 4  |
 *                      --------------------------
 *                        ^    ^    ^    ^    ^
 *     index:             0    1    2    3    4
 *
 * The simplicity of a shift on primitive types also lets us use fast copy methods such as memcpy and memmove
 * when possible.
 *
 * @tparam T The type stored in this vector.
 */
template <typename T>
class vector_list
{
private:
    using value_type = T;
    using reference = value_type &;
    using const_reference = const value_type &;
    using pointer = value_type *;
    using const_pointer = const value_type *;
    using node_ptr = size_t *;
    size_t n;
    size_t int_n;
    size_t cap;
    size_t *m_ptr_array;
    pointer m_internal_array;

public:
    struct iterator
    {
        friend class vector_list;

        using iterator_category = std::random_access_iterator_tag;
        iterator() = default;
        reference operator*() {
            return internal[*ptr];
        }
        pointer operator->() {
            return &internal[*ptr];
        }
        reference operator[](size_t idx) {
            return internal[ptr[idx]];
        }
        iterator &operator++() {
            ++ptr;
            return *this;
        }
        iterator &operator--() {
            --ptr;
            return *this;
        }
        friend bool operator==(const iterator &lhs, const iterator &rhs) {
            return lhs.ptr == rhs.ptr;
        }
        friend bool operator!=(const iterator &lhs, const iterator &rhs) {
            return !(lhs == rhs);
        }
        friend bool operator<(const iterator &lhs, const iterator &rhs) {
            return lhs.ptr < rhs.ptr;
        }
        friend bool operator>(const iterator &lhs, const iterator &rhs) {
            return rhs < lhs;
        }
        iterator &operator+=(ptrdiff_t offset) {
            ptr += offset;
            return *this;
        }
        iterator &operator-=(ptrdiff_t offset) {
            return operator+=(-offset);
        }
        friend iterator operator+(const iterator &lhs, ptrdiff_t rhs) {
            auto ret = iterator(lhs.ptr);
            ret += rhs;
            return ret;
        }
        friend iterator operator+(ptrdiff_t lhs, const iterator &rhs) {
            return rhs + lhs;
        }
        friend ptrdiff_t operator-(const iterator &lhs, const iterator &rhs) {
            return lhs.ptr - rhs.ptr;
        }
        friend iterator operator-(const iterator &lhs, ptrdiff_t rhs) {
            auto ret = iterator(lhs.ptr);
            ret -= rhs;
            return ret;
        }

    private:
        node_ptr ptr;
        pointer internal;
        iterator(node_ptr _ptr, pointer _internal) : ptr(_ptr), internal(_internal) {}
    };

    struct const_iterator
    {
        friend class vector_list;

        using iterator_category = std::random_access_iterator_tag;
        const_iterator() = default;
        const_reference operator*() const {
            return internal[*ptr];
        }
        const_pointer operator->() const {
            return &internal[*ptr];
        }
        const_reference operator[](size_t idx) const {
            return internal[ptr[idx]];
        }
        const_iterator &operator++() {
            ++ptr;
            return *this;
        }
        const_iterator &operator--() {
            --ptr;
            return *this;
        }
        friend bool operator==(const const_iterator &lhs, const const_iterator &rhs) {
            return lhs.ptr == rhs.ptr;
        }
        friend bool operator!=(const const_iterator &lhs, const const_iterator &rhs) {
            return !(lhs == rhs);
        }
        friend bool operator<(const const_iterator &lhs, const const_iterator &rhs) {
            return lhs.ptr < rhs.ptr;
        }
        friend bool operator>(const const_iterator &lhs, const const_iterator &rhs) {
            return rhs < lhs;
        }
        const_iterator &operator+=(ptrdiff_t offset) {
            ptr += offset;
            return *this;
        }
        const_iterator &operator-=(ptrdiff_t offset) {
            return operator+=(-offset);
        }
        friend const_iterator operator+(const const_iterator &lhs, ptrdiff_t rhs) {
            auto ret = const_iterator(lhs.ptr);
            ret += rhs;
            return ret;
        }
        friend const_iterator operator+(ptrdiff_t lhs, const const_iterator &rhs) {
            return rhs + lhs;
        }
        friend ptrdiff_t operator-(const const_iterator &lhs, const const_iterator &rhs) {
            return lhs.ptr - rhs.ptr;
        }
        friend const_iterator operator-(const const_iterator &lhs, ptrdiff_t rhs) {
            auto ret = const_iterator(lhs.ptr);
            ret -= rhs;
            return ret;
        }

    private:
        node_ptr ptr;
        pointer internal;
        const_iterator(node_ptr _ptr, pointer _internal) : ptr(_ptr), internal(_internal) {}
    };

    vector_list() : n(0), int_n(0), cap(2), m_ptr_array(new size_t[2]),
                    m_internal_array(static_cast<pointer>(operator new(cap * sizeof(value_type)))) {}

    template <typename RAIt>
    vector_list(RAIt begin, RAIt end) : n(end - begin), int_n(n), cap(n),
                                        m_ptr_array(new size_t[cap]),
                                        m_internal_array(static_cast<pointer>(operator new(cap * sizeof(value_type)))) {
        try {
            _vector_list_detail::_uninitialized_copy(m_ptr_array, m_ptr_array + n, m_internal_array, begin);
        } catch (...) {
            delete[] m_ptr_array;
            operator delete (m_internal_array);
            throw;
        }
    }

    vector_list(const vector_list<T> &other) : n(other.n), int_n(n), cap(other.cap),
                                               m_ptr_array(new size_t[cap]),
                                               m_internal_array(
                                                   static_cast<pointer>(operator new(cap * sizeof(value_type)))) {
        try {
            _vector_list_detail::_uninitialized_copy(m_ptr_array, m_ptr_array + n, m_internal_array,
                                                     other.m_ptr_array, other.m_internal_array);
        } catch (...) {
            delete[] m_ptr_array;
            operator delete (m_internal_array);
            throw;
        }
    }

    vector_list(vector_list<T> &&other) noexcept : n(other.n), int_n(other.int_n), cap(other.cap),
                                                   m_ptr_array(other.m_ptr_array),
                                                   m_internal_array(other.m_internal_array) {
        other.n = 0;
        other.int_n = 0;
        other.cap = 0;
        other.m_ptr_array = nullptr;
        other.m_internal_array = nullptr;
    }

    vector_list &operator=(vector_list<T> other) {
        std::swap(n, other.n);
        std::swap(int_n, other.int_n);
        std::swap(cap, other.cap);
        std::swap(m_ptr_array, other.m_ptr_array);
        std::swap(m_internal_array, other.m_internal_array);
    }

    ~vector_list() {
        _vector_list_detail::_destroy_nodes(m_internal_array, n);
        delete[] m_ptr_array;
        operator delete (m_internal_array);
    }

    size_t size() const {
        return n;
    }

    size_t capacity() const {
        return cap;
    }

    bool empty() const {
        return n == 0;
    }

    void reserve(size_t new_cap) {
        if (new_cap <= cap) return;

        auto new_ptr_array = new size_t[new_cap];
        auto new_internal_array = static_cast<pointer>(operator new(new_cap * sizeof(value_type)));

        try {
            _vector_list_detail::_uninitialized_move(new_ptr_array, new_ptr_array + n, new_internal_array,
                                                     m_ptr_array, m_internal_array);
        } catch (...) {
            delete[] new_ptr_array;
            operator delete(new_internal_array);
            throw;
        }

        std::swap(m_ptr_array, new_ptr_array);
        std::swap(m_internal_array, new_internal_array);
        delete[] new_ptr_array;
        operator delete(new_internal_array);
        int_n = n;
        cap = new_cap;
    }

    void erase(size_t pos) {
        m_internal_array[m_ptr_array[pos]].~T();
        if (pos < n - 1) {
            memmove(m_ptr_array + pos, m_ptr_array + pos + 1, (n - pos - 1) * sizeof(size_t));
        }
        --n;
    }

    void insert(size_t pos, value_type obj) {
        if (int_n == cap) {
            reserve(2 * cap);
        }
        if (pos < n) {
            memmove(m_ptr_array + pos + 1, m_ptr_array + pos, (n - pos) * sizeof(size_t));
        }
        new (m_internal_array + int_n) value_type(std::move(obj));
        *(m_ptr_array + pos) = int_n;
        ++int_n;
        ++n;
    }

    template <typename... Args>
    void emplace(size_t pos, Args&&... args) {
        if (int_n == cap) {
            reserve(2 * cap);
        }
        if (pos < n) {
            memmove(m_ptr_array + pos + 1, m_ptr_array + pos, (n - pos) * sizeof(size_t));
        }
        new (m_internal_array + int_n) value_type(std::forward<Args>(args)...);
        *(m_ptr_array + pos) = int_n;
        ++int_n;
        ++n;
    }

    void pop_back() {
        erase(n - 1);
    }

    void push_back(value_type obj) {
        insert(n, std::move(obj));
    }

    template <typename... Args>
    void emplace_back(Args&&... args) {
        emplace(n, std::forward<Args>(args)...);
    }

    reference operator[](size_t idx) {
        return m_internal_array[m_ptr_array[idx]];
    }

    const_reference operator[](size_t idx) const {
        return m_internal_array[m_ptr_array[idx]];
    }

    reference front() {
        return m_internal_array[m_ptr_array[0]];
    }

    const_reference front() const {
        return m_internal_array[m_ptr_array[0]];
    }

    reference back() {
        return m_internal_array[m_ptr_array[n - 1]];
    }

    const_reference back() const {
        return m_internal_array[m_ptr_array[n - 1]];
    }

    iterator begin() {
        return iterator(m_ptr_array, m_internal_array);
    }

    const_iterator begin() const {
        return const_iterator(m_ptr_array, m_internal_array);
    }

    iterator end() {
        return iterator(m_ptr_array + n, m_internal_array);
    }

    const_iterator end() const {
        return const_iterator(m_ptr_array + n, m_internal_array);
    }
};

}

#endif //SGMWSMCPP_VECTORLIST_H
