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
#include <utility>

namespace sgm
{

namespace _vector_list_detail
{

template <typename T>
void _destroy_nodes(T **array, size_t n) {
    for (auto i = 0ul; i < n; ++i) {
        delete *(array + i);
    }
}

template <typename T>
void _initialized_copy(T **start, T **end, T **src) {
    auto progress = 0ul;
    try {
        for (; start + progress != end; ++progress) {
            *(start + progress) = new T(**(src + progress));
        }
    } catch (...) {
        for (auto p = 0ul; p != progress; ++p) {
            delete *(start + p);
        }
        throw;
    }
}

template <typename T, typename It>
void _initialized_copy(T **start, T **end, It src) {
    auto progress = 0ul;
    try {
        for (; start + progress != end; ++progress, ++src) {
            *(start + progress) = new T(*src);
        }
    } catch (...) {
        for (auto p = 0ul; p != progress; ++p) {
            delete *(start + p);
        }
        throw;
    }
}

}

template <typename T>
class vector_list
{
private:
    using value_type = T;
    using reference = value_type &;
    using const_reference = const value_type &;
    using pointer = value_type *;
    using const_pointer = const value_type *;
    using node_ptr = value_type **;
    size_t n;
    size_t cap;
    node_ptr m_ptr_array;

public:
    struct iterator
    {
        friend class vector_list;

        using iterator_category = std::random_access_iterator_tag;
        iterator() = default;
        reference operator*() {
            return **ptr;
        }
        pointer operator->() {
            return *ptr;
        }
        reference operator[](size_t idx) {
            return **(ptr + idx);
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
        explicit iterator(node_ptr _ptr) : ptr(_ptr) {}
    };

    struct const_iterator
    {
        friend class vector_list;

        using iterator_category = std::random_access_iterator_tag;
        const_iterator() = default;
        const_reference operator*() const {
            return **ptr;
        }
        const_pointer operator->() const {
            return *ptr;
        }
        const_reference operator[](size_t idx) const {
            return **(ptr + idx);
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
        explicit const_iterator(node_ptr _ptr) : ptr(_ptr) {}
    };

    vector_list() : n(0), cap(2), m_ptr_array(new pointer[2]) {}

    template <typename RAIt>
    vector_list(RAIt begin, RAIt end) : n(end - begin), cap(n), m_ptr_array(new pointer[cap]) {
        try {
            _vector_list_detail::_initialized_copy(m_ptr_array, m_ptr_array + n, begin);
        } catch (...) {
            delete[] m_ptr_array;
            throw;
        }
    }

    vector_list(const vector_list<T> &other) : n(other.n), cap(other.cap), m_ptr_array(new pointer[cap]) {
        try {
            _vector_list_detail::_initialized_copy(m_ptr_array, m_ptr_array + n, other.m_ptr_array);
        } catch (...) {
            delete[] m_ptr_array;
            throw;
        }
    }

    vector_list(vector_list<T> &&other) noexcept : n(other.n), cap(other.cap), m_ptr_array(other.m_ptr_array) {
        other.n = 0;
        other.cap = 0;
        other.m_ptr_array = nullptr;
    }

    vector_list &operator=(vector_list<T> other) {
        std::swap(n, other.n);
        std::swap(cap, other.cap);
        std::swap(m_ptr_array, other.m_ptr_array);
    }

    ~vector_list() {
        _vector_list_detail::_destroy_nodes(m_ptr_array, n);
        delete[] m_ptr_array;
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
        auto new_ptr_array = new pointer[new_cap];
        memcpy(new_ptr_array, m_ptr_array, n * sizeof(pointer));
        std::swap(m_ptr_array, new_ptr_array);
        delete[] new_ptr_array;
        cap = new_cap;
    }

    void erase(size_t pos) {
        delete *(m_ptr_array + pos);
        if (pos < n - 1) {
            memmove(m_ptr_array + pos, m_ptr_array + pos + 1, (n - pos - 1) * sizeof(pointer));
        }
        --n;
    }

    void insert(size_t pos, value_type obj) {
        if (n == cap) {
            reserve(2 * cap);
        }
        if (pos < n) {
            memmove(m_ptr_array + pos + 1, m_ptr_array + pos, (n - pos) * sizeof(pointer));
        }
        *(m_ptr_array + pos) = new value_type(std::move(obj));
        ++n;
    }

    template <typename... Args>
    void emplace(size_t pos, Args&&... args) {
        if (n == cap) {
            reserve(2 * cap);
        }
        if (pos < n) {
            memmove(m_ptr_array + pos + 1, m_ptr_array + pos, (n - pos) * sizeof(pointer));
        }
        *(m_ptr_array + pos) = new value_type(std::forward<Args>(args)...);
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
        return **(m_ptr_array + idx);
    }

    const_reference operator[](size_t idx) const {
        return **(m_ptr_array + idx);
    }

    reference front() {
        return **m_ptr_array;
    }

    const_reference front() const {
        return **m_ptr_array;
    }

    reference back() {
        return **(m_ptr_array + n - 1);
    }

    const_reference back() const {
        return **(m_ptr_array + n - 1);
    }

    iterator begin() {
        return iterator(m_ptr_array);
    }

    const_iterator begin() const {
        return const_iterator(m_ptr_array);
    }

    iterator end() {
        return iterator(m_ptr_array + n);
    }

    const_iterator end() const {
        return const_iterator(m_ptr_array + n);
    }
};

}

#endif //SGMWSMCPP_VECTORLIST_H
