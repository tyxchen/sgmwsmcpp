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

#ifndef SGMWSMCPP_CONSTSTR_H
#define SGMWSMCPP_CONSTSTR_H

#include <cstring>
#include <iostream>

#include "utils/hash.h"

namespace sgm
{

class conststr
{
    const char *m_data;
    size_t m_hash;

public:
    constexpr conststr() noexcept : m_data(""), m_hash(0) {}
    constexpr conststr(const char *data, size_t hash) noexcept : m_data(data), m_hash(hash) {}
    constexpr char operator*() const noexcept { return *m_data; }
    constexpr char operator[](size_t idx) const noexcept { return m_data[idx]; }
    constexpr const char *data() const noexcept { return m_data; }
    constexpr size_t hash() const noexcept { return m_hash; }
    friend constexpr bool operator==(const conststr &lhs, const conststr &rhs) noexcept {
        return lhs.m_hash == rhs.m_hash;
    }
    friend bool operator==(const conststr &lhs, const char *rhs) noexcept {
        return strcmp(lhs.m_data, rhs) == 0;
    }
    friend bool operator==(const char *lhs, const conststr &rhs) noexcept {
        return strcmp(lhs, rhs.m_data) == 0;
    }
    friend std::ostream &operator<<(std::ostream &out, const conststr &rhs) {
        out << rhs.m_data;
        return out;
    }
};

template <>
struct hash<conststr>
{
public:
    constexpr size_t operator()(const conststr &str) const noexcept {
        return str.hash();
    }
};

}

#endif //SGMWSMCPP_CONSTSTR_H
