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

#ifndef SGMWSMCPP_DEBUG_H
#define SGMWSMCPP_DEBUG_H

#include <chrono>
#include <functional>
#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include <unordered_map>
#include <Eigen/Core>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

#include "utils/types.h"
#include "utils/debug_macros.h"

namespace sgm
{

using performance_accumulator =
    typename boost::accumulators::accumulator_set<long, boost::accumulators::stats<boost::accumulators::tag::mean>>;

extern int count;

class performance_timer
{
public:
    using clock = std::chrono::high_resolution_clock;

private:
    clock::time_point m_start;
    clock::time_point m_end;

public:
    explicit performance_timer(bool autostart = false);
    void start();
    void end();
    clock::duration::rep elapsed() const;
    clock::duration::rep diff() const;
};

namespace Timers
{

performance_timer &get_timer(const std::string &id);

performance_accumulator &get_accumulator(const std::string &id);

void start(const std::string &id);

void end(const std::string &id);

performance_timer::clock::duration::rep elapsed(const std::string &id);

performance_timer::clock::duration::rep diff(const std::string &id);

void save(const std::string &id);

void reset(const std::string &id);

double mean(const std::string &id);

}

class debugstream
{
    std::ostream &tty;
    std::ofstream log_file;
    int precision;

    bool init_log();

public:
    debugstream(std::ostream &tty, const std::string &filename, int precision = 8);

    void set_log_file(const std::string &filename);

    template <typename T>
    debugstream &operator<<(const T &thing) {
        tty << thing;
        log_file << thing;
        return *this;
    }

    debugstream &operator<<(std::ostream &(*func)(std::ostream &));

    template <typename T>
    debugstream &operator<=(const T &thing) {
        log_file << thing;
        return *this;
    }

    debugstream &operator<=(std::ostream &(*func)(std::ostream &));
};

extern debugstream logger;

struct runtime_error : public std::runtime_error
{
    explicit runtime_error(const char *msg);
    explicit runtime_error(const std::string &msg);
};

template <typename T>
DECLARE_PRINT_OVERLOAD(sgm::edge_type_base<T>);

template <typename T, typename U>
DECLARE_PRINT_OVERLOAD(std::pair<T, U>);

template <typename T>
DECLARE_PRINT_OVERLOAD(std::vector<T>);

template <typename T>
DECLARE_PRINT_OVERLOAD(sgm::set_t<T>);

template <typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
DECLARE_PRINT_OVERLOAD(Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>);

template <typename K, typename T>
DECLARE_PRINT_OVERLOAD_WITH_SEP(",\n ", sgm::map_t<K, T>);

namespace detail
{

template <typename T, typename = std::void_t<>>
struct print_exists : std::false_type
{
};

template <typename T>
struct print_exists<T, std::void_t<decltype(print(std::declval<std::ostream &>(),
                                                  std::declval<const T &>(),
                                                  std::declval<const std::string &>()))>>
    : std::true_type
{
};

template <typename T>
constexpr bool print_exists_v = print_exists<T>::value;

}

template <typename T, std::enable_if_t<detail::print_exists_v<T>, int> = 0>
void use_print_if_exists(std::ostream &out, const T &obj) {
    print(out, obj, ", ");
}

template <typename T, std::enable_if_t<!detail::print_exists_v<T>, int> = 0>
void use_print_if_exists(std::ostream &out, const T &obj) {
    out << obj;
}

template <typename T>
DEFINE_PRINT_OVERLOAD(edge, sgm::edge_type_base<T>) {
    return print(out, *edge, sep);
}

template <typename T, typename U>
DEFINE_PRINT_OVERLOAD(pair, std::pair<T, U>) {
    out << "(" << pair.first << sep << pair.second << ")";
    return out;
}

template <typename T>
DEFINE_PRINT_OVERLOAD(set, std::vector<T>) {
    auto i = 0u;
    auto s = set.size();
    out << "[";
    for (const auto &e : set) {
        use_print_if_exists(out, e);
        ++i;
        if (i < s) out << sep;
    }
    out << "]";
    return out;
}

template <typename T>
DEFINE_PRINT_OVERLOAD(set, sgm::set_t<T>) {
    auto i = 0u;
    auto s = set.size();
    out << "{";
    for (const auto &e : set) {
        use_print_if_exists(out, e);
        ++i;
        if (i < s) out << sep;
    }
    out << "}";
    return out;
}

template <typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
DEFINE_PRINT_OVERLOAD(matrix, Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>) {
    auto fmt = Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", sep, "", "", "[", "]");
    out << matrix.format(fmt);
    return out;
}

template <typename K, typename T>
DEFINE_PRINT_OVERLOAD(map, sgm::map_t<K, T>) {
    auto i = 0u;
    auto s = map.size();
    out << "{";
    for (const auto &e : map) {
        use_print_if_exists(out, e.first);
        out << " -> ";
        use_print_if_exists(out, e.second);
        ++i;
        if (i < s) out << sep;
    }
    out << "}";
    return out;
}

template <typename T>
std::function<std::ostream &(std::ostream &)> print_wrapper(const T &obj, const std::string &sep) {
    return [&](std::ostream &out) -> std::ostream & {
        return print(out, obj, sep);
    };
}

template <typename T>
std::function<std::ostream &(std::ostream &)> print_wrapper(const T &obj) {
    return [&](std::ostream &out) -> std::ostream & {
        return print(out, obj);
    };
}

}

template <typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits> &operator<<(std::basic_ostream<CharT, Traits> &os,
                                              const std::function<std::basic_ostream<CharT, Traits> &(
                                                  std::basic_ostream<CharT, Traits> &)> &func) {
    return func(os);
}

template <typename T, typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits> &operator<<(std::basic_ostream<CharT, Traits> &out,
                                              const sgm::node_type_base<T> &node) {
    out << *node;
    return out;
}

template <typename T, typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits> &operator<<(std::basic_ostream<CharT, Traits> &out,
                                              const sgm::edge_type_base<T> &edge) {
    out << *edge;
    return out;
}

template <typename T, typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits> &operator<<(std::basic_ostream<CharT, Traits> &out, const std::vector<T> &set) {
    out << sgm::print_wrapper(set);
    return out;
}

template <typename T, typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits> &operator<<(std::basic_ostream<CharT, Traits> &out, const sgm::set_t<T> &set) {
    out << sgm::print_wrapper(set);
    return out;
}

template <typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits> &operator<<(std::basic_ostream<CharT, Traits> &out, const Eigen::VectorXd &vec) {
    out << sgm::print_wrapper(vec);
    return out;
}

template <typename K, typename T, typename CharT, typename Traits = std::char_traits<CharT>>
std::basic_ostream<CharT, Traits> &operator<<(std::basic_ostream<CharT, Traits> &out, const sgm::map_t<K, T> &map) {
    out << sgm::print_wrapper(map);
    return out;
}

#endif //SGMWSMCPP_DEBUG_H
