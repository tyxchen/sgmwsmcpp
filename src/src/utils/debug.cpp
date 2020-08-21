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

#include "utils/debug.h"

#include <ctime>
#include <iomanip>
#include <stdexcept>

#ifdef COMPILED_WITH_RCPP
#include <Rcpp.h>
#endif

using namespace sgm;

int sgm::count = 0;

performance_timer::performance_timer(bool autostart) {
    if (autostart) {
        start();
    }
}

void performance_timer::start() {
    m_start = std::chrono::high_resolution_clock::now();
}

void performance_timer::end() {
    m_end = std::chrono::high_resolution_clock::now();
}

std::chrono::high_resolution_clock::duration::rep performance_timer::elapsed() const {
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(end - m_start).count();
}

std::chrono::high_resolution_clock::duration::rep performance_timer::diff() const {
    return std::chrono::duration_cast<std::chrono::microseconds>(m_end - m_start).count();
}

static std::unordered_map<std::string, sgm::performance_timer> performance_timers;
static std::unordered_map<std::string, sgm::performance_accumulator> performance_accumulators;

performance_timer &Timers::get_timer(const std::string &id) {
    return performance_timers[id];
}

performance_accumulator &Timers::get_accumulator(const std::string &id) {
    return performance_accumulators[id];
}

void Timers::start(const std::string &id) {
    performance_accumulators[id];  // initialize an accumulator if one hasn't been initialized yet
    performance_timers[id].start();
}

void Timers::end(const std::string &id) {
    performance_timers.at(id).end();
}

performance_timer::clock::duration::rep Timers::elapsed(const std::string &id) {
    return performance_timers[id].elapsed();
}

performance_timer::clock::duration::rep Timers::diff(const std::string &id) {
    return performance_timers[id].diff();
}

void Timers::save(const std::string &id) {
    performance_accumulators[id](performance_timers[id].diff());
}

void Timers::reset(const std::string &id) {
    performance_accumulators[id] = {};
}

double Timers::mean(const std::string &id) {
    return boost::accumulators::mean(performance_accumulators[id]);
}

debugstream::debugstream(std::ostream &tty, const std::string &filename, int precision)
    : tty(tty), log_file(filename, std::ofstream::trunc), precision(precision) {
    if (!init_log())
        throw std::runtime_error("Could not open log file " + filename);
    tty << std::setprecision(precision);
}

bool debugstream::init_log() {
    auto t = std::time(nullptr);
    auto time = std::localtime(&t);

    if (log_file.is_open()) {
        log_file << "Started at " << std::put_time(time, "%c") << "\n\n";
        log_file << std::setprecision(precision);
        return true;
    } else
        return false;
}

void debugstream::set_log_file(const std::string &filename) {
    log_file = std::ofstream(filename, std::ofstream::trunc);
    if (!init_log())
        throw std::runtime_error("Could not open log file " + filename);
}

debugstream &debugstream::operator<<(std::ostream &(*func)(std::ostream &)) {
    func(tty);
    func(log_file);
    return *this;
}

debugstream &debugstream::operator<=(std::ostream &(*func)(std::ostream &)) {
    func(log_file);
    return *this;
}

debugstream sgm::logger(
#ifndef COMPILED_WITH_RCPP
    std::cout,
#else
    Rcpp::Rcout,
#endif
#ifdef _WIN32
    "nul"
#else
    "/dev/null"
#endif
);

runtime_error::runtime_error(const char *msg) : std::runtime_error(msg) {
    sgm::logger <= msg <= std::endl;
}

runtime_error::runtime_error(const std::string &msg) : std::runtime_error(msg) {
    sgm::logger <= msg <= std::endl;
}
