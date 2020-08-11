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

using namespace sgm;

int sgm::count = 0;

debugstream::debugstream(std::ostream &tty, const std::string &filename, int precision)
    : tty(tty), log_file(filename, std::ofstream::trunc), precision(precision) {
    auto t = std::time(nullptr);
    auto time = std::localtime(&t);

    if (log_file.is_open())
        log_file << "Started at " << std::put_time(time, "%c") << "\n\n";
    else
        throw std::runtime_error("could not open log file " + filename);

    tty << std::setprecision(precision);
    log_file << std::setprecision(precision);
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

debugstream sgm::logger(std::cout, "log.txt");

runtime_error::runtime_error(const char *msg) : std::runtime_error(msg) {
    sgm::logger <= msg <= std::endl;
}

runtime_error::runtime_error(const std::string &msg) : std::runtime_error(msg) {
    sgm::logger <= msg <= std::endl;
}
