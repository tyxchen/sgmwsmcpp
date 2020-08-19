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

#include "ExpUtils.h"
#include <iostream>

using namespace sgm;

int ExpUtilsConfig::concrete_particles = 1000;
int ExpUtilsConfig::max_virtual_particles = 10000;
std::string ExpUtilsConfig::output_path = "output/knot-matching/";
double ExpUtilsConfig::lambda = 1.0;
double ExpUtilsConfig::tol = 1e-6;
bool ExpUtilsConfig::exact_sampling = true;
bool ExpUtilsConfig::sequential_matching = true;

std::vector<std::vector<KnotDataReader::Segment>> ExpUtils::read_test_boards(const std::vector<std::string> &boards,
                                                                             bool reverse_sequence) {
    std::vector<std::vector<KnotDataReader::Segment>> instances;
    auto idx = 1;
    for (const auto &board: boards) {
        std::cout << idx << ": ";
        instances.emplace_back(KnotDataReader::read_segmented_test_board(board));
        ++idx;
    }
    return instances;
}

