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

#include "CompactPopulation.h"

#include <cmath>
#include "utils/NumericalUtils.h"

using namespace sgm;

size_t smc::CompactPopulation::num_particles() const {
    return m_num_particles;
}

double smc::CompactPopulation::log_sum() const {
    return m_log_sum;
}

double smc::CompactPopulation::log_sum_of_squares() const {
    return m_log_sum_of_squares;
}

void smc::CompactPopulation::insert_log_weight(double log_weight) {
    ++m_num_particles;
    m_log_sum = NumericalUtils::log_add(m_log_sum, log_weight);
    m_log_sum_of_squares = NumericalUtils::log_add(m_log_sum_of_squares, 2 * log_weight);
}

double smc::CompactPopulation::ess() const {
    return std::exp(2 * m_log_sum - m_log_sum_of_squares);
}

double smc::CompactPopulation::logZ_estimate() const {
    return m_log_sum - std::log(m_num_particles);
}

void smc::CompactPopulation::clear() {
    m_num_particles = 0;
    m_log_sum = Consts::NEGATIVE_INFINITY;
    m_log_sum_of_squares = Consts::NEGATIVE_INFINITY;
}
