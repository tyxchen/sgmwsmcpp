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

#include "LBFGSMinimizer.h"

#include <cmath>
#include <vector>

using namespace sgm;

LBFGSMinimizer::LBFGSMinimizer() = default;

LBFGSMinimizer::LBFGSMinimizer(int max_iterations, bool verbose) : max_iterations(max_iterations), verbose(verbose) {}

bool detail::is_converged(double value, double next_value, double tolerance) {
    if (value == next_value) return true;

    auto value_change = std::abs(next_value - value);
    auto value_average = std::abs(next_value + value + LBFGSMinimizer::EPS) / 2.0;

    return value_change / value_average < tolerance;
}

void LBFGSMinimizer::update_histories(const Eigen::VectorXd &guess, const Eigen::VectorXd &next_guess,
                                      const Eigen::VectorXd &gradient, const Eigen::VectorXd &next_gradient) {
    auto guess_change = next_guess - guess;
    auto gradient_change = next_gradient - gradient;
    push_onto_list(guess_change, input_difference_vector_list);
    push_onto_list(gradient_change, gradient_difference_vector_list);
}

void LBFGSMinimizer::push_onto_list(Eigen::VectorXd vector, std::deque<Eigen::VectorXd> &vector_list) {
    vector_list.emplace_front(std::move(vector));
    if (vector_list.size() > max_history_size) {
        vector_list.pop_back();
    }
}

Eigen::VectorXd LBFGSMinimizer::implicit_multiply(const Eigen::VectorXd &initial_inverse_hessian_diagonal,
                                                  const Eigen::VectorXd &gradient) const {
    auto size = input_difference_vector_list.size();
    auto rho = std::vector<double>(size);
    auto alpha = std::vector<double>(size);
    auto right = gradient;

    // loop last backward
    // size is unsigned so we run the loop comparison on ind, which is always non-negative
    for (auto ind = size, i = ind - 1; ind > 0; --ind, --i) {
        auto input_difference = input_difference_vector_list[i];
        auto gradient_difference = gradient_difference_vector_list[i];
        rho[i] = input_difference.dot(gradient_difference);
        if (rho[i] == 0.0)
            throw sgm::runtime_error("LBFGSMinimizer::implicit_multiply: Curvature problem.");
        alpha[i] = input_difference.dot(right) / rho[i];
        right -= alpha[i] * gradient_difference;
    }

    auto left = static_cast<Eigen::VectorXd>(initial_inverse_hessian_diagonal.cwiseProduct(right));

    // loop forwards
    for (auto i = 0; i < size; ++i) {
        auto input_difference = input_difference_vector_list[i];
        auto gradient_difference = gradient_difference_vector_list[i];
        auto beta = gradient_difference.dot(left) / rho[i];
        left += (alpha[i] - beta) * input_difference;
    }

    return left;
}
