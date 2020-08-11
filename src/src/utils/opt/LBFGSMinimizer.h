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

#ifndef SGMWSMCPP_LBFGSMINIMIZER_H
#define SGMWSMCPP_LBFGSMINIMIZER_H

#include <deque>
#include <Eigen/Core>

#include "utils/debug.h"
#include "utils/opt/BacktrackingLineSearcher.h"

namespace sgm
{

namespace detail {

bool is_converged(double value, double next_value, double tolerance);

}

class LBFGSMinimizer
{
    int max_iterations = 20;
    int max_history_size = 5;
    std::deque<Eigen::VectorXd> input_difference_vector_list;
    std::deque<Eigen::VectorXd> gradient_difference_vector_list;
    bool verbose = false;

    bool converged = false;

public:
    static constexpr double EPS = 1e-10;

    LBFGSMinimizer();

    explicit LBFGSMinimizer(int max_iterations, bool verbose = false);

    template <typename DifferentiableFunction>
    Eigen::VectorXd minimize(DifferentiableFunction &function, const Eigen::VectorXd &initial, double tolerance) {
        BacktrackingLineSearcher line_searcher;
        converged = false;
        auto guess = initial;
        auto value = function.value_at(guess);
        auto gradient = function.derivative_at(guess);

        for (auto iteration = 0; iteration < max_iterations; ++iteration) {
#ifndef DEBUG
            if (verbose)
#endif
                sgm::logger << "Value before iteration " << iteration << ": " << value << "\n";

            auto initial_inverse_hessian_diagonal = get_initial_inverse_hessian_diagonal(function);
            auto direction = implicit_multiply(initial_inverse_hessian_diagonal, gradient);
            direction *= -1;

//            sgm::logger << "guess: " << guess << "\n";
//            sgm::logger << "gradient: " << gradient << "\n";
//            sgm::logger << "direction: " << direction << "\n";

            if (iteration == 0)
                line_searcher.set_step_size_multiplier(0.01);
            else
                line_searcher.set_step_size_multiplier(0.5);

            auto next_guess = line_searcher.minimize(function, guess, direction);
            auto next_value = function.value_at(next_guess);
            auto next_gradient = function.derivative_at(next_guess);

            if (detail::is_converged(value, next_value, tolerance)) {
                converged = true;
                return next_guess;
            }

            update_histories(guess, next_guess, gradient, next_gradient);
            guess = next_guess;
            value = next_value;
            gradient = next_gradient;
        }
        sgm::logger << "LBFGSMinimizer::minimizer: Exceeded max_iterations without converging\n";
        return guess;
    }

private:
    void update_histories(const Eigen::VectorXd &guess, const Eigen::VectorXd &next_guess,
                          const Eigen::VectorXd &gradient, const Eigen::VectorXd &next_gradient);

    void push_onto_list(Eigen::VectorXd vector, std::deque<Eigen::VectorXd> &vector_list);

    Eigen::VectorXd implicit_multiply(const Eigen::VectorXd &initial_inverse_hessian_diagonal,
                                      const Eigen::VectorXd &gradient) const;

    template <typename DifferentiableFunction>
    Eigen::VectorXd get_initial_inverse_hessian_diagonal(DifferentiableFunction &function) {
        auto scale = 1.0;
        if (!gradient_difference_vector_list.empty()) {
            auto last_derivative_difference = gradient_difference_vector_list.front();
            auto last_input_difference = input_difference_vector_list.front();
            auto num = last_derivative_difference.dot(last_input_difference);
            auto den = last_derivative_difference.dot(last_derivative_difference);
            scale = num / den;
        }
        return Eigen::VectorXd::Constant(function.dim(), scale);
    }
};

}

#endif //SGMWSMCPP_LBFGSMINIMIZER_H
