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

#ifndef SGMWSMCPP_BACKTRACKINGLINESEARCHER_H
#define SGMWSMCPP_BACKTRACKINGLINESEARCHER_H

#include <Eigen/Core>

#include "utils/debug.h"

namespace sgm
{

class BacktrackingLineSearcher
{
    static constexpr double sufficient_decrease_constant = 1e-4;
    static constexpr double EPS = 1e-10;
    double step_size_multiplier = 0.9;

public:
    double set_step_size_multiplier(double value);

    template <typename DifferentiableFunction>
    Eigen::VectorXd minimize(DifferentiableFunction &function,
                             const Eigen::VectorXd &initial, const Eigen::VectorXd &direction) const {
        auto step_size = 1.0;
        auto initial_function_value = function.value_at(initial);
        auto initial_gradient = function.derivative_at(initial).dot(direction);
        auto guess = Eigen::VectorXd();
        auto guess_value = 0.0;
        auto sufficient_decrease_obtained = false;

//#ifdef DEBUG
//        sgm::logger << "BLS initial: " << initial << "\n";
//        sgm::logger << "BLS direction: " << direction << "\n";
//        sgm::logger << "BLS initial_function_value: " << initial_function_value << "\n";
//        sgm::logger << "BLS initial_gradient: " << initial_gradient << "\n";
//#endif

        while (!sufficient_decrease_obtained) {
            guess = initial + step_size * direction;
            guess_value = function.value_at(guess);
            auto sufficient_decrease_value = initial_function_value +
                sufficient_decrease_constant * initial_gradient * step_size;

//#ifdef DEBUG
//            sgm::logger << "BLS guess: " << guess << "\n";
//            sgm::logger << "BLS guess value: " << guess_value << "\n";
//            sgm::logger << "BLS suff decr value: " << sufficient_decrease_value << "\n";
//            sgm::logger << "BLS step size: " << step_size << "\n";
//#endif

            sufficient_decrease_obtained = guess_value <= sufficient_decrease_value;
            if (!sufficient_decrease_obtained) {
                step_size *= step_size_multiplier;
                if (step_size < EPS) {
                    sgm::logger << "BacktrackingLineSearcher::minimize: step_size underflow.\n";
                    return initial;
                }
            }
        }
        return guess;
    }
};

}

#endif //SGMWSMCPP_BACKTRACKINGLINESEARCHER_H
