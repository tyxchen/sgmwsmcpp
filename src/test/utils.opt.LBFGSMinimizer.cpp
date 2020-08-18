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

#include "catch.hpp"

#include <cmath>
#include <Eigen/Core>

#include "utils/opt/LBFGSMinimizer.h"

class Rosenbrock
{
    int n;
    double *t1s;
    double *t2s;
public:
    Rosenbrock(int n) : n(n), t1s(new double[n]), t2s(new double[n]) {}

    double value_at(const Eigen::VectorXd &vec) {
        auto fx = 0.0;
        for (auto i = 0; i < n; i += 2) {
            auto t1 = 1.0 - vec(i);
            auto t2 = 10 * (vec(i + 1) - vec(i) * vec(i));
            t1s[i] = t1;
            t2s[i] = t2;
            fx += t1 * t1 + t2 * t2;
        }
        return fx;
    }

    Eigen::VectorXd derivative_at(const Eigen::VectorXd &vec) {
        auto ret = Eigen::VectorXd(vec.rows());

        for (auto i = 0; i < n; i += 2) {
            ret(i + 1) = 20 * t2s[i];
            ret(i) = -2.0 * (vec(i) * ret(i + 1) + t1s[i]);
        }

        return ret;
    }

    int dim() const { return n; }

    ~Rosenbrock() {
        delete[] t1s;
        delete[] t2s;
    }
};

TEST_CASE("LBFGS minimizer") {
    auto minimizer = sgm::LBFGSMinimizer(100, true);

    SECTION("Rosenbrock") {
        auto n = 10;
        auto tol = 1e-10;
        auto fcn = Rosenbrock(n);
        auto initial = Eigen::VectorXd::Zero(n);

        auto param = minimizer.minimize(fcn, initial, tol);
        auto min_val = fcn.value_at(param);

        REQUIRE(param.isApprox(Eigen::VectorXd::Ones(n)));
        REQUIRE(std::abs(min_val - 0.0) < tol);
    }
}
