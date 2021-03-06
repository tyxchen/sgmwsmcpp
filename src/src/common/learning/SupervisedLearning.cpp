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

#include "common/learning/SupervisedLearning.h"

#include <cmath>

using namespace sgm;

bool SupervisedLearningConfig::parallel = true;
int SupervisedLearningConfig::num_lbfgs_iters = 100;

bool SupervisedLearning::check_convergence(double old_nllk, double new_nllk, double tolerance) {
    return std::abs(old_nllk - new_nllk) < tolerance;
}
