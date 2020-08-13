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

#ifndef SGMWSMCPP_AREAFEATUREEXTRACTOR_H
#define SGMWSMCPP_AREAFEATUREEXTRACTOR_H

#include <array>
#include <memory>
#include <string>
#include <boost/math/distributions/chi_squared.hpp>
#include <parallel_hashmap/phmap.h>

#include "common/model/GraphFeatureExtractor.h"
#include "knot/data/EllipticalKnot.h"

namespace sgm
{
namespace AreaFeatureExtractorConsts
{
constexpr int DIM = 2;
constexpr const char* TWO_MATCHING_AREA_DIFF = "TWO_MATCHING_AREA_DIFF";
constexpr const char* THREE_MATCHING_AREA_DIFF = "THREE_MATCHING_AREA_DIFF";
constexpr double NORM_CONST = 1;
constexpr double NORM_CONST2 = 500;
constexpr double CONFIDENCE_LEVEL = 0.975;
const double SQRT_CRITICAL_VALUE = []() noexcept -> double {
    static_assert(0 <= CONFIDENCE_LEVEL && CONFIDENCE_LEVEL < 1, "Must choose confidence level in [0,1)");
    boost::math::chi_squared_distribution<double> chi(2);
    return std::sqrt(boost::math::quantile(chi, CONFIDENCE_LEVEL));
}();
}

std::pair<double, double> compute_area(const EllipticalKnot &knot);

bool shares_axis(const EllipticalKnot &k1, const EllipticalKnot &k2);

class AreaFeatureExtractor : public GraphFeatureExtractor<std::string, EllipticalKnot>
{
    Counter<std::string> _extract_features(const node_type &node, const edge_type &decision) override;

    Counter<std::string> _extract_features(const edge_type &e) override;

    Counter<std::string> _default_parameters() const override;

    int _dim() const override;

};
}

#endif //SGMWSMCPP_AREAFEATUREEXTRACTOR_H
