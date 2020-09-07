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
#include <boost/math/distributions/chi_squared.hpp>
#include <parallel_hashmap/phmap.h>

#include "utils/types.h"
#include "common/model/GraphFeatureExtractor.h"
#include "knot/data/EllipticalKnot.h"

namespace sgm
{
namespace AreaFeatureExtractorConsts
{
static constexpr size_t DIM = 2;
static constexpr string_t TWO_MATCHING_AREA_DIFF { "TWO_MATCHING_AREA_DIFF", 211 };
static constexpr string_t THREE_MATCHING_AREA_DIFF { "THREE_MATCHING_AREA_DIFF", 311 };
static constexpr double NORM_CONST = 1;
//constexpr double NORM_CONST2 = 500;
static constexpr double CONFIDENCE_LEVEL = 0.975;
// constexpr double SQRT_CRITICAL_VALUE = 2.7162030314820957;
static const double SQRT_CRITICAL_VALUE = []() noexcept -> double {
    static_assert(0 <= CONFIDENCE_LEVEL && CONFIDENCE_LEVEL < 1, "Must choose confidence level in [0,1)");
    try {
        boost::math::chi_squared_distribution<double> chi(2);
        return std::sqrt(boost::math::quantile(chi, CONFIDENCE_LEVEL));
    } catch (...) {
        return 0;
    }
}();
}

std::pair<double, double> compute_area(const EllipticalKnot &knot);

bool shares_axis(const EllipticalKnot &k1, const EllipticalKnot &k2);

class AreaFeatureExtractor : public GraphFeatureExtractor<string_t, EllipticalKnot>
{
    void _extract_features(const node_type &node, const edge_type &decision, counter_type &features) const override;

    void _extract_features(const edge_type &e, counter_type &features) const override;

    counter_type _default_parameters() const override;

    size_t _dim() const override;

};
}

#endif //SGMWSMCPP_AREAFEATUREEXTRACTOR_H
