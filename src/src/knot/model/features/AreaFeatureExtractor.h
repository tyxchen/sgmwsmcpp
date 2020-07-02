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
struct AreaFeatureExtractorConsts
{
    static constexpr int DIM = 2;
    static const char *TWO_MATCHING_AREA_DIFF;
    static const char *THREE_MATCHING_AREA_DIFF;
    static constexpr double NORM_CONST = 1;
    static constexpr double NORM_CONST2 = 500;
    static constexpr double CONFIDENCE_LEVEL = 0.975;
    static double SQRT_CRITICAL_VALUE;

    static struct _init
    {
        _init() noexcept {
            static_assert(0 <= CONFIDENCE_LEVEL && CONFIDENCE_LEVEL < 1, "Must choose confidence level in [0,1)");
            boost::math::chi_squared_distribution<double> chi(2);
            SQRT_CRITICAL_VALUE = boost::math::quantile(chi, CONFIDENCE_LEVEL);
        }
    } _initializer;
};

std::array<double, 2> compute_area(const EllipticalKnot &knot);

bool shares_axis(const EllipticalKnot &k1, const EllipticalKnot &k2);

class AreaFeatureExtractor : public GraphFeatureExtractor<std::string, EllipticalKnot>
{
protected:
    Counter<std::string> _extract_features(const node_type &node,
                                           const phmap::flat_hash_set<node_type> &decision) override;

    Counter<std::string> _extract_features(const phmap::flat_hash_set<node_type> &e) override;

    Counter<std::string> _default_parameters() const override;

    inline int _dim() const override;

};
}

#endif //SGMWSMCPP_AREAFEATUREEXTRACTOR_H
