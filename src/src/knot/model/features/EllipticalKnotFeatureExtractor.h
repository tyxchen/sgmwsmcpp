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

#ifndef SGMWSMCPP_ELLIPTICALKNOTFEATUREEXTRACTOR_H
#define SGMWSMCPP_ELLIPTICALKNOTFEATUREEXTRACTOR_H

#include <memory>
#include <parallel_hashmap/phmap.h>

#include "utils/types.h"
#include "utils/container/Counter.h"
#include "knot/data/EllipticalKnot.h"
#include "common/model/GraphFeatureExtractor.h"
#include "knot/model/features/ThreeMatchingDistanceFeatureExtractor.h"
#include "knot/model/features/AreaFeatureExtractor.h"

namespace sgm
{

namespace EllipticalKnotFeatureExtractorConsts
{
static constexpr string_t TWO_MATCHING_DISTANCE_1 =
    ThreeMatchingDistanceFeatureExtractorConsts::TWO_MATCHING_DISTANCE_1;
static constexpr string_t TWO_MATCHING_DISTANCE_2 =
    ThreeMatchingDistanceFeatureExtractorConsts::TWO_MATCHING_DISTANCE_2;
static constexpr string_t TWO_MATCHING_AREA_DIFF =
    AreaFeatureExtractorConsts::TWO_MATCHING_AREA_DIFF;
static constexpr string_t THREE_MATCHING_DISTANCE_1 =
    ThreeMatchingDistanceFeatureExtractorConsts::THREE_MATCHING_DISTANCE_1;
static constexpr string_t THREE_MATCHING_DISTANCE_2 =
    ThreeMatchingDistanceFeatureExtractorConsts::THREE_MATCHING_DISTANCE_2;
static constexpr string_t THREE_MATCHING_AREA_DIFF =
    AreaFeatureExtractorConsts::THREE_MATCHING_AREA_DIFF;
}

class EllipticalKnotFeatureExtractor: public GraphFeatureExtractor<string_t, EllipticalKnot>
{
    ThreeMatchingDistanceFeatureExtractor<EllipticalKnot> m_distance_fe;
    AreaFeatureExtractor m_area_fe;

    counter_type m_mean;
    counter_type m_sd;

    void _extract_features(const node_type &node, const edge_type &decision, counter_type &features) const override;

    void _extract_features(const edge_type &e, counter_type &features) const override;

    counter_type _default_parameters() const override;

    int _dim() const override;

    void _standardize(const counter_type &mean, const counter_type &sd) override;

public:
    const ThreeMatchingDistanceFeatureExtractor<EllipticalKnot> &distance_fe() const;

    const AreaFeatureExtractor &area_fe() const;

    const counter_type &mean() const;

    const counter_type &sd() const;
};
}

#endif //SGMWSMCPP_ELLIPTICALKNOTFEATUREEXTRACTOR_H
