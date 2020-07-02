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
#include <string>
#include <parallel_hashmap/phmap.h>

#include "utils/container/Counter.h"
#include "knot/data/EllipticalKnot.h"
#include "common/model/GraphFeatureExtractor.h"
#include "knot/model/features/ThreeMatchingDistanceFeatureExtractor.h"
#include "knot/model/features/AreaFeatureExtractor.h"

namespace sgm
{
class EllipticalKnotFeatureExtractor: public GraphFeatureExtractor<std::string, EllipticalKnot>
{
    ThreeMatchingDistanceFeatureExtractor<EllipticalKnot> m_distance_fe;
    AreaFeatureExtractor m_area_fe;

    Counter<std::string> m_mean;
    Counter<std::string> m_sd;

public:
    Counter<std::string> _extract_features(const node_type &node, const phmap::flat_hash_set<node_type> &decision) override;

    Counter<std::string> _extract_features(const phmap::flat_hash_set<node_type> &e) override;

    Counter<std::string> _default_parameters() const override;

    int _dim() const override;

    void _standardize(const Counter<std::string> &mean, const Counter<std::string> &sd) override;
};
}

#endif //SGMWSMCPP_ELLIPTICALKNOTFEATUREEXTRACTOR_H
