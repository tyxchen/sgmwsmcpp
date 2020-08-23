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

#ifndef SGMWSMCPP_THREEMATCHINGDISTANCEFEATUREEXTRACTOR_H
#define SGMWSMCPP_THREEMATCHINGDISTANCEFEATUREEXTRACTOR_H

#include <cmath>
#include <array>

#include "utils/types.h"
#include "utils/consts.h"
#include "common/model/GraphFeatureExtractor.h"
#include "knot/data/EllipticalKnot.h"
#include "knot/model/features/DistanceFeatureExtractor.h"

namespace sgm
{
namespace ThreeMatchingDistanceFeatureExtractorConsts
{
constexpr int DIM = 4;
static constexpr string_t TWO_MATCHING_DISTANCE_1 = DistanceFeatureExtractorConsts::TWO_MATCHING_DISTANCE_1;
static constexpr string_t TWO_MATCHING_DISTANCE_2 = DistanceFeatureExtractorConsts::TWO_MATCHING_DISTANCE_2;
static constexpr string_t THREE_MATCHING_DISTANCE_1 { "THREE_MATCHING_DISTANCE_1", 301 };
static constexpr string_t THREE_MATCHING_DISTANCE_2 { "THREE_MATCHING_DISTANCE_2", 302 };

constexpr double NORM_CONST = 1; // TODO: Do this in R as a pre-processing step and remove this constant
}

namespace detail
{
template <typename NodeType>
void extract_features_3(const node_type_base<NodeType> &node1,
                        const node_type_base<NodeType> &node2,
                        const node_type_base<NodeType> &node3,
                        Counter<string_t> &features) {
    // 3-matching -- maximal distance
    using ThreeMatchingDistanceFeatureExtractorConsts::NORM_CONST;
    constexpr size_t N = 3;

    auto max_dist = 0.0, min_dist = Consts::POSITIVE_INFINITY;
    std::array<const node_type_base<NodeType> *, N> nodes { &node1, &node2, &node3 };

    for (auto i = 0u; i < N; ++i) {
        for (auto j = i + 1; j < N; ++j) {
            auto ij = compute_distance(**nodes[i], **nodes[j]);
            if (ij > max_dist) {
                max_dist = ij;
            }
            if (ij < min_dist) {
                min_dist = ij;
            }
        }
    }

    features.set(ThreeMatchingDistanceFeatureExtractorConsts::THREE_MATCHING_DISTANCE_1, min_dist / NORM_CONST);
    features.set(ThreeMatchingDistanceFeatureExtractorConsts::THREE_MATCHING_DISTANCE_2, max_dist / NORM_CONST);
}
}

template <typename KnotType>
class ThreeMatchingDistanceFeatureExtractor : public GraphFeatureExtractor<string_t, KnotType>
{
public:
    using base_class = GraphFeatureExtractor<string_t, KnotType>;
    using node_type = typename base_class::node_type;
    using edge_type = typename base_class::edge_type;
    using counter_type = typename base_class::counter_type;

private:
    void _extract_features(const node_type &node, const edge_type &decision, counter_type &features) const override {
        using namespace ThreeMatchingDistanceFeatureExtractorConsts;

        if (decision->size() == 1) {
            DistanceFeatureExtractor<KnotType> fe;
            fe.extract_features(node, decision, features);
            features.set(TWO_MATCHING_DISTANCE_1, features.get(TWO_MATCHING_DISTANCE_1) / NORM_CONST);
            features.set(TWO_MATCHING_DISTANCE_2, features.get(TWO_MATCHING_DISTANCE_2) / NORM_CONST);
        } else if (decision->size() == 2) {
            auto begin = decision->begin();
            detail::extract_features_3<KnotType>(node, *begin, *std::next(begin), features);
        } else {
            features.set(TWO_MATCHING_DISTANCE_1, 10);
        }
    }

    void _extract_features(const edge_type &e, counter_type &features) const override {
        using namespace ThreeMatchingDistanceFeatureExtractorConsts;

        if (e->size() == 2) {
            DistanceFeatureExtractor<KnotType> fe;
            fe.extract_features(e, features);
            features.set(TWO_MATCHING_DISTANCE_1, features.get(TWO_MATCHING_DISTANCE_1) / NORM_CONST);
            features.set(TWO_MATCHING_DISTANCE_2, features.get(TWO_MATCHING_DISTANCE_2) / NORM_CONST);
        } else if (e->size() == 3) {
            auto begin = e->begin();
            detail::extract_features_3<KnotType>(*begin, *std::next(begin), *std::next(begin, 2), features);
        } else {
            features.set(TWO_MATCHING_DISTANCE_1, 10);
        }
    }

    counter_type _default_parameters() const override {
        return counter_type {
            { ThreeMatchingDistanceFeatureExtractorConsts::TWO_MATCHING_DISTANCE_1, 0.0 },
            { ThreeMatchingDistanceFeatureExtractorConsts::TWO_MATCHING_DISTANCE_2, 0.0 },
            { ThreeMatchingDistanceFeatureExtractorConsts::THREE_MATCHING_DISTANCE_1, 0.0 },
            { ThreeMatchingDistanceFeatureExtractorConsts::THREE_MATCHING_DISTANCE_2, 0.0 },
        };
    }

    int _dim() const override {
        return ThreeMatchingDistanceFeatureExtractorConsts::DIM;
    }
};
}
#endif //SGMWSMCPP_THREEMATCHINGDISTANCEFEATUREEXTRACTOR_H
