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
#include <limits>
#include <string>

#include "common/model/GraphFeatureExtractor.h"
#include "knot/data/EllipticalKnot.h"
#include "knot/model/features/DistanceFeatureExtractor.h"

namespace sgm
{

namespace ThreeMatchingDistanceFeatureExtractorConsts
{
constexpr int DIM = 4;
const char* const TWO_MATCHING_DISTANCE_1 = "TWO_MATCHING_DISTANCE_1";
const char* const TWO_MATCHING_DISTANCE_2 = "TWO_MATCHING_DISTANCE_2";
const char* const THREE_MATCHING_DISTANCE_1 = "THREE_MATCHING_DISTANCE_1";
const char* const THREE_MATCHING_DISTANCE_2 = "THREE_MATCHING_DISTANCE_2";

constexpr double NORM_CONST = 1; // TODO: Do this in R as a pre-processing step and remove this constant
}


namespace
{
template <typename NodeType>
using node_type = typename GraphFeatureExtractor<std::string, NodeType>::node_type;

template <typename NodeType>
double compute_distance(const NodeType &node, const NodeType &other) {
    auto node_f = node.node_features();
    auto other_f = other.node_features();

    auto diffX = std::pow(node_f.get("x") - other_f.get("x"), 2);
    auto diffY = std::pow(node_f.get("y") - other_f.get("y"), 2);
    auto diffZ = std::pow(node_f.get("z") - other_f.get("z"), 2);

    return std::sqrt(diffX + diffY + diffZ);
}

template <typename NodeType>
void extract_features_3(const node_type<NodeType> &node1,
                        const node_type<NodeType> &node2,
                        const node_type<NodeType> &node3,
                        Counter<std::string> &features) {
    // 3-matching -- maximal distance
    using ThreeMatchingDistanceFeatureExtractorConsts::NORM_CONST;
    constexpr size_t N = 3;

    auto max_dist = 0.0, min_dist = std::numeric_limits<double>::infinity();
    std::array<const node_type<NodeType> *, N> nodes { &node1, &node2, &node3 };

    for (auto i = 0; i < N; ++i) {
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
class ThreeMatchingDistanceFeatureExtractor : public GraphFeatureExtractor<std::string, KnotType>
{
public:
    using node_type = typename GraphFeatureExtractor<std::string, KnotType>::node_type;

protected:
    Counter<std::string> _extract_features(const node_type &node,
                                           const phmap::flat_hash_set<node_type> &decision) override {
        using ThreeMatchingDistanceFeatureExtractorConsts::NORM_CONST;
        auto f = _default_parameters();

        if (decision.size() == 1) {
            DistanceFeatureExtractor<KnotType> fe;
            auto fe_features = fe.extract_features(node, decision);
            f.set(ThreeMatchingDistanceFeatureExtractorConsts::TWO_MATCHING_DISTANCE_1,
                  fe_features.get(DistanceFeatureExtractorConsts::TWO_MATCHING_DISTANCE_1) / NORM_CONST);
            f.set(ThreeMatchingDistanceFeatureExtractorConsts::TWO_MATCHING_DISTANCE_2,
                  fe_features.get(DistanceFeatureExtractorConsts::TWO_MATCHING_DISTANCE_2) / NORM_CONST);
        } else if (decision.size() == 2) {
            auto begin = decision.begin();
            extract_features_3<KnotType>(node, *begin, *std::next(begin), f);
        }

        return f;
    }

    Counter<std::string> _extract_features(const phmap::flat_hash_set<node_type> &e) override {
        using ThreeMatchingDistanceFeatureExtractorConsts::NORM_CONST;
        auto f = _default_parameters();

        if (e.size() == 2) {
            DistanceFeatureExtractor<KnotType> fe;
            auto fe_features = fe.extract_features(e);
            f.set(ThreeMatchingDistanceFeatureExtractorConsts::TWO_MATCHING_DISTANCE_1,
                  fe_features.get(DistanceFeatureExtractorConsts::TWO_MATCHING_DISTANCE_1) / NORM_CONST);
            f.set(ThreeMatchingDistanceFeatureExtractorConsts::TWO_MATCHING_DISTANCE_2,
                  fe_features.get(DistanceFeatureExtractorConsts::TWO_MATCHING_DISTANCE_2) / NORM_CONST);
        } else if (e.size() == 3) {
            auto begin = e.begin();
            extract_features_3<KnotType>(*begin, *std::next(begin), *std::next(begin, 2), f);
        }

        return f;
    }

    Counter<std::string> _default_parameters() const override {
        return Counter<std::string> {
            { ThreeMatchingDistanceFeatureExtractorConsts::TWO_MATCHING_DISTANCE_1, 0.0 },
            { ThreeMatchingDistanceFeatureExtractorConsts::TWO_MATCHING_DISTANCE_2, 0.0 },
            { ThreeMatchingDistanceFeatureExtractorConsts::THREE_MATCHING_DISTANCE_1, 0.0 },
            { ThreeMatchingDistanceFeatureExtractorConsts::THREE_MATCHING_DISTANCE_2, 0.0 },
        };
    }

    inline int _dim() const override {
        return ThreeMatchingDistanceFeatureExtractorConsts::DIM;
    }
};
}
#endif //SGMWSMCPP_THREEMATCHINGDISTANCEFEATUREEXTRACTOR_H