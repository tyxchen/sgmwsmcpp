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

#ifndef SGMWSMCPP_DISTANCEFEATUREEXTRACTOR_H
#define SGMWSMCPP_DISTANCEFEATUREEXTRACTOR_H

#include <array>
#include <cmath>
#include <iterator>
#include <stdexcept>

#include "utils/types.h"
#include "common/model/GraphFeatureExtractor.h"
#include "knot/data/EllipticalKnot.h"

namespace sgm
{

namespace DistanceFeatureExtractorConsts
{
constexpr int DIM = 2;
static constexpr string_t TWO_MATCHING_DISTANCE_1 { "TWO_MATCHING_DISTANCE_1", 201 };
static constexpr string_t TWO_MATCHING_DISTANCE_2 { "TWO_MATCHING_DISTANCE_2", 202 };
}

template <typename NodeType>
double compute_distance(const NodeType &node, const NodeType &other) {
    auto diffX = std::pow(node.x() - other.x(), 2);
    auto diffY = std::pow(node.y() - other.y(), 2);
    auto diffZ = std::pow(node.z() - other.z(), 2);

    return std::sqrt(diffX + diffY + diffZ);
}

template <typename KnotType>
void compute_distance(const KnotType &node, const KnotType &other, Counter<string_t> &features) {
    auto ind1 = node.pidx() % 2;
    auto ind2 = other.pidx() % 2;

    auto diffX = std::pow(node.x() - other.x(), 2);
    auto diffY = std::pow(node.y() - other.y(), 2);
    auto diffZ = std::pow(node.z() - other.z(), 2);

    if (ind1 == 0 && ind1 == ind2) {
        // both knots are on a wide surface
        features.set(DistanceFeatureExtractorConsts::TWO_MATCHING_DISTANCE_1, std::sqrt(diffX + diffY + diffZ));
    } else {
        // one knot is on a narrow surface
        features.set(DistanceFeatureExtractorConsts::TWO_MATCHING_DISTANCE_2, std::sqrt(diffX + diffY + diffZ));
    }
}

template <typename KnotType>
class DistanceFeatureExtractor : public GraphFeatureExtractor<string_t, KnotType>
{
public:
    using base_class = GraphFeatureExtractor<string_t, KnotType>;
    using node_type = typename base_class::node_type;
    using edge_type = typename base_class::edge_type;
    using counter_type = typename base_class::counter_type;

private:
    void _extract_features(const node_type &node, const edge_type &decision, counter_type &features) const override {
        if (decision->size() != 1) {
            throw std::runtime_error("Expected two-matching, received " + std::to_string(decision->size() - 1) +
                                     "-matching.");
        }

        compute_distance(*node, **decision->begin(), features);
    }

    void _extract_features(const edge_type &e, counter_type &features) const override {
        if (e->size() != 2) {
            throw std::runtime_error("Expected two-matching, received " + std::to_string(e->size()) + "-matching.");
        }

        auto begin = e->begin();
        compute_distance(**begin, **std::next(begin), features);
    }

    counter_type _default_parameters() const override {
        return counter_type {
            { DistanceFeatureExtractorConsts::TWO_MATCHING_DISTANCE_1, 0.0 },
            { DistanceFeatureExtractorConsts::TWO_MATCHING_DISTANCE_2, 0.0 },
        };
    }

    int _dim() const override {
        return DistanceFeatureExtractorConsts::DIM;
    }
};
}

#endif //SGMWSMCPP_DISTANCEFEATUREEXTRACTOR_H
