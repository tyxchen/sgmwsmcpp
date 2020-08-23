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

#include "knot/model/features/AreaFeatureExtractor.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include "utils/types.h"
#include "utils/consts.h"
#include "utils/debug.h"

using namespace sgm;

const Eigen::Vector2d e1(1.0, 0.0);

std::pair<double, double> sgm::compute_area(const EllipticalKnot &knot) {
    Eigen::Matrix2d S;
    Eigen::EigenSolver<Eigen::Matrix2d> es;
    S(0, 0) = knot.node_features().get(EllipticalKnotFeatureNames::VAR_X);
    S(0, 1) = S(1, 0) = knot.node_features().get(EllipticalKnotFeatureNames::COV);
    S(1, 1) = knot.node_features().get(EllipticalKnotFeatureNames::VAR_Y);
    auto eigen = es.compute(S, true);
    auto lambdas = eigen.eigenvalues().real();

    double area = Consts::PI * std::sqrt(lambdas(0)) *
                  AreaFeatureExtractorConsts::SQRT_CRITICAL_VALUE * std::sqrt(lambdas(1)) *
                  AreaFeatureExtractorConsts::SQRT_CRITICAL_VALUE;
    // compute the rotation angle
    double dot = eigen.eigenvectors().real().col(0).dot(e1);
    // the eigen vector has length 1 so does e1
    double angle = std::acos(dot);
    return std::make_pair(area, angle);
}

bool sgm::shares_axis(const EllipticalKnot &k1, const EllipticalKnot &k2) {
    double boundary_axes_1[2] = {
        k1.node_features().get(EllipticalKnotFeatureNames::BOUNDARY_AXIS0),
        k1.node_features().get(EllipticalKnotFeatureNames::BOUNDARY_AXIS1)
    };
    double boundary_axes_2[2] = {
        k2.node_features().get(EllipticalKnotFeatureNames::BOUNDARY_AXIS0),
        k2.node_features().get(EllipticalKnotFeatureNames::BOUNDARY_AXIS1)
    };

    return (boundary_axes_2[0] != 0 &&
            (boundary_axes_1[0] == boundary_axes_2[0] || boundary_axes_1[1] == boundary_axes_2[0])) ||
           (boundary_axes_2[1] != 0 &&
            (boundary_axes_1[0] == boundary_axes_2[1] || boundary_axes_1[1] == boundary_axes_2[1]));

}

void extract_features_2(const AreaFeatureExtractor::node_type &node1,
                        const AreaFeatureExtractor::node_type &node2,
                        Counter<string_t> &features) {
    // 2-matching -- compare the area and the number of points
    auto area1 = compute_area(*node1);
    auto area2 = compute_area(*node2);

    if (area1.first == 0.0 || area2.first == 0.0) {
        throw sgm::runtime_error("Area is zero: either computation failed or knot detection failed.");
    }

    if ((node1->pidx() % 2 == 0 && node2->pidx() % 2 == 0) || !shares_axis(*node1, *node2)) {
        features.set(AreaFeatureExtractorConsts::TWO_MATCHING_AREA_DIFF,
                     std::abs(area1.first - area2.first) / AreaFeatureExtractorConsts::NORM_CONST);
    }
}

std::tuple<double, double, double> extract_features_3_base(const AreaFeatureExtractor::node_type &node1,
                                                           const AreaFeatureExtractor::node_type &node2,
                                                           const AreaFeatureExtractor::node_type &node3) {
    // 3-matching -- compute area for each of the knots, then compare the two largest knots
    // TODO: this covariate can be improved
    auto largest = -1.0, middle = largest, smallest = largest;
    AreaFeatureExtractor::node_type const * nodes[3] = { &node1, &node2, &node3 };

    for (auto node : nodes) {
        auto area = compute_area(**node).first;
        if (area > largest) {
            smallest = middle;
            middle = largest;
            largest = area;
        } else if (middle < area) {
            smallest = middle;
            middle = area;
        } else {
            smallest = area;
        }
    }

    return std::make_tuple(smallest, middle, largest);
}

void extract_features_3(const AreaFeatureExtractor::node_type &node1,
                        const AreaFeatureExtractor::node_type &node2,
                        const AreaFeatureExtractor::node_type &node3,
                        AreaFeatureExtractor::counter_type &features) {
    auto areas = extract_features_3_base(node1, node2, node3);
    features.set(AreaFeatureExtractorConsts::THREE_MATCHING_AREA_DIFF,
                 std::abs(std::get<2>(areas) - std::get<1>(areas)) / AreaFeatureExtractorConsts::NORM_CONST);
}

void extract_features_3_legacy(const AreaFeatureExtractor::node_type &node1,
                               const AreaFeatureExtractor::node_type &node2,
                               const AreaFeatureExtractor::node_type &node3,
                               AreaFeatureExtractor::counter_type &features) {
    auto areas = extract_features_3_base(node1, node2, node3);
    features.set(AreaFeatureExtractorConsts::THREE_MATCHING_AREA_DIFF,
                 std::abs(std::get<0>(areas) + std::get<1>(areas) - std::get<2>(areas)) /
                     AreaFeatureExtractorConsts::NORM_CONST);
}

void AreaFeatureExtractor::_extract_features(const node_type &node, const edge_type &decision,
                                             counter_type &features) const {
    if (decision->size() == 1) {
        extract_features_2(node, *decision->begin(), features);
    } else if (decision->size() == 2) {
        auto begin = decision->begin();
        extract_features_3(node, *begin, *std::next(begin), features);
    }
}

void AreaFeatureExtractor::_extract_features(const edge_type &e, counter_type &features) const {
    if (e->size() == 2) {
        auto begin = e->begin();
        extract_features_2(*begin, *std::next(begin), features);
    } else if (e->size() == 3) {
        auto begin = e->begin();
        extract_features_3_legacy(*begin, *std::next(begin), *std::next(begin, 2), features);
    }
}

AreaFeatureExtractor::counter_type AreaFeatureExtractor::_default_parameters() const {
    return counter_type {
        { AreaFeatureExtractorConsts::TWO_MATCHING_AREA_DIFF, 0 },
        { AreaFeatureExtractorConsts::THREE_MATCHING_AREA_DIFF, 0 },
    };
}

int AreaFeatureExtractor::_dim() const {
    return AreaFeatureExtractorConsts::DIM;
}
