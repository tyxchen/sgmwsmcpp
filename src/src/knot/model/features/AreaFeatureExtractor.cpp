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
#include <cmath>
#include <iterator>
#include <stdexcept>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

using namespace sgm;

constexpr double PI = 3.141592653589793238462643383279502884;

std::array<double, 2> sgm::compute_area(const EllipticalKnot &knot) {
    Eigen::Matrix2d S;
    Eigen::EigenSolver<Eigen::Matrix2d> es;
    S(0, 0) = knot.node_features().get("var_x");
    S(0, 1) = S(1, 0) = knot.node_features().get("cov");
    S(1, 1) = knot.node_features().get("var_y");
    auto eigen = es.compute(S, true);
    auto lambdas = eigen.eigenvalues().real();

    double area = PI * std::sqrt(lambdas(0)) *
                  AreaFeatureExtractorConsts::SQRT_CRITICAL_VALUE * std::sqrt(lambdas(1)) *
                  AreaFeatureExtractorConsts::SQRT_CRITICAL_VALUE;
    // compute the rotation angle
    double dot = eigen.eigenvectors().real().col(0).dot(Eigen::Vector2d(1.0, 0.0));
    // the eigen vector has length 1 so does e1
    double angle = std::acos(dot);
    return std::array<double, 2> { area, angle };
}

bool sgm::shares_axis(const EllipticalKnot &k1, const EllipticalKnot &k2) {
    double boundary_axes_1[2] = {
        k1.node_features().get("boundary_axis0"),
        k1.node_features().get("boundary_axis1")
    };
    double boundary_axes_2[2] = {
        k2.node_features().get("boundary_axis0"),
        k2.node_features().get("boundary_axis1")
    };

    return (boundary_axes_2[0] != 0 &&
            (boundary_axes_1[0] == boundary_axes_2[0] || boundary_axes_1[1] == boundary_axes_2[0])) ||
           (boundary_axes_2[1] != 0 &&
            (boundary_axes_1[0] == boundary_axes_2[1] || boundary_axes_1[1] == boundary_axes_2[1]));

}

void extract_features_2(const AreaFeatureExtractor::node_type &node1,
                        const AreaFeatureExtractor::node_type &node2,
                        Counter<std::string> &features) {
    // 2-matching -- compare the area and the number of points
    auto area1 = compute_area(*node1);
    auto area2 = compute_area(*node2);

    if (area1[0] == 0.0 || area2[0] == 0.0) {
        throw std::runtime_error("Area is zero: either computation failed or knot detection failed.");
    }

    if ((node1->partition_idx() % 2 == 0 && node2->partition_idx() % 2 == 0) || !shares_axis(*node1, *node2)) {
        features.set(AreaFeatureExtractorConsts::TWO_MATCHING_AREA_DIFF,
                     std::abs(area1[0] - area2[0]) / AreaFeatureExtractorConsts::NORM_CONST);
    }
}

void extract_features_3(const AreaFeatureExtractor::node_type &node1,
                        const AreaFeatureExtractor::node_type &node2,
                        const AreaFeatureExtractor::node_type &node3,
                        Counter<std::string> &features) {
    // 3-matching -- compute area for each of the knots, then compare the two largest knots
    // TODO: this covariate can be improved
    auto largest = -1.0, middle = largest, smallest = largest;
    std::array<const AreaFeatureExtractor::node_type *, 3> nodes { &node1, &node2, &node3 };

    for (auto node : nodes) {
        auto area = compute_area(**node)[0];
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

    features.set(AreaFeatureExtractorConsts::THREE_MATCHING_AREA_DIFF,
                 std::abs(largest - middle) / AreaFeatureExtractorConsts::NORM_CONST);
}

Counter<std::string> AreaFeatureExtractor::_extract_features(const node_type &node, const edge_type &decision) {
    auto f = _default_parameters();

    if (decision->size() == 1) {
        extract_features_2(node, *decision->begin(), f);
    } else if (decision->size() == 2) {
        auto begin = decision->begin();
        extract_features_3(node, *begin, *std::next(begin), f);
    }

    return f;
}

Counter<std::string> AreaFeatureExtractor::_extract_features(const edge_type &e) {
    auto f = _default_parameters();

    if (e->size() == 2) {
        auto begin = e->begin();
        extract_features_2(*begin, *std::next(begin), f);
    } else if (e->size() == 3) {
        auto begin = e->begin();
        extract_features_3(*begin, *std::next(begin), *std::next(begin, 2), f);
    }

    return f;
}

Counter<std::string> AreaFeatureExtractor::_default_parameters() const {
    return Counter<std::string> {
        { AreaFeatureExtractorConsts::TWO_MATCHING_AREA_DIFF, 0 },
        { AreaFeatureExtractorConsts::THREE_MATCHING_AREA_DIFF, 0 },
    };
}

inline int AreaFeatureExtractor::_dim() const {
    return AreaFeatureExtractorConsts::DIM;
}
