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

#include "EllipticalKnotFeatureExtractor.h"

using namespace sgm;

void EllipticalKnotFeatureExtractor::_extract_features(const node_type &node, const edge_type &decision,
                                                       counter_type &features) const {
    m_distance_fe.extract_features(node, decision, features);
    m_area_fe.extract_features(node, decision, features);

    for (const auto &feature : features) {
        const auto &feature_name = feature.first;
        features.set(feature_name, (feature.second - m_mean.get(feature_name)) / m_sd.get(feature_name, 1.0));
    }
}

void EllipticalKnotFeatureExtractor::_extract_features(const edge_type &e, counter_type &features) const {
    m_distance_fe.extract_features(e, features);
    m_area_fe.extract_features(e, features);
}

EllipticalKnotFeatureExtractor::counter_type EllipticalKnotFeatureExtractor::_default_parameters() const {
    auto params = m_distance_fe.default_parameters();

    for (auto &param : m_area_fe.default_parameters()) {
        params.set(param.first, param.second);
    }

    return params;
}

int EllipticalKnotFeatureExtractor::_dim() const {
    return m_distance_fe.dim() + m_area_fe.dim();
}

void EllipticalKnotFeatureExtractor::_standardize(counter_type mean, counter_type sd) {
    m_mean = mean;
    m_sd = sd;
}

const ThreeMatchingDistanceFeatureExtractor<EllipticalKnot> &EllipticalKnotFeatureExtractor::distance_fe() const {
    return m_distance_fe;
}

const AreaFeatureExtractor &EllipticalKnotFeatureExtractor::area_fe() const {
    return m_area_fe;
}

const EllipticalKnotFeatureExtractor::counter_type &EllipticalKnotFeatureExtractor::mean() const {
    return m_mean;
}

const EllipticalKnotFeatureExtractor::counter_type &EllipticalKnotFeatureExtractor::sd() const {
    return m_sd;
}

