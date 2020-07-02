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

Counter<std::string> EllipticalKnotFeatureExtractor::_extract_features(const node_type &node,
                                                                       const phmap::flat_hash_set<node_type>
                                                                       &decision) {
    Counter<std::string> f;

    auto distance_features = m_distance_fe.extract_features(node, decision);
    auto area_features = m_area_fe.extract_features(node, decision);

    for (const auto &feature : distance_features) {
        const auto &feature_name = feature.first;
        f.set(feature_name,
              (distance_features.get(feature_name) - m_mean.get(feature_name)) / m_sd.get(feature_name, 1.0));
    }

    for (const auto &feature : area_features) {
        const auto &feature_name = feature.first;
        f.set(feature_name,
              (area_features.get(feature_name) - m_mean.get(feature_name)) / m_sd.get(feature_name, 1.0));
    }

    return f;
}

Counter<std::string> EllipticalKnotFeatureExtractor::_extract_features(const phmap::flat_hash_set<node_type> &e) {
    auto f = m_distance_fe.extract_features(e);

    f.increment_all(m_area_fe.extract_features(e));

    return f;
}

Counter<std::string> EllipticalKnotFeatureExtractor::_default_parameters() const {
    auto p = m_distance_fe.default_parameters();

    p.increment_all(m_area_fe.default_parameters());

    return p;
}

int EllipticalKnotFeatureExtractor::_dim() const {
    return m_distance_fe.dim() + m_area_fe.dim();
}

void EllipticalKnotFeatureExtractor::_standardize(Counter<std::string> mean, Counter<std::string> sd) {
    m_mean = std::move(mean);
    m_sd = std::move(sd);
}

const ThreeMatchingDistanceFeatureExtractor<EllipticalKnot> &EllipticalKnotFeatureExtractor::distance_fe() const {
    return m_distance_fe;
}

const AreaFeatureExtractor &EllipticalKnotFeatureExtractor::area_fe() const {
    return m_area_fe;
}

const Counter<std::string> &EllipticalKnotFeatureExtractor::mean() const {
    return m_mean;
}

const Counter<std::string> &EllipticalKnotFeatureExtractor::sd() const {
    return m_sd;
}

