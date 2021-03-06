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

#ifndef SGMWSMCPP_GRAPHFEATUREEXTRACTOR_H
#define SGMWSMCPP_GRAPHFEATUREEXTRACTOR_H

#include <memory>
#include <parallel_hashmap/phmap.h>

#include "utils/types.h"
#include "utils/container/Counter.h"

namespace sgm
{
template <typename F, typename NodeType>
class GraphFeatureExtractor
{
public:
    using node_type = node_type_base<NodeType>;
    using edge_type = edge_type_base<NodeType>;
    using counter_type = Counter<F>;

    void extract_features(const node_type &node, const edge_type &decision, counter_type &features) const {
        _extract_features(node, decision, features);
    }

    void extract_features(const edge_type &e, counter_type &features) const {
        _extract_features(e, features);
    }

    counter_type default_parameters() const {
        return _default_parameters();
    }

    size_t dim() const {
        return _dim();
    }

    void standardize(const counter_type &mean, const counter_type &sd) {
        _standardize(mean, sd);
    }

    virtual ~GraphFeatureExtractor() = default;

private:
    virtual void _extract_features(const node_type &node, const edge_type &decision,
                                   counter_type &features) const = 0;

    virtual void _extract_features(const edge_type &e, counter_type &features) const = 0;

    virtual counter_type _default_parameters() const = 0;

    virtual size_t _dim() const = 0;

    virtual void _standardize(const counter_type &mean, const counter_type &sd) {}
};
}

#endif //SGMWSMCPP_GRAPHFEATUREEXTRACTOR_H
