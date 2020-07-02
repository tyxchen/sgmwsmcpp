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

#include "utils/container/Counter.h"

namespace sgm
{
template <typename F, typename NodeType>
class GraphFeatureExtractor
{
public:
    using node_type = typename std::shared_ptr<NodeType>;

    Counter<F> extract_features(const node_type &node, const phmap::flat_hash_set<node_type> &decision) {
        return _extract_features(node, decision);
    }

    Counter<F> extract_features(const phmap::flat_hash_set<node_type> &e) {
        return _extract_features(e);
    }

    Counter<F> default_parameters() const {
        return _default_parameters();
    }

    inline int dim() const {
        return _dim();
    }

    void standardize(const Counter<F> &mean, const Counter<F> &sd) {
        _standardize(mean, sd);
    }

protected:
    virtual Counter<F> _extract_features(const node_type &node,
                                         const phmap::flat_hash_set<node_type> &decision) = 0;

    virtual Counter<F> _extract_features(const phmap::flat_hash_set<node_type> &e) = 0;

    virtual Counter<F> _default_parameters() const = 0;

    virtual inline int _dim() const = 0;

    virtual void _standardize(const Counter<F> &mean, const Counter<F> &sd) {}
};
}

#endif //SGMWSMCPP_GRAPHFEATUREEXTRACTOR_H
