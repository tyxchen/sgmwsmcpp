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

#ifndef SGMWSMCPP_SEQUENTIALGRAPHMATCHINGSAMPLER_H
#define SGMWSMCPP_SEQUENTIALGRAPHMATCHINGSAMPLER_H

#include <functional>
#include <memory>
#include <vector>

#include "utils/types.h"
#include "utils/consts.h"
#include "utils/debug.h"
#include "utils/Random.h"
#include "smc/components/GenericMatchingLatentSimulator.h"
#include "smc/CompactPopulation.h"
#include "smc/ObservationDensity.h"
#include "smc/StreamingParticleFilter.h"

namespace sgm
{
namespace smc
{

template <typename F, typename NodeType>
double sample(const GenericMatchingLatentSimulator<F, NodeType> &transition_density,
              const ObservationDensity<F, NodeType> &observation_density,
              const std::vector<node_type_base<NodeType>> &emissions,
              Random::seed_type seed, int num_concrete_particles, int max_virtual_particles, bool use_SPF,
              std::vector<std::shared_ptr<GraphMatchingState<F, NodeType>>> &samples) {
    auto logZ = Consts::NEGATIVE_INFINITY;
    if (use_SPF) {
        StreamingParticleFilter<F, NodeType> spf(transition_density, observation_density, emissions, seed);
        auto &options = spf.options();
        options.num_concrete_particles = num_concrete_particles;
        options.max_virtual_particles = max_virtual_particles;
        options.verbose = false;
        options.targeted_relative_ess = 1.0;

        logZ = spf.sample(samples);
    } else {
        throw sgm::runtime_error("Using non-streaming particle filter SMC is not yet supported.");
    }
    return logZ;
}

}
}

#endif //SGMWSMCPP_SEQUENTIALGRAPHMATCHINGSAMPLER_H
