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
#include <limits>
#include <memory>
#include <vector>

#include "utils/types.h"
#include "utils/debug.h"
#include "utils/Random.h"
#include "smc/components/GenericMatchingLatentSimulator.h"
#include "smc/ObservationDensity.h"
#include "smc/StreamingParticleFilter.h"

namespace sgm
{
namespace smc
{

template <typename F, typename NodeType>
class SequentialGraphMatchingSampler
{
    std::reference_wrapper<GenericMatchingLatentSimulator<F, NodeType>> m_transition_density;
    std::reference_wrapper<ObservationDensity<F, NodeType>> m_observation_density;
    std::reference_wrapper<const std::vector<node_type_base<NodeType>>> m_emissions;
    std::vector<GraphMatchingState<F, NodeType>> m_samples;

    bool m_use_SPF = true;

public:
    SequentialGraphMatchingSampler(GenericMatchingLatentSimulator<F, NodeType> &transition_density,
                                   ObservationDensity<F, NodeType> &observation_density,
                                   const std::vector<node_type_base<NodeType>> &emissions,
                                   bool use_SPF)
        : m_transition_density(transition_density),
          m_observation_density(observation_density),
          m_emissions(emissions),
          m_use_SPF(use_SPF) {}

    double sample(Random &random, int num_concrete_particles, int max_virtual_particles) {
        auto logZ = -std::numeric_limits<double>::infinity();
        if (m_use_SPF) {
            StreamingParticleFilter<F, NodeType> spf(m_transition_density, m_observation_density, m_emissions,
                                                     Random(random()));
            auto &options = spf.options();
            options.num_concrete_particles = num_concrete_particles;
            options.max_virtual_particles = max_virtual_particles;
            options.verbose = false;
            options.targeted_relative_ess = 1.0;

            logZ = spf.sample();
            m_samples = std::move(spf.samples());
        } else {
            throw sgm::runtime_error("Using non-streaming particle filter SMC is not yet supported.");
        }
        return logZ;
    }

    std::vector<GraphMatchingState<F, NodeType>> &samples() {
        return m_samples;
    }
};

}
}

#endif //SGMWSMCPP_SEQUENTIALGRAPHMATCHINGSAMPLER_H
