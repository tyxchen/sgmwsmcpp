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

#ifndef SGMWSMCPP_STREAMINGPARTICLEFILTER_H
#define SGMWSMCPP_STREAMINGPARTICLEFILTER_H

#include <functional>
#include <memory>
#include <vector>

#include "utils/debug.h"
#include "utils/types.h"
#include "common/graph/GraphMatchingState_fwd.h"
#include "smc/Proposal.h"
#include "smc/StreamingPropagator.h"
#include "smc/PermutationStream.h"
#include "smc/components/GenericMatchingLatentSimulator.h"
#include "smc/components/PruningObservationDensity.h"

namespace sgm
{
namespace smc
{

template <typename F, typename NodeType>
class StreamingBootstrapProposal : public Proposal<GraphMatchingState<F, NodeType>,
                                                   StreamingBootstrapProposal<F, NodeType>>
{
    using latent_type = GraphMatchingState<F, NodeType>;
    using emissions_type = node_type_base<NodeType>;

    int m_seed;
    Random m_random;
    PermutationStream m_permutation_stream;

    emissions_type m_cur_emission;
    emissions_type m_old_emission;
    std::shared_ptr<const std::vector<latent_type>> m_old_latents;

    std::reference_wrapper<GenericMatchingLatentSimulator<F, NodeType>> m_transition_density;
    std::reference_wrapper<PruningObservationDensity<F, NodeType>> m_observation_density;

public:
    StreamingBootstrapProposal(int seed,
                               emissions_type cur_emission,
                               emissions_type old_emission,
                               std::shared_ptr<const std::vector<latent_type>> old_latents,
                               GenericMatchingLatentSimulator<F, NodeType> &transition_density,
                               PruningObservationDensity<F, NodeType> &observation_density)
        : m_seed(seed), m_random(seed * 171),
          m_permutation_stream(old_latents.get() != nullptr ? old_latents->size() : 0, m_random),
          m_cur_emission(std::move(cur_emission)), m_old_emission(std::move(old_emission)),
          m_old_latents(std::move(old_latents)),
          m_transition_density(transition_density), m_observation_density(observation_density) {}

    StreamingBootstrapProposal(int seed, emissions_type cur_emission,
                               GenericMatchingLatentSimulator<F, NodeType> &transition_density,
                               PruningObservationDensity<F, NodeType> &observation_density)
        : StreamingBootstrapProposal(seed, std::move(cur_emission), nullptr, nullptr, transition_density,
            observation_density) {}

private:
    bool is_initial() const {
        return m_old_latents.get() == nullptr;
    }

    const latent_type &sample_old_latent() {
        auto idx = m_permutation_stream.index();
        return (*m_old_latents)[idx];
    }

    std::pair<double, latent_type> _next_log_weight_sample_pair() override {
        auto *old_latent = is_initial() ? nullptr : &sample_old_latent();
        auto cur_latent = is_initial()
                          ? m_transition_density.get().sample_initial(m_random)
                          : m_transition_density.get().sample_forward_transition(m_random, *old_latent);

#ifdef DEBUG
        if (old_latent) {
//            sgm::logger <= "\n" <= *old_latent;
        }
#endif

        auto log_weight = m_observation_density.get().log_density(cur_latent, m_cur_emission);

        if (old_latent != nullptr) {
            auto old_weight = m_observation_density.get().log_density(*old_latent, m_old_emission);
            log_weight -= old_weight;
//            log_weight += m_observation_density.log_weight_correction(cur_latent, old_latent);
        }

        return std::make_pair(log_weight, std::move(cur_latent));
    }

    StreamingBootstrapProposal<F, NodeType> _restart() override {
        return StreamingBootstrapProposal(m_seed, m_cur_emission, m_old_emission, m_old_latents,
                                          m_transition_density, m_observation_density);
    }
};

template <typename F, typename NodeType>
class StreamingParticleFilter
{
    using latent_type = GraphMatchingState<F, NodeType>;
    using emissions_type = node_type_base<NodeType>;

    std::reference_wrapper<GenericMatchingLatentSimulator<F, NodeType>> m_transition_density;
    std::reference_wrapper<PruningObservationDensity<F, NodeType>> m_observation_density;
    std::reference_wrapper<const std::vector<emissions_type>> m_emissions;
    Random m_random { 1 };

    std::pair<CompactPopulation, std::vector<latent_type>> m_results;
    PropagatorOptions m_options;

public:
    StreamingParticleFilter(GenericMatchingLatentSimulator<F, NodeType> &transition_density,
                            PruningObservationDensity<F, NodeType> &observation_density,
                            const std::vector<emissions_type> &emissions,
                            Random random)
        : m_transition_density(transition_density), m_observation_density(observation_density),
          m_emissions(emissions), m_random(random) {}

    PropagatorOptions &options() {
        return m_options;
    }

private:
    StreamingBootstrapProposal<F, NodeType> initial_distribution_proposal() {
        return StreamingBootstrapProposal<F, NodeType>(
            m_random(), m_emissions.get()[0], m_transition_density, m_observation_density
        );
    }

public:
    double sample() {
//        auto proposal = initial_distribution_proposal();
        StreamingPropagator<latent_type, StreamingBootstrapProposal<F, NodeType>> propagator(
            initial_distribution_proposal(),
            m_options
        );

        sgm::logger <= "=== RUN 0 ===\n";

        m_results = propagator.execute();
        auto logZ = m_results.first.logZ_estimate();

        // recursive
        for (auto i = 1; i < m_transition_density.get().iterations(); ++i) {
            StreamingPropagator<latent_type, StreamingBootstrapProposal<F, NodeType>> rec_propagator(
                StreamingBootstrapProposal<F, NodeType>(m_random(), m_emissions.get()[i], m_emissions.get()[i - 1],
                                                        std::shared_ptr<const std::vector<latent_type>>(
                                                            &m_results.second,
                                                            [](auto *) {}
                                                        ),
                                                        m_transition_density, m_observation_density),
                m_options
            );
            sgm::logger <= "=== RUN " <= i <= " ===\n";
            auto results = rec_propagator.execute();

            // FIXME: This is a workaround for when the results give no new matchings
            // If we were to continue, we'd find all the matching states are the same, which doesn't make sense
            if (results.first.log_sum() == -std::numeric_limits<double>::infinity()) {
                break;
            }

            logZ += results.first.logZ_estimate();
            m_results = std::move(results);
        }

        return logZ;
    }

    std::vector<latent_type> &samples() {
        return m_results.second;
    }
};

}
}

#endif //SGMWSMCPP_STREAMINGPARTICLEFILTER_H
