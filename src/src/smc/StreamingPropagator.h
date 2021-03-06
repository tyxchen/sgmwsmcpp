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

#ifndef SGMWSMCPP_STREAMINGPROPAGATOR_H
#define SGMWSMCPP_STREAMINGPROPAGATOR_H

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "utils/debug.h"
#include "utils/Random.h"
#include "smc/CompactPopulation.h"
#include "smc/Proposal.h"

namespace sgm
{
namespace smc
{

struct PropagatorOptions
{
    static constexpr size_t DEFAULT_NUM_CONCRETE_PARTICLES = 1000;
    bool verbose = true;
    size_t num_concrete_particles = DEFAULT_NUM_CONCRETE_PARTICLES;
    size_t max_virtual_particles = 100000;
    double targeted_relative_ess = 0.5;
    Random resampling_random { 1 };
};

template <typename S, typename ProposalType>
class StreamingPropagator
{
    ProposalType m_proposal;
    PropagatorOptions m_options;

public:
    StreamingPropagator(ProposalType proposal, PropagatorOptions options)
        : m_proposal(std::move(proposal)), m_options(options) {}

    explicit StreamingPropagator(ProposalType proposal)
        : StreamingPropagator(std::move(proposal), PropagatorOptions()) {}

private:
    void propose(CompactPopulation &pop) {
        while (pop.num_particles() < m_options.num_concrete_particles ||
               (pop.num_particles() < m_options.max_virtual_particles &&
                pop.ess() / m_options.num_concrete_particles < m_options.targeted_relative_ess)) {
            auto weight = m_proposal.next_log_weight();
            pop.insert_log_weight(weight);
#ifdef NDEBUG
            if (m_options.verbose) {
#endif
//                sgm::logger << pop.num_particles() << ": " << pop.log_sum() << ", " << pop.log_sum_of_squares() << ", "
//                            << pop.ess() << ", " << weight << '\n';
#ifdef NDEBUG
            }
#endif
        }
#ifndef NDEBUG
        sgm::logger <= "proposed: " <= pop.num_particles() <= std::endl;
#endif
    }

    std::vector<double> extract_sorted_cumulative_probabilites() {
        std::vector<double> res(m_options.num_concrete_particles);
        auto &rng = m_options.resampling_random;
        auto spacing = 1.0 / m_options.num_concrete_particles;
        auto i = 0;

        for (auto &dart : res) {
            dart = spacing * i + spacing * rng.next_double();
            ++i;
        }

        return res;
    }

    void resample_internal(ProposalType &proposal,
                           CompactPopulation &population,
                           std::vector<double> &sorted_cumulative_probabilities,
                           std::vector<S> &result) {
        auto log_sum = population.log_sum();
        auto num_particles = population.num_particles();
        auto pop_after_collapse = sorted_cumulative_probabilities.size();
        auto normalized_partial_sum = 0.0;

        CompactPopulation sanity_check;

        result.reserve(pop_after_collapse);
        std::vector<S> candidate;  // hack for storing a single candidate

        for (auto i = 0ul; i < pop_after_collapse; ++i) {
            auto next_cumulative_probability = sorted_cumulative_probabilities[i];
            // sum normalized weights until we get to the next resampled cumulative probability
            while (normalized_partial_sum < next_cumulative_probability) {
                auto before = proposal.num_calls();
                auto next_log_weight_sample_pair = proposal.next_log_weight_sample_pair();
                auto weight = next_log_weight_sample_pair.first;
                if (proposal.num_calls() != before + 1) {
                    throw std::runtime_error("num_calls() incorrectly implemented in proposal");
                }
                if (candidate.empty()) {
                    candidate.emplace_back(std::move(next_log_weight_sample_pair.second));
                } else {
                    candidate[0] = std::move(next_log_weight_sample_pair.second);
                }
                normalized_partial_sum += std::exp(weight - log_sum);
                sanity_check.insert_log_weight(weight);
#ifndef NDEBUG
//                sgm::logger <= sanity_check.num_particles() <= ": " <= sanity_check.log_sum() <= ", "
//                            <= sanity_check.log_sum_of_squares() <= ", "
//                            <= sanity_check.ess() <= ", " <= weight <= "; "
//                            <= next_cumulative_probability <= ", " <= normalized_partial_sum <= '\n';
#endif
            }
            // we have found one particle that survived the collapse
            result.emplace_back(candidate[0]);
        }

        sgm::logger <= "resample before replay: "
                    <= proposal.num_calls() <= ", " <= sanity_check.num_particles() <= '\n';

        // replay the last few calls of the proposal sequence to make sure things were indeed behaving deterministically
        for (auto i = proposal.num_calls(); i < num_particles; i++) {
            auto weight = proposal.next_log_weight();

            sanity_check.insert_log_weight(weight);
#ifndef NDEBUG
//            sgm::logger <= sanity_check.num_particles() <= ": " <= sanity_check.log_sum() <= ", "
//                        <= sanity_check.log_sum_of_squares() <= ", "
//                        <= sanity_check.ess() <= ", " <= weight <= '\n';
#endif
        }

        sgm::logger <= "resample after replay: "
                    <= proposal.num_calls() <= ", " <= sanity_check.num_particles() <= '\n';

        if (sanity_check.log_sum() != log_sum || sanity_check.log_sum_of_squares() != population.log_sum_of_squares()) {
            throw sgm::runtime_error(
                "The provided proposal does not behave deterministically: " +
                std::to_string(sanity_check.log_sum()) +
                " vs " +
                std::to_string(log_sum)
            );
        }
    }

    void resample(CompactPopulation &population, std::vector<double> &sorted_cumulative_probabilities,
                  std::vector<S> &result) {
        if (m_proposal.num_calls() != 0) {
            // TODO: could replace with m_proposal
            auto proposal = m_proposal.restart();
            resample_internal(proposal, population, sorted_cumulative_probabilities, result);
        } else {
            resample_internal(m_proposal, population, sorted_cumulative_probabilities, result);
        }
    }

public:
    /**
     * Propose and then resample the given proposal.
     *
     * @param population A reference to an empty population to store population data.
     * @param result A reference to an empty vector to store the samples.
     */
    void execute(CompactPopulation &population, std::vector<S> &result) {
        propose(population);

#ifdef NDEBUG
        if (m_options.verbose) {
#endif
        sgm::logger << "nVirtual=" << population.num_particles() << ", "
                    << "nConcrete=" << m_options.num_concrete_particles << ", "
                    << "relative_ess=" << (population.ess() / m_options.num_concrete_particles)
                    << std::endl;
#ifdef NDEBUG
        }
#endif

#ifndef NDEBUG
        sgm::logger <= "--- RESAMPLE ---\n";
#endif

        auto sorted_cumulative_probabilities_for_final_resampling = extract_sorted_cumulative_probabilites();
        resample(population, sorted_cumulative_probabilities_for_final_resampling, result);
    }
};

}
}

#endif //SGMWSMCPP_STREAMINGPROPAGATOR_H
