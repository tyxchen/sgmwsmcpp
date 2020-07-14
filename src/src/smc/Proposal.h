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

#ifndef SGMWSMCPP_PROPOSAL_H
#define SGMWSMCPP_PROPOSAL_H

#include <utility>

namespace sgm
{
namespace smc
{

template <typename S, typename Derived>
class Proposal
{
    size_t m_num_calls = 0;

public:
    std::pair<double, S> next_log_weight_sample_pair() {
        ++m_num_calls;
        return _next_log_weight_sample_pair();
    }

    double next_log_weight() {
        ++m_num_calls;
        return _next_log_weight();
    }

    double num_calls() const {
        return m_num_calls;
    }

    Derived restart() {
        return _restart();
    }

    bool has_next_log_weight_sample_pair() {
        return _has_next_log_weight_sample_pair();
    }

private:
    virtual std::pair<double, S> _next_log_weight_sample_pair() = 0;
    virtual double _next_log_weight() {
        return _next_log_weight_sample_pair().first;
    }
    virtual Derived _restart() = 0;
    virtual bool _has_next_log_weight_sample_pair() {
        return true;
    }
};

}
}

#endif //SGMWSMCPP_PROPOSAL_H
