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

#ifndef SGMWSMCPP_KNOTDATAREADERCONFIG_H
#define SGMWSMCPP_KNOTDATAREADERCONFIG_H

#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <vector>
#include <boost/filesystem.hpp>

#include "knot/data/EllipticalKnot.h"

namespace fs = boost::filesystem;

namespace sgm
{
struct KnotDataReaderConfig
{
    const static double Y_DIM[2];
    const static double Z_DIM[2];
    // following units are in inches
    const static int BOARD_LENGTH;
    const static int BOARD_WIDTH;
    const static int BOARD_HEIGHT;
};

namespace KnotDataReader
{

class Segment
{
public:
    using map_type = typename std::map<int, std::set<std::shared_ptr<EllipticalKnot>>>;
private:
    int m_id;
    map_type m_label_to_edge;
    std::vector<std::shared_ptr<EllipticalKnot>> m_knots;

public:
    explicit Segment(int id);
    int id() const;
    map_type &label_to_edge();
    const map_type &label_to_edge() const;
    std::vector<std::shared_ptr<EllipticalKnot>> &knots();
    const std::vector<std::shared_ptr<EllipticalKnot>> &knots() const;
    void add_node(int label, const std::shared_ptr<EllipticalKnot>& knot);
};

std::vector<Segment> read_segmented_test_board(const fs::path &file);
}
}

#endif //SGMWSMCPP_KNOTDATAREADERCONFIG_H
