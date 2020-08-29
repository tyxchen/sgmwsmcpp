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

#include "KnotDataReader.h"
#include <csv.h>

using namespace sgm;
using namespace sgm::KnotDataReader;

Segment::Segment(int id) : m_id(id) {}

int Segment::id() const { return m_id; }

Segment::map_type &Segment::label_to_edge() { return m_label_to_edge; }
const Segment::map_type &Segment::label_to_edge() const { return m_label_to_edge; }

std::vector<Segment::node_type> &Segment::knots() { return m_knots; }
const std::vector<Segment::node_type> &Segment::knots() const { return m_knots; }

void Segment::add_node(int label, const node_type &knot) {
    m_knots.push_back(knot);
    if (!m_label_to_edge.count(label)) {
        m_label_to_edge.emplace(label, std::make_shared<typename edge_type::element_type>());
    }
    m_label_to_edge[label]->insert(knot);
}

Segment KnotDataReader::read_segmented_test_board(const std::string &file) {
    std::cout << "Processing " << file << std::endl;

    Segment segment(1);
    io::CSVReader<15> test_board(file);
//    test_board.read_header(io::ignore_extra_column,
//                           "surface", "x", "y", "var_x", "var_y", "cov", "n", "idx", "z",
//                           "boundary_axis_idx1", "boundary_axis_idx2", "yaxis", "zaxis", "area_over",
//                           "matching");
    test_board.next_line();

    int pidx;
    double x, y;
    double var_x, var_y, cov;
    double n;
    int idx;
    double z;
    int boundary_axis0, boundary_axis1;
    double yaxis, zaxis;
    double area_over_axis;
    int label;
    while (test_board.read_row(
        pidx, x, y, var_x, var_y, cov, n, idx, z, boundary_axis0, boundary_axis1, yaxis, zaxis, area_over_axis, label
    )) {
        segment.add_node(label, std::make_shared<EllipticalKnot>(pidx, idx, x, y, z, n, var_x, var_y, cov,
                                                                 boundary_axis0, boundary_axis1,
                                                                 yaxis, zaxis, area_over_axis));
    }

    return segment;
}
