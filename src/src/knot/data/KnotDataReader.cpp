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

const double KnotDataReaderConfig::Y_DIM[2] = { 200.0, 500.0 };
const double KnotDataReaderConfig::Z_DIM[2] = { 350.0, 500.0 };

Segment::Segment(int id) : m_id(id) {}

int Segment::id() const { return m_id; }

Segment::map_type &Segment::label_to_edge() { return m_label_to_edge; }
const Segment::map_type &Segment::label_to_edge() const { return m_label_to_edge; }

std::vector<std::shared_ptr<EllipticalKnot>> &Segment::knots() { return m_knots; }
const std::vector<std::shared_ptr<EllipticalKnot>> &Segment::knots() const { return m_knots; }

void Segment::add_node(int label, const std::shared_ptr<EllipticalKnot> &knot) {
    m_knots.push_back(knot);
    m_label_to_edge[label].insert(knot);
}

std::vector<Segment> KnotDataReader::read_segmented_test_board(const fs::path &file) {
    std::cout << "Processing " << file << std::endl;

    std::vector<Segment> segments;
    io::CSVReader<15> test_board(file.string());

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
        if (segments.empty()) {
            segments.emplace_back(1);
        }
        segments[0].add_node(label,
                             std::make_shared<EllipticalKnot>(pidx, idx, x, y, z, n, var_x, var_y, cov,
                                                              boundary_axis0, boundary_axis1,
                                                              yaxis, zaxis, area_over_axis));
    }

    return std::move(segments);
}
