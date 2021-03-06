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

#ifndef SGMWSMCPP_DEBUG_MACROS_H
#define SGMWSMCPP_DEBUG_MACROS_H

#define DECLARE_PRINT_OVERLOAD(...) \
    std::ostream &print(std::ostream &, const __VA_ARGS__ &, const std::string & = ", ")
#define DECLARE_PRINT_OVERLOAD_WITH_SEP(sep, ...) \
    std::ostream &print(std::ostream &, const __VA_ARGS__ &, const std::string & = sep)
#define DEFINE_PRINT_OVERLOAD(name, ...) \
    std::ostream &print(std::ostream &out, const __VA_ARGS__ &name, const std::string &sep)

#endif //SGMWSMCPP_DEBUG_MACROS_H
