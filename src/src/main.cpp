#include <iostream>
#include <string>
#include "utils/container/Counter.h"
#include "knot/data/EllipticalKnot.h"

struct C {
    int key;

    explicit C (int key) : key { key } {}
    C (const C &other) : key { other.key } {
        std::cerr<< "copy cc" << std::endl;
    }
};

std::ostream &operator<<(std::ostream &out, const C &c) {
    out << c.key;
    return out;
}

int main() {
    sgm::Counter<std::string> test {{"d", 0}};
//    auto a = C{1}, b = C{2};
auto a = "a";
    test.set(a, 2);
    test.set("b", 3);
    test.set("c", 5);
//    test.set(C{3}, 3);
    test.increment("b");
    std::cout << "Hello, World!" << std::endl;
    std::cout << test << std::endl;
    for (const auto &c : test) {
        std::cout << c.first << ":" << c.second << std::endl;
    }
    std::cout << test.arg_max() << " " << test.max() << std::endl;
    return 0;
}
