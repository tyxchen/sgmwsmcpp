#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <vector>
#include <utility>

#include <boost/filesystem.hpp>
#include <cxxopts.hpp>

#include "knot/data/EllipticalKnot.h"
#include "knot/data/KnotDataReader.h"
#include "common/learning/SupervisedLearning.h"
#include "utils/ExpUtils.h"
#include "utils/container/Counter.h"

using namespace sgm;
namespace fs = boost::filesystem;

using datum_type = typename std::vector<std::pair<std::vector<std::set<std::shared_ptr<EllipticalKnot>>>,
    std::vector<std::shared_ptr<EllipticalKnot>>>>;

std::vector<datum_type> unpack(std::vector<std::vector<KnotDataReader::Segment>> instances) {
    std::vector<datum_type> data;
    for (auto &instance : instances) {
        datum_type datum;
        for (auto &segment : instance) {
            std::vector<std::set<std::shared_ptr<EllipticalKnot>>> edges;
            for (auto &matching : segment.label_to_edge()) {
                edges.emplace_back(std::move(matching.second));
            }
            datum.emplace_back(std::move(edges), std::move(segment.knots()));
        }
        data.emplace_back(std::move(datum));
    }
    return std::move(data);
}

int main(int argc, char** argv) {
    cxxopts::Options options("sgmwsmc", "Description");

    options.add_options()
        ("use-spf", "Use SPF", cxxopts::value<bool>()->default_value("true"))
        ("target-ess", "", cxxopts::value<int>()->default_value("100"))
        ("concrete-particles", "Number of concrete particles", cxxopts::value<int>()->default_value("100"))
        ("max-implicit-particles", "Maximum number of implicit particles",
            cxxopts::value<int>()->default_value("1000"))
        ("lambda", "", cxxopts::value<double>()->default_value("10.0"))
        ("max-em-iter", "", cxxopts::value<int>()->default_value("10"))
        ("parallel", "", cxxopts::value<bool>()->default_value("true"))
        ("max-lbfgs-iter", "", cxxopts::value<int>()->default_value("100"))
        ("seed", "", cxxopts::value<int>()->default_value("123"))
        ("tol", "", cxxopts::value<double>()->default_value("1e-10"))
        ("exact-sampling", "", cxxopts::value<bool>()->default_value("true"))
        ("sequential-matching", "", cxxopts::value<bool>()->default_value("true"))
        ("d,data-directories", "", cxxopts::value<std::vector<std::string>>())
        ("t,test-data-directories", "", cxxopts::value<std::vector<std::string>>())
        ("o,output-dir", "", cxxopts::value<std::string>());

    auto args = options.parse(argc, argv);

    auto use_spf = args["use-spf"].as<bool>();
    auto target_ess = args["target-ess"].as<int>();
    auto concrete_particles = args["concrete-particles"].as<int>();
    auto max_implicit_particles = args["max-implicit-particles"].as<int>();
    auto lambda = args["lambda"].as<double>();
    auto max_em_iter = args["max-em-iter"].as<int>();
    auto parallel = args["parallel"].as<bool>();
    auto max_lbfgs_iter = args["max-lbfgs-iter"].as<int>();
    auto seed = args["seed"].as<int>();
    auto tol = args["tol"].as<double>();
    auto exact_sampling = args["exact-sampling"].as<bool>();
    auto sequential_matching = args["sequential-matching"].as<bool>();

    auto data_directories = args["data-directories"].as<std::vector<std::string>>();
    auto test_data_directories = args["test-data-directories"].as<std::vector<std::string>>();
    auto output_dir = args["output-dir"].as<std::string>();

    std::vector<fs::path> BOARDS;
    std::vector<fs::path> TEST_BOARDS;

    for (const auto &dir : data_directories) {
        auto path = fs::path(dir);
        if (!fs::is_directory(path)) {
            std::cerr << "Error: Data directory `" << dir << "` is not a directory." << std::endl;
            return 1;
        }
        auto dir_iter = fs::directory_iterator(path);
        for (const auto &dir_item : dir_iter) {
            auto board = dir_item.path().filename();
            if (board.string()[0] == '.') continue;
            BOARDS.emplace_back(path / board);
        }
    }

    for (const auto &dir : test_data_directories) {
        auto path = fs::path(dir);
        if (!fs::is_directory(path)) {
            std::cerr << "Error: Test data directory `" << dir << "` is not a directory." << std::endl;
            return 1;
        }
        auto dir_iter = fs::directory_iterator(path);
        for (const auto &dir_item : dir_iter) {
            auto board = dir_item.path().filename();
            if (board.string()[0] == '.') continue;
            TEST_BOARDS.emplace_back(path / board);
        }
    }

    SupervisedLearningConfig::parallel = parallel;
    SupervisedLearningConfig::num_lbfgs_iters = max_lbfgs_iter;

    std::cout << "Current working dir: " << fs::current_path() << std::endl;

    auto training_instances = ExpUtils::read_test_boards(BOARDS, false);
    auto training_data = unpack(std::move(training_instances));

    auto test_instances = ExpUtils::read_test_boards(TEST_BOARDS, false);
    std::cout << test_instances.size() << std::endl;

    return 0;
}
