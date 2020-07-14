#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include <boost/filesystem.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <cxxopts.hpp>
#include <parallel_hashmap/phmap.h>

#include "utils/types.h"
#include "knot/data/EllipticalKnot.h"
#include "knot/data/KnotDataReader.h"
#include "common/learning/SupervisedLearning.h"
#include "utils/ExpUtils.h"
#include "utils/Random.h"
#include "common/model/PairwiseMatchingModel.h"
#include "knot/model/features/EllipticalKnotFeatureExtractor.h"
#include "common/model/Command.h"

using namespace sgm;
namespace fs = boost::filesystem;
namespace acc = boost::accumulators;

using node_type = node_type_base<EllipticalKnot>;
using edge_type = edge_type_base<EllipticalKnot>;
using datum_type = typename std::vector<std::pair<std::vector<edge_type>, std::vector<node_type>>>;

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
    auto random = Random(args["seed"].as<int>());
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
    auto training_data = ExpUtils::unpack<EllipticalKnot>(training_instances);

    auto test_instances = ExpUtils::read_test_boards(TEST_BOARDS, false);
    std::cout << test_instances.size() << std::endl;

    PairwiseMatchingModel<std::string, EllipticalKnot> decision_model;
    EllipticalKnotFeatureExtractor fe;
    auto fe_dim = fe.dim();
    Command<std::string, EllipticalKnot> command(decision_model, fe);

    // compute the features on each of these data points and standardization
    std::vector<acc::accumulator_set<double,
                                     acc::stats<acc::tag::count, acc::tag::mean, acc::tag::variance(acc::lazy)>>>
        standardizations(fe_dim);

    for (const auto &data : training_data) {
        for (const auto &datum : data) {
            for (const auto &e : datum.first) {
                auto phi = fe.extract_features(e);
                for (const auto &vals : phi) {
                    standardizations[command.indexer().o2i(vals.first)](phi.get(vals.first));
                }
            }
        }
    }

    Counter<std::string> mean;
    Counter<std::string> sd;

    for (auto i = 0; i < fe_dim; ++i) {
        auto f = command.indexer().i2o(i);
        auto n = static_cast<double>(acc::count(standardizations[i]));
        mean.set(f, acc::mean(standardizations[i]));
        sd.set(f, std::sqrt((n / (n - 1)) * acc::variance(standardizations[i])));
    }

    fe.standardize(std::move(mean), std::move(sd));

    /*
     * Train the model parameters and run SMC on test data
     */
    const auto &fe_mean = fe.mean();
    const auto &fe_sd = fe.sd();
    for (auto i = 0; i < fe_dim; ++i) {
        auto f = command.indexer().i2o(i);
        std::cout << "Feature " << i << ":" << std::endl;
        std::cout << "  name: " << f << std::endl;
        std::cout << "  mean: " << fe_mean.get(f) << std::endl;
        std::cout << "  sd:   " << fe_sd.get(f) << std::endl;
    }

    auto initial = Eigen::VectorXd::Zero(fe_dim);
    auto instances = ExpUtils::pack<EllipticalKnot>(training_data);
    SupervisedLearning::MAP_via_MCEM(random, 0, command, instances, max_em_iter, concrete_particles,
        max_implicit_particles, initial, tol, use_spf);

    return 0;
}
