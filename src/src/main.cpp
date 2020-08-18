#include <cmath>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>
#include <utility>

#include <boost/filesystem.hpp>
#include <cxxopts.hpp>

#include "utils/debug.h"
#include "utils/types.h"
#include "knot/data/EllipticalKnot.h"
#include "common/learning/SupervisedLearning.h"
#include "utils/ExpUtils.h"
#include "utils/Random.h"
#include "common/model/PairwiseMatchingModel.h"
#include "knot/model/features/EllipticalKnotFeatureExtractor.h"
#include "common/model/Command.h"
#include "common/graph/GraphMatchingState.h"
#include "smc/components/ExactProposalObservationDensity.h"
#include "smc/components/GenericMatchingLatentSimulator.h"

using namespace sgm;
namespace fs = boost::filesystem;

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
        ("seed", "", cxxopts::value<Random::seed_type>()->default_value("123"))
        ("tol", "", cxxopts::value<double>()->default_value("1e-10"))
        ("exact-sampling", "", cxxopts::value<bool>()->default_value("true"))
        ("sequential-matching", "", cxxopts::value<bool>()->default_value("true"))
        ("trace", "", cxxopts::value<std::string>()->default_value(""))
        ("d,data-directories", "", cxxopts::value<std::vector<std::string>>())
        ("t,test-data-directories", "", cxxopts::value<std::vector<std::string>>())
        ("o,output-dir", "", cxxopts::value<std::string>());

    auto args = options.parse(argc, argv);

    // Setup log file
    auto log_dir = fs::path(args["trace"].as<std::string>());

    if (log_dir.empty()) {
        auto time = std::time(nullptr);
        char log_dir_fmt[100];
        std::strftime(log_dir_fmt, sizeof(log_dir_fmt) - 1, "%Y-%m-%d-%H-%M-%S", std::localtime(&time));
        log_dir = "results";
        log_dir /= std::string(log_dir_fmt);
        fs::create_directories(log_dir);

        // make a symlink
        try {
            fs::remove("results/latest");
            fs::create_symlink(std::string(log_dir_fmt), "results/latest");
        } catch (...) {
            std::cerr << "Could not create a symlink to the latest log, maybe your system does not support it?\n";
        }
    }

    sgm::logger.set_log_file((log_dir / "stdout.log").string());

    auto use_spf = args["use-spf"].as<bool>();
    auto target_ess = args["target-ess"].as<int>();
    auto concrete_particles = args["concrete-particles"].as<int>();
    auto max_implicit_particles = args["max-implicit-particles"].as<int>();
    auto lambda = args["lambda"].as<double>();
    auto max_em_iter = args["max-em-iter"].as<int>();
    auto parallel = args["parallel"].as<bool>();
    auto max_lbfgs_iter = args["max-lbfgs-iter"].as<int>();
    auto random = Random(args["seed"].as<Random::seed_type>());
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
            throw sgm::runtime_error("Error: Data directory `" + dir + "` is not a directory.");
        }
        auto dir_iter = fs::directory_iterator(path);
        for (const auto &dir_item : dir_iter) {
            auto board = dir_item.path().filename();
            if (board.string()[0] == '.') continue;
            BOARDS.emplace_back(path / board);
        }
        // Sort for compatibility with reference impl
        std::sort(BOARDS.begin(), BOARDS.end());
    }

    for (const auto &dir : test_data_directories) {
        auto path = fs::path(dir);
        if (!fs::is_directory(path)) {
            throw sgm::runtime_error("Error: Test data directory `" + dir + "` is not a directory.");
        }
        auto dir_iter = fs::directory_iterator(path);
        for (const auto &dir_item : dir_iter) {
            auto board = dir_item.path().filename();
            if (board.string()[0] == '.') continue;
            TEST_BOARDS.emplace_back(path / board);
        }
        // Sort for compatibility with reference impl
        std::sort(TEST_BOARDS.begin(), TEST_BOARDS.end());
    }

    SupervisedLearningConfig::parallel = parallel;
    SupervisedLearningConfig::num_lbfgs_iters = max_lbfgs_iter;

    sgm::logger << "Current working dir: " << fs::current_path() << std::endl;

    auto training_instances = ExpUtils::read_test_boards(BOARDS, false);
    auto training_data = ExpUtils::unpack<EllipticalKnot>(training_instances);

    auto test_instances = ExpUtils::read_test_boards(TEST_BOARDS, false);
    sgm::logger << "Test instances size: " << test_instances.size() << std::endl;

    PairwiseMatchingModel<std::string, EllipticalKnot> decision_model;
    EllipticalKnotFeatureExtractor fe;
    auto fe_dim = fe.dim();
    Command<std::string, EllipticalKnot> command(decision_model, fe);

    // compute the features on each of these data points and standardization
    std::vector<std::vector<double>> standardizations(fe_dim);

    for (const auto &data : training_data) {
        for (const auto &datum : data) {
            for (const auto &e : datum.first) {
                auto phi = fe.extract_features(e);
                for (const auto &vals : phi) {
                    standardizations[command.indexer().o2i(vals.first)].push_back(phi.get(vals.first));
                }
            }
        }
    }

    Counter<std::string> mean;
    Counter<std::string> sd;

    for (auto i = 0; i < fe_dim; ++i) {
        auto f = command.indexer().i2o(i);
        auto &standardization = standardizations[i];
        auto n = static_cast<double>(standardization.size());

        // calculate mean
        auto m = 0.0;
        for (auto s : standardization) {
            m += s;
        }
        m /= n;

        // calculate variance
        auto var = 0.0;
        for (auto s : standardization) {
            var += (s - m) * (s - m);
        }

        mean.set(f, m);
        sd.set(f, std::sqrt(var / (n - 1)));
    }

    fe.standardize(mean, sd);

    /*
     * Train the model parameters and run SMC on test data
     */
    const auto &fe_mean = fe.mean();
    const auto &fe_sd = fe.sd();
    for (auto i = 0; i < fe_dim; ++i) {
        auto f = command.indexer().i2o(i);
        sgm::logger << "Feature " << i << ":" << std::endl;
        sgm::logger << "  name: " << f << std::endl;
        sgm::logger << "  mean: " << fe_mean.get(f) << std::endl;
        sgm::logger << "  sd:   " << fe_sd.get(f) << std::endl;
    }

    auto initial = Eigen::VectorXd::Zero(fe_dim);
    auto instances = ExpUtils::pack<EllipticalKnot>(training_data);
    auto ret = SupervisedLearning::MAP_via_MCEM(random, 0, command, instances, max_em_iter, concrete_particles,
        max_implicit_particles, initial, tol, log_dir, false, use_spf);

    sgm::logger << ret.first << "\n" << ret.second << std::endl;

    command.update_model_parameters(ret.second);

    fs::create_directories(output_dir);

    for (auto j = 0ul, size = test_instances.size(); j < size; ++j) {
        auto segment = test_instances[j][0];
        auto initial_state = GraphMatchingState<std::string, EllipticalKnot>(segment.knots());
        auto transition_density = smc::GenericMatchingLatentSimulator<std::string, EllipticalKnot>(command,
                                                                                                   initial_state,
                                                                                                   false, true);
        auto observation_density = smc::ExactProposalObservationDensity<std::string, EllipticalKnot>(command);
        auto emissions = std::vector<node_type>(segment.knots());

        auto smc = smc::SequentialGraphMatchingSampler<std::string, EllipticalKnot>(transition_density,
                                                                                    observation_density,
                                                                                    emissions, use_spf);

        Timers::start("testing data");
        smc.sample(random, target_ess, 1000);
        Timers::end("testing data");

        sgm::logger << "Run time: " << Timers::diff("testing data") << "ms\n";

        auto &samples = smc.samples();
        auto best_log_density = -std::numeric_limits<double>::infinity();
        auto best_sample = samples[0];

        for (auto &sample : samples) {
            if (sample.log_density() > best_log_density) {
                best_log_density = sample.log_density();
                best_sample = sample;
            }
        }

        auto filename = TEST_BOARDS[j].filename();
        auto output_filename = output_dir / filename;
        auto log_output_filename = log_dir / filename;
        auto output_csv = std::ofstream(output_filename.string());
        auto log_output_csv = std::ofstream(log_output_filename.string());
        auto idx = 0;

        for (auto &matching : best_sample.matchings()) {
            for (auto &knot : *matching) {
                output_csv << knot->pidx() << "," << knot->idx() << "," << idx << std::endl;
                log_output_csv << knot->pidx() << "," << knot->idx() << "," << idx << std::endl;
            }
            ++idx;
        }
        output_csv.close();
        log_output_csv.close();
    }

    return 0;
}
