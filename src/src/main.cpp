#include <algorithm>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <cxxopts.hpp>

#include "utils/debug.h"
#include "utils/Random.h"
#include "Experiments.h"

using namespace sgm;
namespace fs = boost::filesystem;

int main(int argc, char** argv) {
    Timers::start("__main__");

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
    auto seed = args["seed"].as<Random::seed_type>();
    auto tol = args["tol"].as<double>();
    auto exact_sampling = args["exact-sampling"].as<bool>();
    auto sequential_matching = args["sequential-matching"].as<bool>();

    auto training_directories = args["data-directories"].as<std::vector<std::string>>();
    auto test_directories = args["test-data-directories"].as<std::vector<std::string>>();
    auto output_dir = args["output-dir"].as<std::string>();

    std::vector<std::string> BOARDS;
    std::vector<std::string> TEST_BOARDS;

    for (const auto &dir : training_directories) {
        auto path = fs::path(dir);
        if (!fs::is_directory(path)) {
            throw sgm::runtime_error("Error: Data directory `" + dir + "` is not a directory.");
        }
        auto dir_iter = fs::directory_iterator(path);
        for (const auto &dir_item : dir_iter) {
            auto board = dir_item.path().filename();
            if (board.string()[0] == '.') continue;
            BOARDS.emplace_back((path / board).string());
        }
        // Sort for compatibility with reference impl
        std::sort(BOARDS.begin(), BOARDS.end());
    }

    for (const auto &dir : test_directories) {
        auto path = fs::path(dir);
        if (!fs::is_directory(path)) {
            throw sgm::runtime_error("Error: Test data directory `" + dir + "` is not a directory.");
        }
        auto dir_iter = fs::directory_iterator(path);
        for (const auto &dir_item : dir_iter) {
            auto board = dir_item.path().filename();
            if (board.string()[0] == '.') continue;
            TEST_BOARDS.emplace_back((path / board).string());
        }
        // Sort for compatibility with reference impl
        std::sort(TEST_BOARDS.begin(), TEST_BOARDS.end());
    }

    auto results = train_and_predict(BOARDS, TEST_BOARDS,
                                     concrete_particles, max_implicit_particles, target_ess,
                                     max_em_iter, max_lbfgs_iter, seed, tol, use_spf, parallel);

    fs::create_directories(output_dir);

    for (auto size = results.size(), i = 0ul; i < size; ++i) {
        auto filename = fs::path(TEST_BOARDS[i]).filename();
        auto output_filename = output_dir / filename;
        auto log_output_filename = log_dir / filename;
        auto output_csv = std::ofstream(output_filename.string());
        auto log_output_csv = std::ofstream(log_output_filename.string());
        auto &result = results[i];

        for (auto &row : result) {
            output_csv << row.pidx << ","
                       << row.idx << ","
                       << row.matching << std::endl;
            log_output_csv << row.pidx << ","
                           << row.idx << ","
                           << row.matching << std::endl;
        }

        output_csv.close();
        log_output_csv.close();
    }

    Timers::end("__main__");
    sgm::logger << "Ran in " << Timers::diff("__main__") / 1000 << "ms" << std::endl;

    return 0;
}
