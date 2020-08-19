#include <Rcpp.h>

#include <string>
#include <vector>

#include "src/Experiments.h"

using namespace Rcpp;

//' Run the train and predict experiment.
//'
//' @param training_files A string vector. List of CSV filenames containing the training data.
//' @param test_files A string vector. List of CSV filenames containing the test data.
//' @param concrete_particles An integer scalar. The minimum number of particles per training sample.
//' @param max_implicit_particles An integer scalar. The maximum number of particles per training sample.
//' @param target_ess An integer scalar. The minimum number of particles per test sample.
//' @param max_em_iter An integer scalar. The maximum number of iterations for MCEM. Default 10.
//' @param max_lbfgs_iter An integer scalar. The maximum number of iterations for LBFGS. Default 100.
//' @param seed An integer scalar. The seed to use for the random number generator. Default 123.
//' @param tol A numerical scalar. The tolerance to use for the LBFGS minimizer. Default \eqn{1^{-10}}.
//' @param use_spf A logical scalar. Should we use a streaming particle filter process? Default TRUE.
//' @param parallelize A logical scalar. Should the experiment be run with parallelism? Default TRUE.
//' @export
// [[Rcpp::export]]
DataFrame train_and_predict(StringVector training_files, StringVector test_files,
                            int concrete_particles = 100, int max_implicit_particles = 1000, int target_ess = 100,
                            int max_em_iter = 10, int max_lbfgs_iter = 100, int seed = 123, double tol = 1e-10,
                            bool use_spf = true, bool parallelize = true) {
    auto training_boards = std::vector<std::string>(training_files.begin(), training_files.end());
    auto test_boards = std::vector<std::string>(test_files.begin(), test_files.end());

    auto ret = sgm::train_and_predict(training_boards, test_boards,
                                      concrete_particles, max_implicit_particles, target_ess,
                                      max_em_iter, max_lbfgs_iter, seed, tol,
                                      use_spf, parallelize);

    auto names = std::vector<std::string>();
    auto pidxs = std::vector<int>();
    auto idxs = std::vector<int>();
    auto matchings = std::vector<int>();

    for (auto size = ret.size(), i = 0ul; i < size; ++i) {
      auto &dataset = ret[i];
      for (auto &row : dataset) {
        names.push_back(test_boards[i]);
        pidxs.push_back(row.pidx);
        idxs.push_back(row.idx);
        matchings.push_back(row.matching);
      }
    }

    return DataFrame::create(Named("dataset") = names,
                             Named("surface") = pidxs,
                             Named("idx") = idxs,
                             Named("matching") = matchings);
}
