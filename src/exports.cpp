#define STRICT_R_HEADERS
#include <Rcpp.h>

#include <string>
#include <utility>
#include <vector>

#include "src/utils/types.h"
#include "src/utils/container/Counter.h"
#include "src/knot/model/features/EllipticalKnotFeatureExtractor.h"
#include "src/Experiments.h"

using namespace Rcpp;

//' Train the decision model on the provided data.
//'
//' @param training_files A string vector. List of CSV filenames containing the training data.
//' @param concrete_particles An integer scalar. The minimum number of particles per training sample. Default 100.
//' @param max_implicit_particles An integer scalar. The maximum number of particles per training sample. Default 1000.
//' @param max_em_iter An integer scalar. The maximum number of iterations for MCEM. Default 10.
//' @param max_lbfgs_iter An integer scalar. The maximum number of iterations for LBFGS. Default 100.
//' @param seed An integer scalar. The seed to use for the random number generator. Default 123.
//' @param tol A numerical scalar. The tolerance to use for the LBFGS minimizer. Default \eqn{1^{-10}}.
//' @param use_spf A logical scalar. Should we use a streaming particle filter process? Default TRUE.
//' @param parallelize A logical scalar. Should the experiment be run with parallelism? Default TRUE.
//' @return A dataframe containing the sample means, sample standard deviations, and optimized values of each
//'         parameter.
//'
//' @export
// [[Rcpp::export]]
DataFrame train(const StringVector &training_files,
                int concrete_particles = 100, int max_implicit_particles = 1000,
                int max_em_iter = 10, int max_lbfgs_iter = 100, int seed = 123, double tol = 1e-10,
                bool use_spf = true, bool parallelize = true) {
    auto training_boards = std::vector<std::string>(training_files.begin(), training_files.end());

    auto ret = sgm::train(training_boards,
                          concrete_particles, max_implicit_particles, max_em_iter, max_lbfgs_iter,
                          seed, tol, use_spf, parallelize);

    auto types = StringVector();
    auto features = StringVector();
    auto values = NumericVector();

    for (auto &param : std::get<0>(ret)) {
        types.push_back("params");
        features.push_back(param.first.data());
        values.push_back(param.second);
    }

    for (auto &mean : std::get<1>(ret)) {
        types.push_back("mean");
        features.push_back(mean.first.data());
        values.push_back(mean.second);
    }

    for (auto &sd : std::get<2>(ret)) {
        types.push_back("sd");
        features.push_back(sd.first.data());
        values.push_back(sd.second);
    }

    return DataFrame::create(Named("type") = types, Named("feature") = features, Named("value") = values);
}

// [[Rcpp::export]]
DataFrame predict_internal(const StringVector &test_files,
                           const StringVector &types,
                           const StringVector &features,
                           const NumericVector &values,
                           int target_ess, int seed,
                           bool use_spf, bool parallelize) {
    auto test_boards = std::vector<std::string>(test_files.begin(), test_files.end());

    auto params = sgm::Counter<sgm::string_t>();
    auto mean = sgm::Counter<sgm::string_t>();
    auto sd = sgm::Counter<sgm::string_t>();

    for (size_t nrows = types.size(), i = 0; i < nrows; ++i) {
        auto ctr = static_cast<sgm::Counter<sgm::string_t> *>(nullptr);
        auto type = Rcpp::as<std::string>(types[i]);
        if (type == "params") {
            ctr = &params;
        } else if (type == "mean") {
            ctr = &mean;
        } else if (type == "sd") {
            ctr = &sd;
        } else {
            stop("Unexpected type '%s' in trained params", type);
        }

        auto feature = Rcpp::as<std::string>(features[i]);
        auto value = values[i];
        auto feature_name = static_cast<const sgm::string_t *>(nullptr);

        if (feature.data() == sgm::EllipticalKnotFeatureExtractorConsts::TWO_MATCHING_DISTANCE_1) {
            feature_name = &sgm::EllipticalKnotFeatureExtractorConsts::TWO_MATCHING_DISTANCE_1;
        } else if (feature.data() == sgm::EllipticalKnotFeatureExtractorConsts::TWO_MATCHING_DISTANCE_2) {
            feature_name = &sgm::EllipticalKnotFeatureExtractorConsts::TWO_MATCHING_DISTANCE_2;
        } else if (feature.data() == sgm::EllipticalKnotFeatureExtractorConsts::TWO_MATCHING_AREA_DIFF) {
            feature_name = &sgm::EllipticalKnotFeatureExtractorConsts::TWO_MATCHING_AREA_DIFF;
        } else if (feature.data() == sgm::EllipticalKnotFeatureExtractorConsts::THREE_MATCHING_DISTANCE_1) {
            feature_name = &sgm::EllipticalKnotFeatureExtractorConsts::THREE_MATCHING_DISTANCE_1;
        } else if (feature.data() == sgm::EllipticalKnotFeatureExtractorConsts::THREE_MATCHING_DISTANCE_2) {
            feature_name = &sgm::EllipticalKnotFeatureExtractorConsts::THREE_MATCHING_DISTANCE_2;
        } else if (feature.data() == sgm::EllipticalKnotFeatureExtractorConsts::THREE_MATCHING_AREA_DIFF) {
            feature_name = &sgm::EllipticalKnotFeatureExtractorConsts::THREE_MATCHING_AREA_DIFF;
        } else {
            stop("Unknown feature name '%s'", feature);
        }

        ctr->set(*feature_name, value);
    }

    auto ret = sgm::predict(test_boards, params, mean, sd, target_ess, seed, use_spf, parallelize);

    auto names = std::vector<std::string>();
    auto pidxs = std::vector<int>();
    auto idxs = std::vector<int>();
    auto matchings = std::vector<int>();

    for (size_t size = ret.size(), i = 0; i < size; ++i) {
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

//' Run the train and predict experiment.
//'
//' @param training_files A string vector. List of CSV filenames containing the training data.
//' @param test_files A string vector. List of CSV filenames containing the test data.
//' @param concrete_particles An integer scalar. The minimum number of particles per training sample. Default 100.
//' @param max_implicit_particles An integer scalar. The maximum number of particles per training sample. Default 1000.
//' @param target_ess An integer scalar. The minimum number of particles per test sample. Default 100.
//' @param max_em_iter An integer scalar. The maximum number of iterations for MCEM. Default 10.
//' @param max_lbfgs_iter An integer scalar. The maximum number of iterations for LBFGS. Default 100.
//' @param seed An integer scalar. The seed to use for the random number generator. Default 123.
//' @param tol A numerical scalar. The tolerance to use for the LBFGS minimizer. Default \eqn{1^{-10}}.
//' @param use_spf A logical scalar. Should we use a streaming particle filter process? Default TRUE.
//' @param parallelize A logical scalar. Should the experiment be run with parallelism? Default TRUE.
//' @return A dataframe containing the predicted matchings for each test dataset.
//'
//' @export
// [[Rcpp::export]]
DataFrame train_and_predict(const StringVector &training_files, const StringVector &test_files,
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

    for (size_t size = ret.size(), i = 0; i < size; ++i) {
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
