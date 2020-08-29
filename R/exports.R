#' Predict matchings on test data.
#'
#' @param test_files A string vector. List of CSV filenames containing the test data.
#' @param trained_params A dataframe. Output of predict().
#' @param target_ess An integer scalar. The minimum number of particles per test sample. Default 100.
#' @param seed An integer scalar. The seed to use for the random number generator. Default 123.
#' @param use_spf A logical scalar. Should we use a streaming particle filter process? Default TRUE.
#' @param parallelize A logical scalar. Should the experiment be run with parallelism? Default TRUE.
#' @return A dataframe containing the predicted matchings for each test dataset.
#'
#' @export
predict <- function(test_files, trained_params, target_ess = 100, seed = 123, use_spf = TRUE, parallelize = TRUE) {
  predict_internal(test_files, trained_params$type, trained_params$feature, trained_params$value,
                   target_ess, seed, use_spf, parallelize)
}
