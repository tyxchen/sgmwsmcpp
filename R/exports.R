#
# Copyright (c) 2020 Terry Chen <ty6chen@uwaterloo.ca>
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the
# Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
# Boston, MA  02110-1301, USA.
#


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
