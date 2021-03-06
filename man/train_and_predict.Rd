% Generated by roxygen2: do not edit by hand
% Please edit documentation in R/RcppExports.R
\name{train_and_predict}
\alias{train_and_predict}
\title{Run the train and predict experiment.}
\usage{
train_and_predict(
  training_files,
  test_files,
  concrete_particles = 100L,
  max_implicit_particles = 1000L,
  target_ess = 100L,
  max_em_iter = 10L,
  max_lbfgs_iter = 100L,
  seed = 123L,
  tol = 1e-10,
  use_spf = TRUE,
  parallelize = TRUE
)
}
\arguments{
\item{training_files}{A string vector. List of CSV filenames containing the training data.}

\item{test_files}{A string vector. List of CSV filenames containing the test data.}

\item{concrete_particles}{An integer scalar. The minimum number of particles per training sample. Default 100.}

\item{max_implicit_particles}{An integer scalar. The maximum number of particles per training sample. Default 1000.}

\item{target_ess}{An integer scalar. The minimum number of particles per test sample. Default 100.}

\item{max_em_iter}{An integer scalar. The maximum number of iterations for MCEM. Default 10.}

\item{max_lbfgs_iter}{An integer scalar. The maximum number of iterations for LBFGS. Default 100.}

\item{seed}{An integer scalar. The seed to use for the random number generator. Default 123.}

\item{tol}{A numerical scalar. The tolerance to use for the LBFGS minimizer. Default \eqn{1^{-10}}.}

\item{use_spf}{A logical scalar. Should we use a streaming particle filter process? Default TRUE.}

\item{parallelize}{A logical scalar. Should the experiment be run with parallelism? Default TRUE.}
}
\value{
A dataframe containing the predicted matchings for each test dataset.
}
\description{
Run the train and predict experiment.
}
