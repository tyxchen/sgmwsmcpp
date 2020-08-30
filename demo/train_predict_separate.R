library(sgmwsmcpp)

# Load a training and a testing dataset, and run training and prediction separately.

training_files <- list.files("data/training10", pattern = ".csv$", full.names = TRUE)
test_files <- list.files("data/testing10", pattern = ".csv$", full.names = TRUE)

params <- train(training_files)
results <- data.frame(seed = integer(0), dataset = character(0),
                      surface = integer(0), idx = integer(0), matching = integer(0))

# Predict five times, each with a different randomly generated seed
for (seed in sample(1:100000, 5)) {
  result <- predict(test_files, params, seed = seed)
  result$seed <- rep(seed, nrow(result))
  results <- rbind(results, result)
}
