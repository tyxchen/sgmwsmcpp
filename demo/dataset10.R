library(sgmwsmcpp)

training_files <- list.files("data/training10", pattern = ".csv$", full.names = TRUE)
test_files <- list.files("data/testing10", pattern = ".csv$", full.names = TRUE)

results <- train_and_predict(training_files, test_files)
