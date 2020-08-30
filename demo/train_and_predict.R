library(sgmwsmcpp)

# Load a training and a testing dataset, and run the train and predict experiment.

training_files <- list.files("data/training10", pattern = ".csv$", full.names = TRUE)
test_files <- list.files("data/testing10", pattern = ".csv$", full.names = TRUE)

results <- train_and_predict(training_files, test_files)
