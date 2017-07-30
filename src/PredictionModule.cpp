#include "PredictionModule.h"

vector<vector<Car>> PredictionModule::generatePredictions(int horizon) {
  vector<vector<Car>> predictions;
  for(auto pair_v : cars) {
    predictions.push_back(generatePredictions(pair_v.second, horizon));
  }
  return predictions;
}

vector<Car> PredictionModule::generatePredictions(Car car, int horizon) {
  vector<Car> predictions;
  for(int i = 0; i < horizon; i++) {
    predictions.push_back(car.stateAt(i * dt));
  }
  return predictions;
}
