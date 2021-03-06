#ifndef PATH_PLANNING_PREDICTIONMODULE_H
#define PATH_PLANNING_PREDICTIONMODULE_H

#include <map>
#include "helpers.h"
#include "Map.h"
#include "Car.h"

using namespace std;

class PredictionModule {
  Map& world;
  const map<int, Car>& cars;
  vector<Car> generatePredictions(Car car, int horizon);
public:
  PredictionModule(Map& world_, const map<int, Car>& cars_): world(world_), cars(cars_) {}
  vector<vector<Car>> generatePredictions(int horizon = 50);
};


#endif //PATH_PLANNING_PREDICTIONMODULE_H
