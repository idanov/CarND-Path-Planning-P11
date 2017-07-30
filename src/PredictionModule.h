#ifndef PATH_PLANNING_PREDICTIONMODULE_H
#define PATH_PLANNING_PREDICTIONMODULE_H

#include <map>
#include "Map.h"
#include "Car.h"

using namespace std;

class PredictionModule {
  const Map& world;
  const map<int, Car>& cars;
public:
  PredictionModule(const Map& world_, const map<int, Car>& cars_): world(world_), cars(cars_) {}
};


#endif //PATH_PLANNING_PREDICTIONMODULE_H
