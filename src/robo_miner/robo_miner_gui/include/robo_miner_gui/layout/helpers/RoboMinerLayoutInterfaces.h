#ifndef ROBO_MINER_GUI_ROBOMINERLAYOUTINTERFACES_H_
#define ROBO_MINER_GUI_ROBOMINERLAYOUTINTERFACES_H_

//C system headers

//C++ system headers
#include <cstdint>
#include <vector>

//Other libraries headers
#include "robo_common/layout/helpers/RoboCommonLayoutInterfaces.h"

//Own components headers

//Forward declarations
class CollisionWatcher;

struct RoboMinerLayoutInterface {
  RoboCommonLayoutInterface commonLayoutInterface;
};

struct RoboMinerLayoutOutInterface {
  FinishRobotActCb finishRobotActCb;
  CollisionWatcher *collisionWatcher = nullptr;
};

#endif /* ROBO_MINER_GUI_ROBOMINERLAYOUTINTERFACES_H_ */