//Corresponding header
#include "robo_collector_gui/panels/Panel.h"

//C system headers

//C++ system headers

//Other libraries headers
#include "utils/ErrorCode.h"
#include "utils/Log.h"

//Own components headers

int32_t Panel::init(const PanelConfig &cfg) {
  constexpr auto panelX = 1250;
  _panels[TIME_PANEL].create(cfg.timePanelRsrcId);
  _panels[TIME_PANEL].setPosition(panelX, 50);

  _panels[COIN_PANEL].create(cfg.coinPanelRsrcId);
  _panels[COIN_PANEL].setPosition(panelX, 215);

  _panels[HEALTH_PANEL].create(cfg.healthPanelRsrcId);
  _panels[HEALTH_PANEL].setPosition(panelX, 390);

  _healthIndicator.create(cfg.healthIndicatorRsrcId);
  _healthIndicator.setPosition(panelX + 79, 403);
  _healthIndicator.setCropRect(_healthIndicator.getCropRect());

  _horDelimiter.create(cfg.horDelimiterRsrcId);
  _horDelimiter.setPosition(1245, 500);

  _vertDelimiter.create(cfg.vertDelimiterRsrcId);
  _vertDelimiter.setPosition(1200, 550);

  return SUCCESS;
}

void Panel::draw() const {
  for (const auto& panel : _panels) {
    panel.draw();
  }

  _healthIndicator.draw();
  _horDelimiter.draw();
  _vertDelimiter.draw();
}

void Panel::decreaseHealthIndicator(int32_t damage) {
  auto cropRectangle = _healthIndicator.getCropRect();
  cropRectangle.w -= damage;
  _healthIndicator.setCropRect(cropRectangle);
}