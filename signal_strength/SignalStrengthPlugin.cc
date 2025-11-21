#include "SignalStrengthPlugin.hh"
#include <gz/gui/Application.hh>
#include <gz/plugin/Register.hh>
#include <iostream>

namespace gz::gui::plugins {

SignalStrengthPlugin::SignalStrengthPlugin()
    : dataPtr(std::make_unique<SignalStrengthPluginPrivate>()) {
  // Expose this plugin to QML
  App()->Engine()->rootContext()->setContextProperty("SignalStrengthPlugin",
                                                     this);
}

SignalStrengthPlugin::~SignalStrengthPlugin() = default;

void SignalStrengthPlugin::LoadConfig(const tinyxml2::XMLElement *) {
  if (this->title.empty())
    this->title = "RSRP Monitor";

  // Subscribe to RSRP topic from NS-3 (bridged from ROS /rsrp_values)
  this->dataPtr->node.Subscribe("/rsrp_values",
                                &SignalStrengthPlugin::OnRsrpMessage, this);

  // Subscribe to current cell ID topic
  this->dataPtr->node.Subscribe("/current_cell_id",
                                &SignalStrengthPlugin::OnCellIdMessage, this);
}

void SignalStrengthPlugin::OnRsrpMessage(const gz::msgs::Float_V &_msg) {
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  QList<float> rsrpList;
  for (int i = 0; i < _msg.data_size(); ++i) {
    rsrpList.append(_msg.data(i));
  }
  emit this->SetRsrpValues(rsrpList);
}

void SignalStrengthPlugin::OnCellIdMessage(const gz::msgs::UInt32 &_msg) {
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  int newCellId = static_cast<int>(_msg.data());

  std::cout << "Received cell ID: " << newCellId << std::endl; // Debug

  if (this->dataPtr->currentCellId != newCellId) {
    this->dataPtr->currentCellId = newCellId;
    std::cout << "Emitting CurrentCellIdChanged" << std::endl; // Debug
    emit this->CurrentCellIdChanged();
  }
}

QList<float> SignalStrengthPlugin::RsrpValues() const {
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->rsrpValues;
}

void SignalStrengthPlugin::SetRsrpValues(const QList<float> &_rsrpValues) {
  this->dataPtr->rsrpValues = _rsrpValues;
  this->RsrpValuesChanged();
}

int SignalStrengthPlugin::CurrentCellId() const {
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->currentCellId;
}

} // namespace gz::gui::plugins

GZ_ADD_PLUGIN(gz::gui::plugins::SignalStrengthPlugin, gz::gui::Plugin)
