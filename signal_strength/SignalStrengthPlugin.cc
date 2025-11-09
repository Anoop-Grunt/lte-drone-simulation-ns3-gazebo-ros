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
    this->title = "Signal Strength Monitor";

  // Subscribe to signal strength topic
  this->dataPtr->node.Subscribe(
      "/signal_strength", &SignalStrengthPlugin::OnSignalStrengthMessage, this);
}

void SignalStrengthPlugin::OnSignalStrengthMessage(
    const gz::msgs::Double &_msg) {
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  std::cout << "Received signal strength: " << _msg.data() << std::endl;
  QString value = QString::number(_msg.data(), 'f', 2);
  emit this->SetSignalStrength(value);
}

QString SignalStrengthPlugin::SignalStrength() const {
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->signalStrengthValue;
}

void SignalStrengthPlugin::SetSignalStrength(const QString &_signalStrength) {
  std::cout << "Setting signal strength to: " << _signalStrength.toStdString()
            << std::endl;
  this->dataPtr->signalStrengthValue = _signalStrength;
  this->SignalStrengthChanged();
}

} // namespace gz::gui::plugins

GZ_ADD_PLUGIN(gz::gui::plugins::SignalStrengthPlugin, gz::gui::Plugin)
