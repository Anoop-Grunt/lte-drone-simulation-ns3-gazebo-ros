#ifndef SIGNALSTRENGTHPLUGIN_HH_
#define SIGNALSTRENGTHPLUGIN_HH_

#include <QString>
#include <gz/gui/Plugin.hh>
#include <gz/msgs/double.pb.h>
#include <gz/transport/Node.hh>
#include <memory>
#include <mutex>

namespace gz::gui::plugins {

class SignalStrengthPluginPrivate {
public:
  QString signalStrengthValue{"--"};

public:
  std::mutex mutex;

public:
  gz::transport::Node node;
};

class SignalStrengthPlugin : public gz::gui::Plugin {
  Q_OBJECT

  Q_PROPERTY(QString signalStrength READ SignalStrength WRITE SetSignalStrength
                 NOTIFY SignalStrengthChanged)

public:
  SignalStrengthPlugin();

public:
  virtual ~SignalStrengthPlugin();

public:
  virtual void LoadConfig(const tinyxml2::XMLElement *_pluginElem);

private:
  void OnSignalStrengthMessage(const gz::msgs::Double &_msg);

public:
  Q_INVOKABLE QString SignalStrength() const;

public slots:
  void SetSignalStrength(const QString &_signalStrength);

signals:
  void SignalStrengthChanged();

private:
  std::unique_ptr<SignalStrengthPluginPrivate> dataPtr;
};

} // namespace gz::gui::plugins

#endif
