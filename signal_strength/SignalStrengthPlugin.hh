#ifndef SIGNALSTRENGTHPLUGIN_HH_
#define SIGNALSTRENGTHPLUGIN_HH_

#include <QList>
#include <QString>
#include <gz/gui/Plugin.hh>
#include <gz/msgs/float_v.pb.h>
#include <gz/msgs/uint32.pb.h>
#include <gz/transport/Node.hh>
#include <memory>
#include <mutex>
#include <vector>

namespace gz::gui::plugins {

class SignalStrengthPluginPrivate {
public:
  QList<float> rsrpValues;
  int currentCellId = 0;
  std::mutex mutex;
  gz::transport::Node node;
};

class SignalStrengthPlugin : public gz::gui::Plugin {
  Q_OBJECT
  Q_PROPERTY(QList<float> rsrpValues READ RsrpValues WRITE SetRsrpValues NOTIFY
                 RsrpValuesChanged)
  Q_PROPERTY(int currentCellId READ CurrentCellId NOTIFY CurrentCellIdChanged)

public:
  SignalStrengthPlugin();
  virtual ~SignalStrengthPlugin();
  virtual void LoadConfig(const tinyxml2::XMLElement *_pluginElem);

private:
  void OnRsrpMessage(const gz::msgs::Float_V &_msg);
  void OnCellIdMessage(const gz::msgs::UInt32 &_msg);

public:
  Q_INVOKABLE QList<float> RsrpValues() const;
  Q_INVOKABLE int CurrentCellId() const;

public slots:
  void SetRsrpValues(const QList<float> &_rsrpValues);

signals:
  void RsrpValuesChanged();
  void CurrentCellIdChanged();

private:
  std::unique_ptr<SignalStrengthPluginPrivate> dataPtr;
};

} // namespace gz::gui::plugins

#endif
