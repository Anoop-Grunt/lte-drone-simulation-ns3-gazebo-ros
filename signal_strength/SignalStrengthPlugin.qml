import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

Rectangle {
  id: signalStrengthRoot
  objectName: "signalStrengthRoot"
  Layout.minimumWidth: 300
  Layout.minimumHeight: 150
  color: "transparent"

  ColumnLayout {
    anchors.fill: parent
    anchors.margins: 10

    Text {
      text: "Signal Strength Monitor"
      font.bold: true
      font.pixelSize: 16
    }

    Rectangle {
      Layout.fillWidth: true
      height: 60
      color: "#e8f4f8"
      radius: 5
      border.color: "#4a90e2"
      border.width: 2

      ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10

        Text {
          text: "Signal Strength:"
          font.pixelSize: 12
          color: "#333"
        }

        Text {
          id: valueText
          text: SignalStrengthPlugin.signalStrength + " dBm"
          font.pixelSize: 18
          font.bold: true
          color: "#4a90e2"

          Connections {
            target: SignalStrengthPlugin
            onSignalStrengthChanged: {
              valueText.text = SignalStrengthPlugin.signalStrength + " dBm"
            }
          }
        }
      }
    }

    Item {
      Layout.fillHeight: true
    }
  }
}
