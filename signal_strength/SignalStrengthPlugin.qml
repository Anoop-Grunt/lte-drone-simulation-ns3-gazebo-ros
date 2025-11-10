import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

Rectangle {
  id: signalStrengthRoot
  objectName: "signalStrengthRoot"
  Layout.minimumWidth: 350
  Layout.minimumHeight: 250
  color: "transparent"
  
  ColumnLayout {
    anchors.fill: parent
    anchors.margins: 10
    spacing: 10
    
    Text {
      text: "RSRP Monitor"
      font.bold: true
      font.pixelSize: 18
      color: "#333"
    }
    
    // Dynamic eNodeB boxes
    Repeater {
      model: SignalStrengthPlugin.rsrpValues.length
      
      Rectangle {
        Layout.fillWidth: true
        height: 60
        color: "#e8f4f8"
        radius: 5
        border.color: ["#4a90e2", "#e2764a", "#e2c84a"][index % 3]
        border.width: 2
        
        ColumnLayout {
          anchors.fill: parent
          anchors.margins: 10
          
          Text {
            text: "eNodeB " + index + ":"
            font.pixelSize: 12
            color: "#333"
          }
          
          Text {
            text: SignalStrengthPlugin.rsrpValues[index].toFixed(2) + " dBm"
            font.pixelSize: 16
            font.bold: true
            color: ["#4a90e2", "#e2764a", "#e2c84a"][index % 3]
          }
        }
      }
    }
    
    Item {
      Layout.fillHeight: true
    }
  }
}
