
import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

Rectangle {
    id: signalStrengthRoot
    objectName: "signalStrengthRoot"

    implicitWidth: 400
    implicitHeight:   1000

    color: "#1e1e1e"
    border.color: "#444444"
    border.width: 2
    radius: 8

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 15
        spacing: 15

        // Title
        Text {
            Layout.fillWidth: true
            text: "RSRP Monitor"
            font.pixelSize: 24
            font.bold: true
            color: "#ffffff"
            horizontalAlignment: Text.AlignHCenter
        }

        // Separator
        Rectangle {
            Layout.fillWidth: true
            height: 2
            color: "#444444"
        }

        // Current connected cell
        Rectangle {
            Layout.fillWidth: true
            height: 100
            color: "#28a745"
            radius: 8
            border.color: "#20c997"
            border.width: 2

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 10
                spacing: 5

                Text {
                    Layout.fillWidth: true
                    text: "CONNECTED TO"
                    font.pixelSize: 13
                    font.bold: true
                    color: "#ffffff"
                    horizontalAlignment: Text.AlignHCenter
                }

                Text {
                    Layout.fillWidth: true
                    text: "eNodeB " + SignalStrengthPlugin.currentCellId
                    font.pixelSize: 32
                    font.bold: true
                    color: "#ffffff"
                    horizontalAlignment: Text.AlignHCenter
                }

                Text {
                    Layout.fillWidth: true
                    text: "Cell ID: " + (SignalStrengthPlugin.currentCellId + 1)
                    font.pixelSize: 12
                    color: "#ffffff"
                    horizontalAlignment: Text.AlignHCenter
                }
            }
        }

        // Title for all cells
        Text {
            Layout.fillWidth: true
            text: "All eNodeBs"
            font.pixelSize: 18
            font.bold: true
            color: "#ffffff"
        }

        // Dynamic eNodeB boxes
        Repeater {
            model: SignalStrengthPlugin.rsrpValues.length

            delegate: Rectangle {
                Layout.fillWidth: true
                height: 100
                radius: 8
                color: (SignalStrengthPlugin.currentCellId === index) ? "#2d5016" : "#2d2d2d"
                border.color: ["#4a90e2", "#e2764a", "#e2c84a"][index % 3]
                border.width: 3

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 12
                    spacing: 8

                    Text {
                        Layout.fillWidth: true
                        text: "eNodeB " + index
                        font.pixelSize: 18
                        font.bold: true
                        color: "#ffffff"
                    }

                    Text {
                        Layout.fillWidth: true
                        text: SignalStrengthPlugin.rsrpValues[index].toFixed(2) + " dBm"
                        font.pixelSize: 26
                        font.bold: true
                        color: ["#4a90e2", "#e2764a", "#e2c84a"][index % 3]
                    }

                    Text {
                        Layout.fillWidth: true
                        text: (SignalStrengthPlugin.currentCellId === index) ? "● ACTIVE" : "○ Monitoring"
                        font.pixelSize: 12
                        color: (SignalStrengthPlugin.currentCellId === index) ? "#20c997" : "#888888"
                    }
                }
            }
        }

        Item { Layout.fillHeight: true }
    }
}

