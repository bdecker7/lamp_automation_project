// Button_PressView.mc
// Draws BLE connection status directly on screen so you can debug
// without needing log files or a simulator.
//
// STATUS MESSAGES YOU WILL SEE:
//   "Scanning..."       - app started, looking for ESP32
//   "Found ESP32"       - ESP32 found, attempting to pair
//   "Connected!"        - BLE connection established
//   "Svc OK"            - GATT service found on ESP32
//   "Char Ready!"       - Characteristic found, ready to send
//   "Svc NULL!"         - Service not found (UUID mismatch)
//   "Char NULL!"        - Characteristic not found (UUID or flag issue)
//   "Sending ON"        - Write command sent for Lights ON
//   "Sending OFF"       - Write command sent for Lights OFF
//   "Write OK"          - ESP32 confirmed the write
//   "Write FAIL"        - Write was rejected or failed
//   "Disconnected"      - BLE connection dropped

import Toybox.Graphics;
import Toybox.WatchUi;
import Toybox.Lang;
import Toybox.System;

class Button_PressView extends WatchUi.View {

    function initialize() {
        View.initialize();
    }

    function onLayout(dc as Graphics.Dc) as Void {
        // We draw manually so no layout XML needed
        // If you have a MainLayout and want to keep it, leave this line:
        // setLayout(Rez.Layouts.MainLayout(dc));
    }

    function onShow() as Void {
        // Trigger a redraw when view becomes visible
        WatchUi.requestUpdate();
    }

    function onUpdate(dc as Graphics.Dc) as Void {
        // --- Background ---
        dc.setColor(Graphics.COLOR_BLACK, Graphics.COLOR_BLACK);
        dc.clear();

        var w = dc.getWidth();
        var h = dc.getHeight();
        var cx = w / 2;
        var cy = h / 2;

        // --- Title ---
        dc.setColor(Graphics.COLOR_WHITE, Graphics.COLOR_TRANSPARENT);
        dc.drawText(
            cx, 20,
            Graphics.FONT_TINY,
            "Lamp Control",
            Graphics.TEXT_JUSTIFY_CENTER
        );

        // --- BLE Status (large, center screen) ---
        var status = BLEManager.getStatus();
        var statusColor = BLEManager.getStatusColor();
        dc.setColor(statusColor, Graphics.COLOR_TRANSPARENT);
        dc.drawText(
            cx, cy - 20,
            Graphics.FONT_SMALL,
            status,
            Graphics.TEXT_JUSTIFY_CENTER
        );

        // --- Last action (smaller, below status) ---
        var lastAction = BLEManager.getLastAction();
        dc.setColor(Graphics.COLOR_LT_GRAY, Graphics.COLOR_TRANSPARENT);
        dc.drawText(
            cx, cy + 20,
            Graphics.FONT_TINY,
            lastAction,
            Graphics.TEXT_JUSTIFY_CENTER
        );

        // --- Hint at bottom ---
        dc.setColor(Graphics.COLOR_DK_GRAY, Graphics.COLOR_TRANSPARENT);
        dc.drawText(
            cx, h - 30,
            Graphics.FONT_TINY,
            "SET = Menu",
            Graphics.TEXT_JUSTIFY_CENTER
        );
    }

    function onHide() as Void {
    }
}
