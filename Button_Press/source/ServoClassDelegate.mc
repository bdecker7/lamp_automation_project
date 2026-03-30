
import Toybox.Lang;
import Toybox.WatchUi;
import Toybox.System;
import Toybox.BluetoothLowEnergy;

class ServoMenuDelegate extends WatchUi.Menu2InputDelegate {
    function initialize() { Menu2InputDelegate.initialize(); }

    // Called when the user presses GPS/Select on a Menu2 item
    function onSelect(item as WatchUi.MenuItem) as Void {
        var id = item.getId(); // e.g., "servo_on"
        System.println("[Menu2] onSelect: " + id);

        if (id == :servo_on) {
            BLEManager.sendLampState(true);   // turn lamp ON
            BLEManager._setAction("Sent ON!");
        } else if (id == :servo_off) {
            BLEManager.sendLampState(false);  // turn lamp OFF
            BLEManager._setAction("Sent OFF!");
        }else {
            BLEManager._setAction("Unknown: " + id);
        }
        WatchUi.popView(WatchUi.SLIDE_IMMEDIATE); 
        //tests for what id is being sent
        // var id = item.getId();
        // System.println("[Menu2] onSelect fired! id=" + id);
        // BLEManager._setStatus("onSelect!", Graphics.COLOR_ORANGE);
        // BLEManager._setAction("id=" + id);
        // WatchUi.popView(WatchUi.SLIDE_DOWN);
    }

    // Optional: handle Back to avoid auto-pop
    function onBack() as Void {
        System.println("[Menu2] Back");
        WatchUi.popView(WatchUi.SLIDE_UP);
    }

}
