import Toybox.Lang;
import Toybox.System;
import Toybox.WatchUi;

class Button_PressMenuDelegate extends WatchUi.MenuInputDelegate {

    function initialize() {
        MenuInputDelegate.initialize();
    }

    function onMenuItem(item as Symbol) as Void {
        if (item == :servo_on) {
            // System.println("servo_on pressed");
            BLEManager.sendLampState(true);
        } else if (item == :servo_off) {
            // System.println("servo_off pressed");
            BLEManager.sendLampState(false);
        }
    }

}