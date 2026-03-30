
import Toybox.System;
import Toybox.WatchUi;
import Toybox.Lang;

class Button_PressDelegate extends WatchUi.BehaviorDelegate {

    function initialize() {
        BehaviorDelegate.initialize();
    }

    // Back (lower-left on Instinct) — prevent exiting by consuming it
    function onBack() as Boolean {
        //System.println("[Delegate] Back pressed");
        return true; // consume to keep app running
    }

    // Select / SET — use this to open your menu
    function onSelect() as Boolean {
        System.println("[Delegate] Select/SET pressed — opening menu");
        WatchUi.pushView(
            new Rez.Menus.ServoMenu(),            // your menu resource
            new ServoMenuDelegate(),       // <-- use the correct class name
            WatchUi.SLIDE_UP
        );
        return true;
    }

    // Menu (short press only; long press is captured by the system)
    function onMenu() as Boolean {
        System.println("[Delegate] Menu short press (may not fire on Instinct)");
        // If you *also* want menu here, you can open it the same way:
        // WatchUi.pushView(new Rez.Menus.ServoMenu(), new Button_PressMenuDelegate(), WatchUi.SLIDE_UP);
        return true;
    }

    // Up/Down
    function onPrevious() as Boolean {
        System.println("[Delegate] Previous/Up");
        return true;
    }

    function onNext() as Boolean {
        System.println("[Delegate] Next/Down");
        return true;
    }
}
