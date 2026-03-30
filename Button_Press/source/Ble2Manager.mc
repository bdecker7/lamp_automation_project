// source/BLEManager.mc
// Updated with on-screen status tracking.
// The view calls BLEManager.getStatus() and BLEManager.getLastAction()
// on every frame to show what is happening in real time on the watch face.

import Toybox.System;
import Toybox.Lang;
import Toybox.BluetoothLowEnergy;
import Toybox.Application;
import Toybox.Graphics;
import Toybox.WatchUi;

module BLEManager {

    // UUIDs — must match ESP32 exactly
    const LAMP_SERVICE_UUID = BluetoothLowEnergy.stringToUuid("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
    const LAMP_CHAR_UUID    = BluetoothLowEnergy.stringToUuid("beb5483e-36e1-4688-b7f5-ea07361b26a8");

    // BLE handles
    var _device    as BluetoothLowEnergy.Device?         = null;
    var _service   as BluetoothLowEnergy.Service?        = null;
    var _charLamp  as BluetoothLowEnergy.Characteristic? = null;

    // Status strings shown on screen
    var _status     as Lang.String = "Scanning...";
    var _lastAction as Lang.String = "Waiting...";
    var _statusColor as Graphics.ColorType = Graphics.COLOR_YELLOW;

    // --- Called by the view on every frame ---
    function getStatus() as Lang.String {
        return _status;
    }

    function getLastAction() as Lang.String {
        return _lastAction;
    }

    function getStatusColor() as Graphics.ColorType {
        return _statusColor;
    }

    // Internal helper to update status and request screen refresh
    function _setStatus(msg as Lang.String, color as Graphics.ColorType) as Void {
        _status = msg;
        _statusColor = color;
        System.println("[BLE] STATUS: " + msg);
        WatchUi.requestUpdate(); // redraw the view
    }

    function _setAction(msg as Lang.String) as Void {
        _lastAction = msg;
        System.println("[BLE] ACTION: " + msg);
        WatchUi.requestUpdate();
    }

    // =========================================================
    // BLE Delegate
    // =========================================================
    class Delegate extends BluetoothLowEnergy.BleDelegate {

        function initialize() {
            BluetoothLowEnergy.BleDelegate.initialize();
        }

        function onProfileRegister(uuid as BluetoothLowEnergy.Uuid, status as BluetoothLowEnergy.Status) as Void {
            System.println("[BLE] Profile registered: " + uuid + " status=" + status);
            if (status == BluetoothLowEnergy.STATUS_SUCCESS) {
                _setAction("Profile OK");
            } else {
                _setAction("Profile FAIL:" + status);
            }
        }

        function onScanStateChange(scanState as BluetoothLowEnergy.ScanState, status as BluetoothLowEnergy.Status) as Void {
            System.println("[BLE] Scan state=" + scanState + " status=" + status);
            if (scanState == BluetoothLowEnergy.SCAN_STATE_SCANNING) {
                _setStatus("Scanning...", Graphics.COLOR_YELLOW);
            } else {
                _setStatus("Scan Off", Graphics.COLOR_DK_GRAY);
            }
        }

        function onScanResults(scanResults as BluetoothLowEnergy.Iterator) as Void {
            var sr = null;
            while (true) {
                sr = scanResults.next() as BluetoothLowEnergy.ScanResult?;
                if (sr == null) { break; }

                var uuidsIter = sr.getServiceUuids();
                var match = false;
                var u = null;

                while (true) {
                    u = uuidsIter.next() as BluetoothLowEnergy.Uuid?;
                    if (u == null) { break; }
                    if (u.equals(LAMP_SERVICE_UUID)) {
                        match = true;
                        break;
                    }
                }

                if (match) {
                    System.println("[BLE] Found ESP32: " + sr.getDeviceName());
                    _setStatus("Found ESP32", Graphics.COLOR_BLUE);
                    _setAction("Pairing...");

                    BluetoothLowEnergy.setScanState(BluetoothLowEnergy.SCAN_STATE_OFF);
                    try {
                        _device = BluetoothLowEnergy.pairDevice(sr);
                    } catch (e) {
                        System.println("[BLE] pairDevice failed: " + e);
                        _setStatus("Pair FAIL", Graphics.COLOR_RED);
                        _setAction(e.getErrorMessage());
                    }
                    break;
                }
            }
        }

        function onConnectedStateChanged(dev as BluetoothLowEnergy.Device, state as BluetoothLowEnergy.ConnectionState) as Void {
            System.println("[BLE] Connection state: " + state);

            if (state == BluetoothLowEnergy.CONNECTION_STATE_CONNECTED) {
                _device = dev;
                _setStatus("Connected!", Graphics.COLOR_GREEN);
                _setAction("Getting service...");

                // Step 1: Find the service
                _service = dev.getService(LAMP_SERVICE_UUID);
                if (_service == null) {
                    _setStatus("Svc NULL!", Graphics.COLOR_RED);
                    _setAction("UUID mismatch?");
                    System.println("[BLE] ERROR: Service is NULL");
                    return;
                }
                _setAction("Svc OK - getting char...");
                System.println("[BLE] Service found OK");

                // Step 2: Find the characteristic
                _charLamp = _service.getCharacteristic(LAMP_CHAR_UUID);
                if (_charLamp == null) {
                    _setStatus("Char NULL!", Graphics.COLOR_RED);
                    _setAction("Char not found");
                    System.println("[BLE] ERROR: Characteristic is NULL");
                    return;
                }
                _setStatus("Char Ready!", Graphics.COLOR_GREEN);
                _setAction("Press SET to send");
                System.println("[BLE] Characteristic found OK - ready!");

                // Step 3: Try to enable notifications (optional)
                var cccd = _charLamp.getDescriptor(BluetoothLowEnergy.cccdUuid());
                if (cccd != null) {
                    try {
                        cccd.requestWrite([0x01, 0x00]b);
                        System.println("[BLE] Notifications enabled");
                    } catch (e) {
                        System.println("[BLE] CCCD write failed: " + e);
                    }
                } else {
                    System.println("[BLE] No CCCD - notifications not available");
                }

            } else if (state == BluetoothLowEnergy.CONNECTION_STATE_DISCONNECTED) {
                System.println("[BLE] Disconnected");
                _device   = null;
                _service  = null;
                _charLamp = null;
                _setStatus("Disconnected", Graphics.COLOR_RED);
                _setAction("Restarting scan...");

                // Restart scanning automatically
                BluetoothLowEnergy.setScanState(BluetoothLowEnergy.SCAN_STATE_SCANNING);
            }
        }

        function onEncryptionStatus(dev as BluetoothLowEnergy.Device, status as BluetoothLowEnergy.Status) as Void {
            System.println("[BLE] Encryption status: " + status);
        }

        function onCharacteristicWrite(ch as BluetoothLowEnergy.Characteristic, status as BluetoothLowEnergy.Status) as Void {
            System.println("[BLE] Write complete: status=" + status);
            if (status == BluetoothLowEnergy.STATUS_SUCCESS) {
                _setAction("Write OK!");
            } else {
                _setAction("Write FAIL:" + status);
            }
        }

        function onCharacteristicChanged(ch as BluetoothLowEnergy.Characteristic, value as Lang.ByteArray) as Void {
            System.println("[BLE] Notified value: " + value);
        }
    }

    // =========================================================
    // Public: register profile and start scanning
    // =========================================================
    function initAndScan() as Void {
        _setStatus("Scanning...", Graphics.COLOR_YELLOW);
        _setAction("Starting BLE...");

        BluetoothLowEnergy.setDelegate(new Delegate());

        var profile = {
            :uuid => LAMP_SERVICE_UUID,
            :characteristics => [
                { :uuid => LAMP_CHAR_UUID, :descriptors => [] }
            ]
        };

        BluetoothLowEnergy.registerProfile(profile);
        BluetoothLowEnergy.setScanState(BluetoothLowEnergy.SCAN_STATE_SCANNING);
    }

    // =========================================================
    // Public: send ON or OFF to ESP32
    // =========================================================
    function sendLampState(on as Lang.Boolean) as Void {
        System.println("[BLE] sendLampState called on=" + on + " charReady=" + (_charLamp != null));

        if (_charLamp == null) {
            _setStatus("Char NULL!", Graphics.COLOR_RED);
            _setAction("Not ready yet");
            System.println("[BLE] Lamp characteristic not ready.");
            return;
        }

        var payload = on ? [0x01]b : [0x00]b;
        if (on) {
            _setAction("Sending ON...");
        } else {
            _setAction("Sending OFF...");
        }

        try {
            _charLamp.requestWrite(payload, { :writeType => BluetoothLowEnergy.WRITE_TYPE_WITH_RESPONSE });
            System.println("[BLE] requestWrite sent!");
        } catch (e) {
            System.println("[BLE] requestWrite failed: " + e);
            _setStatus("Send FAIL", Graphics.COLOR_RED);
            _setAction(e.getErrorMessage());
        }
    }
}
