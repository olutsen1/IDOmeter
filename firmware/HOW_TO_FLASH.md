# Flashing IDOmeter firmware 2.2

Built and checked against ESP32 Arduino core 3.3.12.

## 1. Arduino IDE (once)

1. **Boards Manager**: install or update **esp32 by Espressif Systems** to the latest 3.x.
2. **Library Manager**: install **Adafruit ST7735 and ST7789 Library** (accept
   "Install all" dependencies) and **SparkFun MAX3010x Pulse and Proximity
   Sensor Library**.

## 2. Upload

1. Open `IDometer_fw/IDometer_fw.ino`.
2. Board: **ESP32 Dev Module** (or "LilyGo T-Display"); select the COM port.
3. Click Upload. If it fails with "Failed to connect", hold the left button
   (next to USB) while upload starts.

## 3. Wi-Fi — no recompiling

The device remembers up to 5 networks and joins whichever is in range.

**From a phone:** power on while holding the **right button** (or wait 30 s
without a known network). The screen shows "WiFi SETUP". Join the Wi-Fi
**IDometer-setup** on your phone; the setup page opens (or browse to
**192.168.4.1**). Pick the network, enter the password, Save.

**From the serial monitor** (115200 baud): `wifi`, `list`, `forget N`,
`setup`, `status`, `help`.

The network must be 2.4 GHz and the same one the computer is on.

## 4. Screen

| Line | Meaning |
|---|---|
| WiFi: OK / WiFi: ... | connected (network name and device IP below) / searching |
| WiFi SETUP | phone setup mode |
| PC: linked / searching | the app on the computer found / not yet |
| Finger: OK / none | finger on the pulse sensor |
| GSR: number | raw skin signal |
| SD, Bat/USB, sent | backup recording, power, packets sent |

The **right button** turns the screen off and on, so it doesn't distract the client.

## 5. If "PC: searching" stays

Almost always the Windows firewall: set the network profile to **Private**
(Settings → Network & Internet → your connection) and allow `python.exe`
through the firewall (Win+R → `firewall.cpl` → "Allow an app…" → "Allow
another app…"). Alternatively put the computer's IP into `RECEIVER_IP` in `config.h`.

## Data

- No NTP or MQTT: the computer aligns the clocks to milliseconds.
- Hardware-timer sampling: PPG 100 Hz, skin 20 Hz, no averaging.
- Raw pulse wave is sent; heart rate is computed on the computer.
- Runs without network or sensor; backup to SD (`/idm/rec_NNNN.bin`) if a card is present.
