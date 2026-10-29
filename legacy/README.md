# Legacy (v0.8x, 2023–2025)

The original prototype firmware, kept for history. It sent averaged values with
NTP timestamps over MQTT to a Raspberry Pi (Mosquitto + Grafana).
It is superseded by `firmware/IDometer_fw` and `software/`.

- `IDOMTR_BROADCASTING_34_WORKVERSION.ino` — device firmware (TFT_eSPI, MAX30102, Grove GSR, SD, MQTT)
- `IDOMTR_DUMMY_MULTICORE.ino` — dummy signal generator used to set up Grafana

Credentials in these files are placeholders.
