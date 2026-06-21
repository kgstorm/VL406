# Remote Commissioning Design

## Purpose

This project is intended to support customer-installed hardware that must be observed and controlled remotely while the VL406 protocol is still being decoded.

## Transport Model

- Outbound MQTT over TLS for bidirectional command and response traffic
- Optional outbound webhook for boot notifications
- Outbound OTA firmware download for updates behind NAT

Both sides can be behind NAT because the ESP32 and Home Assistant each establish outbound connections to shared services.

## Initial MQTT Topic Contract

Use a shared command namespace for the fleet, and target a specific device in the payload when needed:

```text
spa/cmd/button
spa/cmd/raw_request
spa/cmd/reboot
spa/status/boot
spa/status/state
```

- Broadcast commands use plain payloads such as `warm`, `cool`, `light`, `pump`, or `raw_request`.
- Targeted commands prefix the payload with `target=<device_name>;`, for example `target=vl406-a4f00f5d46e0;warm`.
- Acknowledgements and raw-frame snapshots are published on `<device_name>/status/ack` and `<device_name>/status/raw`, where `<device_name>` is the ESPHome name with its MAC suffix.
- Boot messages include the runtime device name in the payload so Home Assistant can map the device to its per-device response topics.

## First Implementation Slice

1. Add MQTT connectivity and last-will state.
2. Add button command handler.
3. Add raw frame request handler that returns the latest valid frame.
4. Add acknowledgement payloads with command IDs.
5. Add OTA update command after the control loop is working.

## OTA Rules

- Firmware updates must use OTA only.
- Release workflow must not perform full flash erase.
- Wi-Fi credentials should remain in persistent storage across updates.
- Captive portal should only be needed for initial onboarding or explicit recovery.

## Data Capture Notes

- The copied decoder still assumes VL260 framing.
- Early VL406 builds should prefer raw frame capture over inferred state.
- The first stable milestone is remote button control plus on-demand raw frame retrieval.