# Remote Commissioning Design

## Purpose

This project is intended to support customer-installed hardware that must be observed and controlled remotely while the VL406 protocol is still being decoded.

## Transport Model

- Outbound MQTT over TLS for bidirectional command and response traffic
- Optional outbound webhook for boot notifications
- Outbound OTA firmware download for updates behind NAT

Both sides can be behind NAT because the ESP32 and Home Assistant each establish outbound connections to shared services.

## Initial MQTT Topic Contract

Use one device-specific namespace per board:

```text
spa/<device_id>/cmd/button
spa/<device_id>/cmd/raw_request
spa/<device_id>/cmd/update
spa/<device_id>/cmd/reboot
spa/<device_id>/status/ack
spa/<device_id>/status/raw
spa/<device_id>/status/boot
spa/<device_id>/status/update
spa/<device_id>/status/state
```

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