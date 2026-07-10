// lib/serial_helper.dart

import 'dart:async';
import 'dart:io';
import 'dart:convert';
import 'package:flutter/foundation.dart';

class SerialHelper {
  Socket? _socket;

  // The Pi's local port forwarded via ADB reverse
  static const String _piIpAddress = "127.0.0.1";
  static const int _port = 5000;

  // Callback to feed the Pi's absolute fused truth back into Flutter's mapping logic
  Function(double x, double y, double yaw)? onFusedPositionData;

  bool get isConnected => _socket != null;

  // ─── Connection over ADB Tunnel ──────────────────────────────────────────

  Future<bool> connectToPi() async {
    try {
      _socket = await Socket.connect(_piIpAddress, _port, timeout: const Duration(seconds: 3));
      debugPrint("Connected to Pi TCP Server over USB!");
      _startPacketListener();
      return true;
    } catch (e) {
      debugPrint("Network connection to Pi failed: $e");
      _socket = null;
      return false;
    }
  }

  void disconnect() {
    _socket?.destroy();
    _socket = null;
  }

  // ─── Instant Event-Driven Transmission ────────────────────────────────────

  void sendTelemetry({required String direction, required double x, required double y, required double yaw, required bool collisionRisk}) {
    if (!isConnected) return;

    String colFlag = collisionRisk ? '1' : '0';
    final String payload = "NAV,$direction,${x.toStringAsFixed(3)},${y.toStringAsFixed(3)},${yaw.toStringAsFixed(3)},$colFlag\n";

    try {
      _socket!.write(payload);
      _socket!.flush(); // Push instantly
    } catch (e) {
      debugPrint("Socket write error (Likely Disconnected): $e");
      disconnect();
    }
  }

  void sendTargetedCommand(String target, String command) {
    if (!isConnected) return;
    _socket!.write("$target:$command\n");
  }

  // ─── Incoming Packet Listener ─────────────────────────────────────────────

  void _startPacketListener() {
    _socket!.cast<List<int>>().transform(utf8.decoder).transform(const LineSplitter()).listen(
          (String line) {
        String completedPacket = line.trim();
        if (completedPacket.isNotEmpty) {

          // Fused Truth Payload extraction
          if (completedPacket.startsWith("FUSED,")) {
            var parts = completedPacket.split(",");
            if (parts.length >= 4) {
              double? fx = double.tryParse(parts[1]);
              double? fy = double.tryParse(parts[2]);
              double? fyaw = double.tryParse(parts[3]);

              if (fx != null && fy != null && fyaw != null) {
                onFusedPositionData?.call(fx, fy, fyaw);
              }
            }
          } else {
            debugPrint("Received from Pi: $completedPacket");
          }
        }
      },
      onError: (err) {
        debugPrint("Socket stream error: $err");
        disconnect();
      },
      onDone: () {
        debugPrint("Server closed connection.");
        disconnect();
      },
    );
  }
}