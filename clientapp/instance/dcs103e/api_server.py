"""
DCS-103E Flask Web API

This module provides a REST API for controlling DCS-103E light source devices.
It allows remote control of the light sources through HTTP requests.

API Endpoints:
    GET  /api/discover          - Discover all devices
    GET  /api/device/status     - Get device status
    POST /api/device/connect    - Connect to device
    POST /api/device/disconnect - Disconnect from device
    POST /api/channel/on        - Turn on channel
    POST /api/channel/off       - Turn off channel
    POST /api/channel/intensity - Set channel intensity
    GET  /api/channel/status    - Get channel status
"""

from flask import Flask, request, jsonify
from flask_cors import CORS
import logging
import traceback
from typing import Optional
from dcs103e_wrapper import DCS103E, discover_light_sources, DCSException


# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Flask application
app = Flask(__name__)
CORS(app)  # Enable CORS for all routes

# Global device connection
_device_connection: Optional[DCS103E] = None


def get_error_response(message: str, status_code: int = 400):
    """Create standardized error response"""
    return jsonify({"success": False, "error": message}), status_code


def get_success_response(data=None, message: str = "Success"):
    """Create standardized success response"""
    response = {"success": True, "message": message}
    if data is not None:
        response["data"] = data
    return jsonify(response)


@app.route("/api/discover", methods=["GET"])
def api_discover():
    """Discover all DCS devices on the network"""
    try:
        devices = discover_light_sources()
        device_list = [device.to_dict() for device in devices]

        logger.info(f"Discovered {len(device_list)} devices")
        return get_success_response(device_list, f"Found {len(device_list)} devices")

    except DCSException as e:
        logger.error(f"Discovery error: {e}")
        return get_error_response(f"Failed to discover devices: {e}")
    except Exception as e:
        logger.error(f"Unexpected discovery error: {e}")
        return get_error_response("Internal server error", 500)


@app.route("/api/device/connect", methods=["POST"])
def api_connect():
    """Connect to a DCS device"""
    global _device_connection

    try:
        data = request.get_json()
        if not data or "ip_address" not in data:
            return get_error_response("IP address is required")

        ip_address = data["ip_address"]

        # Disconnect existing connection if any
        if _device_connection:
            try:
                _device_connection.disconnect()
            except:
                pass
            _device_connection = None

        # Create new connection
        _device_connection = DCS103E()
        _device_connection.connect(ip_address)

        # Get device info
        device_status = _device_connection.get_device_status()

        logger.info(f"Connected to device at {ip_address}")
        return get_success_response(
            device_status, f"Connected to {device_status['name']}"
        )

    except DCSException as e:
        logger.error(f"Connection error: {e}")
        _device_connection = None
        return get_error_response(f"Failed to connect: {e}")
    except Exception as e:
        logger.error(f"Unexpected connection error: {e}")
        _device_connection = None
        return get_error_response("Internal server error", 500)


@app.route("/api/device/disconnect", methods=["POST"])
def api_disconnect():
    """Disconnect from the current device"""
    global _device_connection

    try:
        if not _device_connection:
            return get_error_response("No device connected")

        _device_connection.disconnect()
        _device_connection = None

        logger.info("Disconnected from device")
        return get_success_response(message="Disconnected successfully")

    except DCSException as e:
        logger.error(f"Disconnection error: {e}")
        return get_error_response(f"Failed to disconnect: {e}")
    except Exception as e:
        logger.error(f"Unexpected disconnection error: {e}")
        return get_error_response("Internal server error", 500)


@app.route("/api/device/status", methods=["GET"])
def api_device_status():
    """Get current device status"""
    try:
        if not _device_connection or not _device_connection.connected:
            return get_error_response("No device connected")

        status = _device_connection.get_device_status()
        return get_success_response(status)

    except DCSException as e:
        logger.error(f"Status error: {e}")
        return get_error_response(f"Failed to get device status: {e}")
    except Exception as e:
        logger.error(f"Unexpected status error: {e}")
        return get_error_response("Internal server error", 500)


@app.route("/api/channel/on", methods=["POST"])
def api_channel_on():
    """Turn on a channel"""
    try:
        if not _device_connection or not _device_connection.connected:
            return get_error_response("No device connected")

        data = request.get_json()
        if not data or "channel" not in data:
            return get_error_response("Channel number is required")

        channel = data["channel"]
        intensity = data.get("intensity", 100.0)

        if channel not in [1, 2, 3]:
            return get_error_response("Channel must be 1, 2, or 3")

        if not 0 <= intensity <= 100:
            return get_error_response("Intensity must be between 0 and 100")

        _device_connection.turn_on_channel(channel, intensity)
        channel_status = _device_connection.get_channel_status(channel)

        logger.info(f"Turned on channel {channel} at {intensity}%")
        return get_success_response(channel_status, f"Channel {channel} turned on")

    except DCSException as e:
        logger.error(f"Channel on error: {e}")
        return get_error_response(f"Failed to turn on channel: {e}")
    except Exception as e:
        logger.error(f"Unexpected channel on error: {e}")
        return get_error_response("Internal server error", 500)


@app.route("/api/channel/off", methods=["POST"])
def api_channel_off():
    """Turn off a channel"""
    try:
        if not _device_connection or not _device_connection.connected:
            return get_error_response("No device connected")

        data = request.get_json()
        if not data or "channel" not in data:
            return get_error_response("Channel number is required")

        channel = data["channel"]

        if channel not in [1, 2, 3]:
            return get_error_response("Channel must be 1, 2, or 3")

        _device_connection.turn_off_channel(channel)
        channel_status = _device_connection.get_channel_status(channel)

        logger.info(f"Turned off channel {channel}")
        return get_success_response(channel_status, f"Channel {channel} turned off")

    except DCSException as e:
        logger.error(f"Channel off error: {e}")
        return get_error_response(f"Failed to turn off channel: {e}")
    except Exception as e:
        logger.error(f"Unexpected channel off error: {e}")
        return get_error_response("Internal server error", 500)


@app.route("/api/channel/intensity", methods=["POST"])
def api_channel_intensity():
    """Set channel intensity"""
    try:
        if not _device_connection or not _device_connection.connected:
            return get_error_response("No device connected")

        data = request.get_json()
        if not data or "channel" not in data or "intensity" not in data:
            return get_error_response("Channel number and intensity are required")

        channel = data["channel"]
        intensity = data["intensity"]

        if channel not in [1, 2, 3]:
            return get_error_response("Channel must be 1, 2, or 3")

        if not 0 <= intensity <= 100:
            return get_error_response("Intensity must be between 0 and 100")

        _device_connection.set_channel_intensity(channel, intensity)
        channel_status = _device_connection.get_channel_status(channel)

        logger.info(f"Set channel {channel} intensity to {intensity}%")
        return get_success_response(
            channel_status, f"Channel {channel} intensity set to {intensity}%"
        )

    except DCSException as e:
        logger.error(f"Channel intensity error: {e}")
        return get_error_response(f"Failed to set channel intensity: {e}")
    except Exception as e:
        logger.error(f"Unexpected channel intensity error: {e}")
        return get_error_response("Internal server error", 500)


@app.route("/api/channel/status", methods=["GET"])
def api_channel_status():
    """Get channel status"""
    try:
        if not _device_connection or not _device_connection.connected:
            return get_error_response("No device connected")

        channel = request.args.get("channel", type=int)

        if channel is None:
            # Return all channels
            all_channels = {}
            for ch in range(1, 4):
                all_channels[ch] = _device_connection.get_channel_status(ch)
            return get_success_response(all_channels)

        if channel not in [1, 2, 3]:
            return get_error_response("Channel must be 1, 2, or 3")

        channel_status = _device_connection.get_channel_status(channel)
        return get_success_response(channel_status)

    except DCSException as e:
        logger.error(f"Channel status error: {e}")
        return get_error_response(f"Failed to get channel status: {e}")
    except Exception as e:
        logger.error(f"Unexpected channel status error: {e}")
        return get_error_response("Internal server error", 500)


@app.route("/api/channel/all/off", methods=["POST"])
def api_all_channels_off():
    """Turn off all channels"""
    try:
        if not _device_connection or not _device_connection.connected:
            return get_error_response("No device connected")

        _device_connection.turn_off_all_channels()
        device_status = _device_connection.get_device_status()

        logger.info("Turned off all channels")
        return get_success_response(device_status, "All channels turned off")

    except DCSException as e:
        logger.error(f"All channels off error: {e}")
        return get_error_response(f"Failed to turn off all channels: {e}")
    except Exception as e:
        logger.error(f"Unexpected all channels off error: {e}")
        return get_error_response("Internal server error", 500)


@app.route("/api/health", methods=["GET"])
def api_health():
    """Health check endpoint"""
    return get_success_response(
        {
            "server": "DCS-103E API Server",
            "version": "1.0.0",
            "connected": (
                _device_connection is not None and _device_connection.connected
                if _device_connection
                else False
            ),
        }
    )


@app.errorhandler(404)
def not_found(error):
    """Handle 404 errors"""
    return get_error_response("Endpoint not found", 404)


@app.errorhandler(500)
def internal_error(error):
    """Handle 500 errors"""
    logger.error(f"Internal server error: {error}")
    return get_error_response("Internal server error", 500)


def create_app():
    """Application factory function"""
    return app


if __name__ == "__main__":
    # Development server
    logger.info("Starting DCS-103E API Server")
    app.run(host="0.0.0.0", port=5001, debug=True)
