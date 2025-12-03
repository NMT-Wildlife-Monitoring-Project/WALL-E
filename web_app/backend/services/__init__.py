"""
WALL-E Control Panel Backend - Services Package

Generated/Edited by Claude
"""

from .rosbridge_client import (
    ROSBridgeClient,
    get_rosbridge_client,
    init_rosbridge_client,
    shutdown_rosbridge_client,
)

__all__ = [
    "ROSBridgeClient",
    "get_rosbridge_client",
    "init_rosbridge_client",
    "shutdown_rosbridge_client",
]
