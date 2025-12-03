"""
Camera Router
Handles camera streaming via MJPEG proxy

Generated/Edited by Claude
"""

import asyncio
import logging
import time
from typing import Optional

import cv2
from fastapi import APIRouter, HTTPException, Response
from fastapi.responses import StreamingResponse

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/camera", tags=["camera"])


# ============================================================================
# Camera Configuration
# ============================================================================

class CameraConfig:
    """Camera configuration and state."""
    device: str = "/dev/video0"
    width: int = 640
    height: int = 480
    fps: int = 30
    jpeg_quality: int = 80


config = CameraConfig()

# Global camera instance with improved thread safety
_camera: Optional[cv2.VideoCapture] = None
_camera_lock = asyncio.Lock()
_camera_initializing = False  # Prevents concurrent initialization attempts


# ============================================================================
# Camera Management
# ============================================================================

async def get_camera() -> Optional[cv2.VideoCapture]:
    """
    Get or initialize the camera instance.

    Fixed: Added initialization flag to prevent race conditions when multiple
    requests attempt to initialize the camera simultaneously.
    """
    global _camera, _camera_initializing

    # Fast path: camera already initialized
    if _camera is not None and _camera.isOpened():
        return _camera

    async with _camera_lock:
        # Double-check after acquiring lock (another request may have initialized)
        if _camera is not None and _camera.isOpened():
            return _camera

        # Prevent concurrent initialization
        if _camera_initializing:
            logger.debug("Camera initialization already in progress, waiting...")
            return None

        _camera_initializing = True

        try:
            # Try to open the camera device
            _camera = cv2.VideoCapture(config.device)

            if not _camera.isOpened():
                # Try numeric device index as fallback
                _camera = cv2.VideoCapture(0)

            if not _camera.isOpened():
                logger.error(f"Failed to open camera at {config.device}")
                _camera = None
                return None

            # Configure camera
            _camera.set(cv2.CAP_PROP_FRAME_WIDTH, config.width)
            _camera.set(cv2.CAP_PROP_FRAME_HEIGHT, config.height)
            _camera.set(cv2.CAP_PROP_FPS, config.fps)
            _camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize latency

            logger.info(f"Camera opened: {config.device} ({config.width}x{config.height}@{config.fps}fps)")
            return _camera

        except Exception as e:
            logger.error(f"Error initializing camera: {e}")
            _camera = None
            return None

        finally:
            _camera_initializing = False


async def release_camera():
    """Release the camera resource."""
    global _camera, _camera_initializing

    async with _camera_lock:
        _camera_initializing = False  # Reset initialization flag
        if _camera is not None:
            _camera.release()
            _camera = None
            logger.info("Camera released")


def encode_frame_jpeg(frame, quality: int = 80) -> bytes:
    """Encode a frame as JPEG."""
    encode_params = [cv2.IMWRITE_JPEG_QUALITY, quality]
    success, encoded = cv2.imencode(".jpg", frame, encode_params)
    if not success:
        raise RuntimeError("Failed to encode frame")
    return encoded.tobytes()


# ============================================================================
# Streaming Generators
# ============================================================================

async def mjpeg_generator():
    """
    Generate MJPEG stream frames.

    Yields multipart JPEG frames for MJPEG streaming.
    """
    camera = await get_camera()

    if camera is None:
        # Yield a placeholder frame or error image
        logger.warning("Camera not available, yielding placeholder")
        placeholder = create_placeholder_frame("Camera Not Available")
        encoded = encode_frame_jpeg(placeholder, config.jpeg_quality)
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + encoded + b"\r\n"
        )
        return

    frame_interval = 1.0 / config.fps
    last_frame_time = 0

    try:
        while True:
            # Rate limiting
            current_time = time.time()
            elapsed = current_time - last_frame_time
            if elapsed < frame_interval:
                await asyncio.sleep(frame_interval - elapsed)

            # Capture frame
            success, frame = camera.read()

            if not success:
                logger.warning("Failed to read frame from camera")
                # Try to reconnect
                await release_camera()
                camera = await get_camera()
                if camera is None:
                    await asyncio.sleep(1.0)
                    continue
                continue

            last_frame_time = time.time()

            # Encode frame
            try:
                encoded = encode_frame_jpeg(frame, config.jpeg_quality)
            except Exception as e:
                logger.error(f"Failed to encode frame: {e}")
                continue

            # Yield MJPEG frame
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + encoded + b"\r\n"
            )

    except asyncio.CancelledError:
        logger.info("MJPEG stream cancelled")
        raise
    except Exception as e:
        logger.error(f"Error in MJPEG generator: {e}")
        raise


def create_placeholder_frame(text: str = "No Signal") -> any:
    """Create a placeholder frame with text."""
    import numpy as np

    # Create black frame
    frame = np.zeros((config.height, config.width, 3), dtype=np.uint8)

    # Add text
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1.0
    thickness = 2
    color = (255, 255, 255)

    # Get text size for centering
    (text_width, text_height), _ = cv2.getTextSize(text, font, font_scale, thickness)
    x = (config.width - text_width) // 2
    y = (config.height + text_height) // 2

    cv2.putText(frame, text, (x, y), font, font_scale, color, thickness)

    return frame


# ============================================================================
# REST Endpoints
# ============================================================================

@router.get("/stream")
async def camera_stream():
    """
    MJPEG camera stream endpoint.

    Returns a multipart MJPEG stream from /dev/video0.
    Connect to this endpoint to view live camera feed.

    Example usage in HTML:
        <img src="/api/camera/stream" />
    """
    return StreamingResponse(
        mjpeg_generator(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )


@router.get("/snapshot")
async def camera_snapshot():
    """
    Get a single JPEG snapshot from the camera.

    Returns a single frame as a JPEG image.
    """
    camera = await get_camera()

    if camera is None:
        # Return placeholder
        placeholder = create_placeholder_frame("Camera Not Available")
        encoded = encode_frame_jpeg(placeholder, config.jpeg_quality)
        return Response(content=encoded, media_type="image/jpeg")

    success, frame = camera.read()

    if not success:
        raise HTTPException(
            status_code=500,
            detail="Failed to capture frame from camera"
        )

    encoded = encode_frame_jpeg(frame, config.jpeg_quality)
    return Response(content=encoded, media_type="image/jpeg")


@router.get("/status")
async def camera_status():
    """
    Get camera status and configuration.
    """
    camera = await get_camera()
    is_available = camera is not None and camera.isOpened()

    actual_width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH)) if is_available else 0
    actual_height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT)) if is_available else 0
    actual_fps = camera.get(cv2.CAP_PROP_FPS) if is_available else 0

    return {
        "available": is_available,
        "device": config.device,
        "configured": {
            "width": config.width,
            "height": config.height,
            "fps": config.fps,
            "jpeg_quality": config.jpeg_quality
        },
        "actual": {
            "width": actual_width,
            "height": actual_height,
            "fps": actual_fps
        }
    }


@router.post("/configure")
async def configure_camera(
    width: Optional[int] = None,
    height: Optional[int] = None,
    fps: Optional[int] = None,
    jpeg_quality: Optional[int] = None
):
    """
    Configure camera settings.

    Changes take effect on next stream start.
    """
    if width is not None:
        if width < 160 or width > 1920:
            raise HTTPException(status_code=400, detail="Width must be between 160 and 1920")
        config.width = width

    if height is not None:
        if height < 120 or height > 1080:
            raise HTTPException(status_code=400, detail="Height must be between 120 and 1080")
        config.height = height

    if fps is not None:
        if fps < 1 or fps > 60:
            raise HTTPException(status_code=400, detail="FPS must be between 1 and 60")
        config.fps = fps

    if jpeg_quality is not None:
        if jpeg_quality < 1 or jpeg_quality > 100:
            raise HTTPException(status_code=400, detail="JPEG quality must be between 1 and 100")
        config.jpeg_quality = jpeg_quality

    # Release camera so new settings take effect
    await release_camera()

    logger.info(f"Camera configured: {config.width}x{config.height}@{config.fps}fps, quality={config.jpeg_quality}")

    return {
        "message": "Camera configuration updated",
        "config": {
            "width": config.width,
            "height": config.height,
            "fps": config.fps,
            "jpeg_quality": config.jpeg_quality
        }
    }


@router.post("/restart")
async def restart_camera():
    """
    Restart the camera.

    Releases and reinitializes the camera device.
    """
    await release_camera()
    camera = await get_camera()

    if camera is None:
        raise HTTPException(
            status_code=500,
            detail="Failed to restart camera"
        )

    return {"message": "Camera restarted successfully"}
