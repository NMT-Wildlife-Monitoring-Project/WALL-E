"""
WALL-E Wildlife Robot Control Panel - FastAPI Backend

Main application entry point with WebSocket support, camera streaming,
waypoint management, and ROS integration.

Generated/Edited by Claude
"""

import asyncio
import logging
import os
import sys
from contextlib import asynccontextmanager
from typing import Any, Dict

from pathlib import Path

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, FileResponse
from fastapi.staticfiles import StaticFiles

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)

# Import routers
from routers import camera, robot, waypoints
from services.rosbridge_client import (
    get_rosbridge_client,
    init_rosbridge_client,
    shutdown_rosbridge_client,
)


# ============================================================================
# Application Lifespan
# ============================================================================

@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Application lifespan manager.
    Handles startup and shutdown events.
    """
    # Startup
    logger.info("Starting WALL-E Control Panel Backend...")

    # Initialize ROSBridge client
    rosbridge_uri = os.environ.get("ROSBRIDGE_URI", "ws://localhost:9090")
    logger.info(f"Connecting to ROSBridge at {rosbridge_uri}")

    try:
        await init_rosbridge_client(rosbridge_uri)
    except Exception as e:
        logger.warning(f"Could not connect to ROSBridge: {e}")
        logger.warning("Robot functionality will be limited until ROSBridge is available")

    # Ensure shared directory exists for waypoints
    os.makedirs("/tmp/shared", exist_ok=True)

    logger.info("Backend startup complete")

    yield

    # Shutdown
    logger.info("Shutting down WALL-E Control Panel Backend...")

    # Release camera
    from routers.camera import release_camera
    await release_camera()

    # Disconnect from ROSBridge
    await shutdown_rosbridge_client()

    logger.info("Backend shutdown complete")


# ============================================================================
# FastAPI Application
# ============================================================================

app = FastAPI(
    title="WALL-E Wildlife Robot Control Panel",
    description="Backend API for controlling and monitoring the WALL-E wildlife observation robot",
    version="1.0.0",
    lifespan=lifespan,
    docs_url="/docs",
    redoc_url="/redoc",
)


# ============================================================================
# CORS Middleware
# ============================================================================

# Configure CORS for React frontend
cors_origins = os.environ.get("CORS_ORIGINS", "http://localhost:3000,http://localhost:5173").split(",")

app.add_middleware(
    CORSMiddleware,
    allow_origins=cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    expose_headers=["*"],
)


# ============================================================================
# Exception Handlers
# ============================================================================

@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    """Global exception handler for unhandled errors."""
    logger.error(f"Unhandled exception: {exc}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={
            "detail": "Internal server error",
            "type": type(exc).__name__,
        }
    )


# ============================================================================
# Include Routers
# ============================================================================

app.include_router(waypoints.router)
app.include_router(robot.router)
app.include_router(camera.router)


# ============================================================================
# Health Check and Root Endpoints
# ============================================================================

@app.get("/", tags=["root"])
async def root():
    """Root endpoint with API information."""
    return {
        "name": "WALL-E Wildlife Robot Control Panel",
        "version": "1.0.0",
        "status": "running",
        "docs": "/docs",
        "health": "/health",
    }


@app.get("/health", tags=["health"])
async def health_check():
    """
    Health check endpoint.

    Returns the health status of the backend and its connections.
    """
    rosbridge = get_rosbridge_client()

    # Check camera availability
    from routers.camera import get_camera
    camera = await get_camera()
    camera_available = camera is not None and camera.isOpened()

    # Check waypoints file accessibility
    waypoints_accessible = os.access("/tmp/shared", os.W_OK)

    return {
        "status": "healthy",
        "services": {
            "rosbridge": {
                "connected": rosbridge.connected if rosbridge else False,
                "uri": rosbridge.uri if rosbridge else None,
            },
            "camera": {
                "available": camera_available,
            },
            "waypoints": {
                "accessible": waypoints_accessible,
                "path": "/tmp/shared/waypoints.yaml",
            },
        },
    }


@app.get("/api/status", tags=["status"])
async def api_status():
    """
    Get overall API status and configuration.
    """
    rosbridge = get_rosbridge_client()

    return {
        "api_version": "1.0.0",
        "rosbridge_uri": os.environ.get("ROSBRIDGE_URI", "ws://localhost:9090"),
        "rosbridge_connected": rosbridge.connected if rosbridge else False,
        "cors_origins": cors_origins,
        "endpoints": {
            "waypoints": "/api/waypoints",
            "robot_stats": "/api/robot/stats",
            "navigation_goal": "/api/navigation/goal",
            "camera_stream": "/api/camera/stream",
            "websocket_ros": "/api/ws/ros",
        },
    }


# ============================================================================
# Static File Serving (Production Mode)
# ============================================================================

# Serve React frontend static files in production
FRONTEND_DIR = Path(__file__).parent.parent / "frontend" / "dist"

if FRONTEND_DIR.exists():
    logger.info(f"Serving frontend static files from {FRONTEND_DIR}")

    # Mount static assets
    app.mount("/assets", StaticFiles(directory=FRONTEND_DIR / "assets"), name="assets")

    # Catch-all route for React SPA (must be after API routes)
    @app.get("/{full_path:path}", include_in_schema=False)
    async def serve_spa(full_path: str):
        """Serve React SPA for all non-API routes."""
        # Don't serve for API routes
        if full_path.startswith("api/") or full_path in ["docs", "redoc", "openapi.json", "health"]:
            return JSONResponse(status_code=404, content={"detail": "Not found"})

        # Serve index.html for all other routes (React Router handles routing)
        index_path = FRONTEND_DIR / "index.html"
        if index_path.exists():
            return FileResponse(index_path)
        return JSONResponse(status_code=404, content={"detail": "Frontend not built"})
else:
    logger.warning(f"Frontend dist directory not found at {FRONTEND_DIR}")
    logger.warning("Run 'npm run build' in frontend/ to build the frontend")


# ============================================================================
# Main Entry Point
# ============================================================================

if __name__ == "__main__":
    import uvicorn

    host = os.environ.get("HOST", "0.0.0.0")
    port = int(os.environ.get("PORT", "8000"))

    logger.info(f"Starting server on {host}:{port}")

    uvicorn.run(
        "main:app",
        host=host,
        port=port,
        reload=os.environ.get("DEBUG", "false").lower() == "true",
        log_level="info",
    )
