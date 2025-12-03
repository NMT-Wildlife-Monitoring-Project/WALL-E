/**
 * CostmapViewer Component - Leaflet.js map for costmap visualization
 * Generated/Edited by Claude
 */
import React, { useEffect, useMemo, useRef, useState } from 'react';
import {
  MapContainer,
  TileLayer,
  ImageOverlay,
  Marker,
  Circle,
  useMap,
} from 'react-leaflet';
import L from 'leaflet';
import { useRobotStore, CostmapData } from '../services/rosbridge';

// Fix for default marker icons in Leaflet with Vite
import markerIcon2x from 'leaflet/dist/images/marker-icon-2x.png';
import markerIcon from 'leaflet/dist/images/marker-icon.png';
import markerShadow from 'leaflet/dist/images/marker-shadow.png';

// Configure default icon
const DefaultIcon = L.icon({
  iconUrl: markerIcon,
  iconRetinaUrl: markerIcon2x,
  shadowUrl: markerShadow,
  iconSize: [25, 41],
  iconAnchor: [12, 41],
  popupAnchor: [1, -34],
  tooltipAnchor: [16, -28],
  shadowSize: [41, 41],
});

L.Marker.prototype.options.icon = DefaultIcon;

// Custom robot icon
const robotIcon = L.divIcon({
  className: 'robot-marker',
  html: `<div style="
    width: 20px;
    height: 20px;
    background-color: #3fb950;
    border: 3px solid white;
    border-radius: 50%;
    box-shadow: 0 0 10px rgba(63, 185, 80, 0.5);
  "></div>`,
  iconSize: [20, 20],
  iconAnchor: [10, 10],
});

// Component to update map view when robot position changes
interface MapUpdaterProps {
  center: [number, number] | null;
  follow: boolean;
}

const MapUpdater: React.FC<MapUpdaterProps> = ({ center, follow }) => {
  const map = useMap();

  useEffect(() => {
    if (center && follow) {
      map.setView(center, map.getZoom());
    }
  }, [map, center, follow]);

  return null;
};

// Convert costmap data to image URL
const costmapToImageUrl = (costmap: CostmapData): string => {
  const canvas = document.createElement('canvas');
  canvas.width = costmap.width;
  canvas.height = costmap.height;
  const ctx = canvas.getContext('2d');

  if (!ctx) return '';

  const imageData = ctx.createImageData(costmap.width, costmap.height);

  for (let i = 0; i < costmap.data.length; i++) {
    const value = costmap.data[i];
    const pixelIndex = i * 4;

    // Color mapping for costmap values
    // -1 (unknown) = gray, 0 (free) = transparent, 100 (occupied) = red
    // Values in between are gradient
    if (value === -1 || value === 255) {
      // Unknown
      imageData.data[pixelIndex] = 128;
      imageData.data[pixelIndex + 1] = 128;
      imageData.data[pixelIndex + 2] = 128;
      imageData.data[pixelIndex + 3] = 100;
    } else if (value === 0) {
      // Free space
      imageData.data[pixelIndex] = 0;
      imageData.data[pixelIndex + 1] = 255;
      imageData.data[pixelIndex + 2] = 0;
      imageData.data[pixelIndex + 3] = 30;
    } else if (value >= 100) {
      // Lethal obstacle
      imageData.data[pixelIndex] = 255;
      imageData.data[pixelIndex + 1] = 0;
      imageData.data[pixelIndex + 2] = 0;
      imageData.data[pixelIndex + 3] = 200;
    } else {
      // Gradient for costs
      const intensity = Math.floor((value / 100) * 255);
      imageData.data[pixelIndex] = 255;
      imageData.data[pixelIndex + 1] = 255 - intensity;
      imageData.data[pixelIndex + 2] = 0;
      imageData.data[pixelIndex + 3] = Math.floor(50 + (value / 100) * 150);
    }
  }

  ctx.putImageData(imageData, 0, 0);
  return canvas.toDataURL();
};

interface CostmapViewerProps {
  initialCenter?: [number, number];
  initialZoom?: number;
}

const CostmapViewer: React.FC<CostmapViewerProps> = ({
  initialCenter = [37.7749, -122.4194],
  initialZoom = 18,
}) => {
  const { stats, costmap } = useRobotStore();
  const [followRobot, setFollowRobot] = useState(true);
  const [showCostmap, setShowCostmap] = useState(true);
  const [costmapOpacity, setCostmapOpacity] = useState(0.7);
  const mapRef = useRef<L.Map | null>(null);

  // Robot position
  const robotPosition: [number, number] | null = useMemo(() => {
    if (stats.gpsLat !== 0 || stats.gpsLon !== 0) {
      return [stats.gpsLat, stats.gpsLon];
    }
    return null;
  }, [stats.gpsLat, stats.gpsLon]);

  // Calculate costmap bounds
  const costmapBounds = useMemo((): L.LatLngBoundsExpression | null => {
    if (!costmap || !robotPosition) return null;

    const halfWidth = (costmap.width * costmap.resolution) / 2;
    const halfHeight = (costmap.height * costmap.resolution) / 2;

    // Convert meters to approximate lat/lng degrees
    const latDelta = halfHeight / 111320; // meters per degree latitude
    const lngDelta = halfWidth / (111320 * Math.cos(robotPosition[0] * (Math.PI / 180)));

    return [
      [robotPosition[0] - latDelta, robotPosition[1] - lngDelta],
      [robotPosition[0] + latDelta, robotPosition[1] + lngDelta],
    ];
  }, [costmap, robotPosition]);

  // Generate costmap image URL
  const costmapImageUrl = useMemo(() => {
    if (!costmap || !showCostmap) return null;
    return costmapToImageUrl(costmap);
  }, [costmap, showCostmap]);

  return (
    <div className="map-container" style={{ position: 'relative' }}>
      <MapContainer
        center={robotPosition || initialCenter}
        zoom={initialZoom}
        style={{ width: '100%', height: '100%' }}
        ref={mapRef}
      >
        {/* Dark theme tile layer */}
        <TileLayer
          attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>'
          url="https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png"
        />

        {/* Map updater for following robot */}
        <MapUpdater center={robotPosition} follow={followRobot} />

        {/* Costmap overlay */}
        {costmapImageUrl && costmapBounds && (
          <ImageOverlay
            url={costmapImageUrl}
            bounds={costmapBounds}
            opacity={costmapOpacity}
          />
        )}

        {/* Robot marker */}
        {robotPosition && (
          <>
            <Marker position={robotPosition} icon={robotIcon} />
            {/* Heading indicator */}
            <Circle
              center={robotPosition}
              radius={2}
              pathOptions={{
                color: '#3fb950',
                fillColor: '#3fb950',
                fillOpacity: 0.3,
              }}
            />
          </>
        )}
      </MapContainer>

      {/* Costmap controls */}
      <div className="costmap-controls">
        <label className="control-checkbox">
          <input
            type="checkbox"
            checked={followRobot}
            onChange={(e) => setFollowRobot(e.target.checked)}
          />
          <span>Follow Robot</span>
        </label>

        <label className="control-checkbox">
          <input
            type="checkbox"
            checked={showCostmap}
            onChange={(e) => setShowCostmap(e.target.checked)}
          />
          <span>Show Costmap</span>
        </label>

        {showCostmap && (
          <label className="control-slider">
            <span>Opacity</span>
            <input
              type="range"
              min="0"
              max="1"
              step="0.1"
              value={costmapOpacity}
              onChange={(e) => setCostmapOpacity(parseFloat(e.target.value))}
            />
          </label>
        )}
      </div>

      {/* Legend */}
      <div className="costmap-legend">
        <div className="legend-item">
          <span className="legend-color" style={{ backgroundColor: 'rgba(0, 255, 0, 0.3)' }}></span>
          <span>Free</span>
        </div>
        <div className="legend-item">
          <span className="legend-color" style={{ backgroundColor: 'rgba(255, 255, 0, 0.7)' }}></span>
          <span>Low Cost</span>
        </div>
        <div className="legend-item">
          <span className="legend-color" style={{ backgroundColor: 'rgba(255, 128, 0, 0.8)' }}></span>
          <span>High Cost</span>
        </div>
        <div className="legend-item">
          <span className="legend-color" style={{ backgroundColor: 'rgba(255, 0, 0, 0.9)' }}></span>
          <span>Obstacle</span>
        </div>
      </div>

      {/* No costmap message */}
      {!costmap && (
        <div className="no-costmap-message">
          <p>Waiting for costmap data...</p>
        </div>
      )}

      <style>{`
        .costmap-controls {
          position: absolute;
          top: var(--spacing-md);
          right: var(--spacing-md);
          display: flex;
          flex-direction: column;
          gap: var(--spacing-sm);
          padding: var(--spacing-sm);
          background-color: rgba(22, 27, 34, 0.9);
          border: 1px solid var(--border-color);
          border-radius: var(--radius-sm);
          z-index: 1000;
        }

        .control-checkbox {
          display: flex;
          align-items: center;
          gap: var(--spacing-sm);
          cursor: pointer;
          font-size: 0.75rem;
          color: var(--text-secondary);
        }

        .control-checkbox input {
          cursor: pointer;
        }

        .control-slider {
          display: flex;
          flex-direction: column;
          gap: 2px;
          font-size: 0.75rem;
          color: var(--text-secondary);
        }

        .control-slider input {
          width: 80px;
        }

        .costmap-legend {
          position: absolute;
          bottom: var(--spacing-md);
          left: var(--spacing-md);
          display: flex;
          flex-direction: column;
          gap: 2px;
          padding: var(--spacing-sm);
          background-color: rgba(22, 27, 34, 0.9);
          border: 1px solid var(--border-color);
          border-radius: var(--radius-sm);
          z-index: 1000;
          font-size: 0.625rem;
        }

        .legend-item {
          display: flex;
          align-items: center;
          gap: var(--spacing-xs);
          color: var(--text-secondary);
        }

        .legend-color {
          width: 12px;
          height: 12px;
          border-radius: 2px;
          border: 1px solid var(--border-color);
        }

        .no-costmap-message {
          position: absolute;
          top: 50%;
          left: 50%;
          transform: translate(-50%, -50%);
          padding: var(--spacing-md);
          background-color: rgba(22, 27, 34, 0.9);
          border: 1px solid var(--border-color);
          border-radius: var(--radius-sm);
          color: var(--text-secondary);
          font-size: 0.875rem;
          z-index: 1000;
        }
      `}</style>
    </div>
  );
};

export default CostmapViewer;
