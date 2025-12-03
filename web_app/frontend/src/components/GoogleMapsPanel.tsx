/**
 * GoogleMapsPanel Component - Satellite map with waypoint editing
 * Generated/Edited by Claude
 */
import React, { useState, useCallback, useRef, useEffect } from 'react';
import {
  GoogleMap,
  useJsApiLoader,
  Marker,
  Polyline,
  InfoWindow,
} from '@react-google-maps/api';
import { useRobotStore, Waypoint } from '../services/rosbridge';

// Map container style
const containerStyle = {
  width: '100%',
  height: '100%',
};

// Default center (can be updated based on robot location)
const defaultCenter = {
  lat: 37.7749,
  lng: -122.4194,
};

// Map options for satellite view with dark styling
const mapOptions: google.maps.MapOptions = {
  mapTypeId: 'satellite',
  disableDefaultUI: false,
  zoomControl: true,
  mapTypeControl: true,
  scaleControl: true,
  streetViewControl: false,
  rotateControl: true,
  fullscreenControl: true,
  styles: [
    {
      featureType: 'all',
      elementType: 'labels.text.fill',
      stylers: [{ color: '#ffffff' }],
    },
  ],
};

// Polyline options for waypoint path
const pathOptions: google.maps.PolylineOptions = {
  strokeColor: '#58a6ff',
  strokeOpacity: 0.8,
  strokeWeight: 3,
};

interface GoogleMapsPanelProps {
  apiKey?: string;
  initialCenter?: { lat: number; lng: number };
  initialZoom?: number;
}

const GoogleMapsPanel: React.FC<GoogleMapsPanelProps> = ({
  apiKey = import.meta.env.VITE_GOOGLE_MAPS_API_KEY || '',
  initialCenter = defaultCenter,
  initialZoom = 18,
}) => {
  const { stats, waypoints, addWaypoint, removeWaypoint, setWaypoints, publishWaypoints } =
    useRobotStore();

  const [map, setMap] = useState<google.maps.Map | null>(null);
  const [selectedWaypoint, setSelectedWaypoint] = useState<Waypoint | null>(null);
  const [isEditing, setIsEditing] = useState(false);
  const [robotMarkerPosition, setRobotMarkerPosition] = useState<{ lat: number; lng: number } | null>(null);
  const waypointIdCounter = useRef(0);

  // Load Google Maps API
  const { isLoaded, loadError } = useJsApiLoader({
    id: 'google-map-script',
    googleMapsApiKey: apiKey,
  });

  // Update robot marker position when GPS data changes
  useEffect(() => {
    if (stats.gpsLat !== 0 || stats.gpsLon !== 0) {
      setRobotMarkerPosition({
        lat: stats.gpsLat,
        lng: stats.gpsLon,
      });
    }
  }, [stats.gpsLat, stats.gpsLon]);

  // Map load callback
  const onLoad = useCallback((map: google.maps.Map) => {
    setMap(map);
  }, []);

  // Map unload callback
  const onUnmount = useCallback(() => {
    setMap(null);
  }, []);

  // Handle map click to add waypoint
  const handleMapClick = useCallback(
    (event: google.maps.MapMouseEvent) => {
      if (!isEditing || !event.latLng) return;

      const newWaypoint: Waypoint = {
        id: `wp-${Date.now()}-${waypointIdCounter.current++}`,
        lat: event.latLng.lat(),
        lng: event.latLng.lng(),
        name: `Waypoint ${waypoints.length + 1}`,
        order: waypoints.length,
      };

      addWaypoint(newWaypoint);
    },
    [isEditing, waypoints.length, addWaypoint]
  );

  // Handle waypoint drag
  const handleWaypointDrag = useCallback(
    (waypointId: string, event: google.maps.MapMouseEvent) => {
      if (!event.latLng) return;

      const updatedWaypoints = waypoints.map((wp) =>
        wp.id === waypointId
          ? { ...wp, lat: event.latLng!.lat(), lng: event.latLng!.lng() }
          : wp
      );

      setWaypoints(updatedWaypoints);
    },
    [waypoints, setWaypoints]
  );

  // Handle waypoint deletion
  const handleDeleteWaypoint = useCallback(
    (waypointId: string) => {
      removeWaypoint(waypointId);
      setSelectedWaypoint(null);

      // Reorder remaining waypoints
      const remaining = waypoints
        .filter((wp) => wp.id !== waypointId)
        .map((wp, idx) => ({ ...wp, order: idx, name: `Waypoint ${idx + 1}` }));

      setWaypoints(remaining);
    },
    [waypoints, removeWaypoint, setWaypoints]
  );

  // Clear all waypoints
  const handleClearWaypoints = useCallback(() => {
    setWaypoints([]);
    setSelectedWaypoint(null);
  }, [setWaypoints]);

  // Center map on robot
  const handleCenterOnRobot = useCallback(() => {
    if (map && robotMarkerPosition) {
      map.panTo(robotMarkerPosition);
    }
  }, [map, robotMarkerPosition]);

  // Send waypoints to robot
  const handleSendWaypoints = useCallback(() => {
    publishWaypoints();
    setIsEditing(false);
  }, [publishWaypoints]);

  // Create path coordinates for polyline
  const pathCoordinates = waypoints
    .sort((a, b) => a.order - b.order)
    .map((wp) => ({ lat: wp.lat, lng: wp.lng }));

  // Handle loading error
  if (loadError) {
    return (
      <div className="map-container">
        <div className="error-container">
          <p>Failed to load Google Maps</p>
          <p style={{ fontSize: '0.875rem', marginTop: '0.5rem' }}>
            Please check your API key configuration
          </p>
        </div>
      </div>
    );
  }

  // Handle loading state
  if (!isLoaded) {
    return (
      <div className="map-container">
        <div className="loading-container">
          <div className="loading-spinner"></div>
          <p>Loading Google Maps...</p>
        </div>
      </div>
    );
  }

  return (
    <div className="map-container">
      <GoogleMap
        mapContainerStyle={containerStyle}
        center={robotMarkerPosition || initialCenter}
        zoom={initialZoom}
        options={mapOptions}
        onLoad={onLoad}
        onUnmount={onUnmount}
        onClick={handleMapClick}
      >
        {/* Robot position marker */}
        {robotMarkerPosition && (
          <Marker
            position={robotMarkerPosition}
            icon={{
              path: google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
              scale: 6,
              fillColor: '#3fb950',
              fillOpacity: 1,
              strokeColor: '#ffffff',
              strokeWeight: 2,
              rotation: stats.heading || 0,
            }}
            title="Robot Position"
          />
        )}

        {/* Waypoint markers */}
        {waypoints.map((waypoint) => (
          <Marker
            key={waypoint.id}
            position={{ lat: waypoint.lat, lng: waypoint.lng }}
            draggable={isEditing}
            onDragEnd={(e) => handleWaypointDrag(waypoint.id, e)}
            onClick={() => setSelectedWaypoint(waypoint)}
            label={{
              text: String(waypoint.order + 1),
              color: '#ffffff',
              fontWeight: 'bold',
            }}
            icon={{
              path: google.maps.SymbolPath.CIRCLE,
              scale: 12,
              fillColor: '#58a6ff',
              fillOpacity: 1,
              strokeColor: '#ffffff',
              strokeWeight: 2,
            }}
          />
        ))}

        {/* Waypoint path polyline */}
        {pathCoordinates.length > 1 && (
          <Polyline path={pathCoordinates} options={pathOptions} />
        )}

        {/* Info window for selected waypoint */}
        {selectedWaypoint && (
          <InfoWindow
            position={{ lat: selectedWaypoint.lat, lng: selectedWaypoint.lng }}
            onCloseClick={() => setSelectedWaypoint(null)}
          >
            <div style={{ color: '#000', padding: '4px' }}>
              <strong>{selectedWaypoint.name}</strong>
              <p style={{ margin: '4px 0', fontSize: '0.875rem' }}>
                Lat: {selectedWaypoint.lat.toFixed(6)}
                <br />
                Lng: {selectedWaypoint.lng.toFixed(6)}
              </p>
              {isEditing && (
                <button
                  onClick={() => handleDeleteWaypoint(selectedWaypoint.id)}
                  style={{
                    marginTop: '4px',
                    padding: '4px 8px',
                    backgroundColor: '#f85149',
                    color: '#fff',
                    border: 'none',
                    borderRadius: '4px',
                    cursor: 'pointer',
                  }}
                >
                  Delete
                </button>
              )}
            </div>
          </InfoWindow>
        )}
      </GoogleMap>

      {/* Map controls overlay */}
      <div className="map-controls-overlay">
        <button
          className={`map-control-btn ${isEditing ? 'active' : ''}`}
          onClick={() => setIsEditing(!isEditing)}
          title={isEditing ? 'Exit Edit Mode' : 'Edit Waypoints'}
        >
          {isEditing ? 'Done' : 'Edit'}
        </button>

        {isEditing && (
          <>
            <button
              className="map-control-btn"
              onClick={handleClearWaypoints}
              disabled={waypoints.length === 0}
              title="Clear All Waypoints"
            >
              Clear
            </button>
            <button
              className="map-control-btn primary"
              onClick={handleSendWaypoints}
              disabled={waypoints.length === 0}
              title="Send Waypoints to Robot"
            >
              Send
            </button>
          </>
        )}

        <button
          className="map-control-btn"
          onClick={handleCenterOnRobot}
          disabled={!robotMarkerPosition}
          title="Center on Robot"
        >
          Center
        </button>
      </div>

      {/* Waypoint count badge */}
      <div className="waypoint-badge">
        {waypoints.length} waypoint{waypoints.length !== 1 ? 's' : ''}
      </div>

      <style>{`
        .map-controls-overlay {
          position: absolute;
          top: var(--spacing-md);
          left: var(--spacing-md);
          display: flex;
          gap: var(--spacing-sm);
          z-index: 1000;
        }

        .map-control-btn {
          padding: var(--spacing-sm) var(--spacing-md);
          background-color: rgba(22, 27, 34, 0.9);
          border: 1px solid var(--border-color);
          border-radius: var(--radius-sm);
          color: var(--text-primary);
          font-size: 0.875rem;
          cursor: pointer;
          transition: all 0.2s;
        }

        .map-control-btn:hover:not(:disabled) {
          background-color: rgba(33, 38, 45, 0.95);
        }

        .map-control-btn:disabled {
          opacity: 0.5;
          cursor: not-allowed;
        }

        .map-control-btn.active {
          background-color: var(--accent-blue);
          border-color: var(--accent-blue);
        }

        .map-control-btn.primary {
          background-color: var(--accent-green);
          border-color: var(--accent-green);
        }

        .waypoint-badge {
          position: absolute;
          bottom: var(--spacing-md);
          left: var(--spacing-md);
          padding: var(--spacing-xs) var(--spacing-sm);
          background-color: rgba(22, 27, 34, 0.9);
          border: 1px solid var(--border-color);
          border-radius: var(--radius-sm);
          color: var(--text-secondary);
          font-size: 0.75rem;
          z-index: 1000;
        }
      `}</style>
    </div>
  );
};

export default GoogleMapsPanel;
