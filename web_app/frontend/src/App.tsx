/**
 * Main Application Component with Dashboard Layout
 * Generated/Edited by Claude
 */
import React, { useEffect, useState } from 'react';
import CameraFeed from './components/CameraFeed';
import GoogleMapsPanel from './components/GoogleMapsPanel';
import CostmapViewer from './components/CostmapViewer';
import StatsPanel from './components/StatsPanel';
import JoystickControl from './components/JoystickControl';
import { useRobotStore } from './services/rosbridge';

const App: React.FC = () => {
  const { connected, connect, disconnect } = useRobotStore();
  const [connectionStatus, setConnectionStatus] = useState<'connecting' | 'connected' | 'disconnected'>('disconnected');

  useEffect(() => {
    setConnectionStatus('connecting');
    connect('ws://localhost:9090');

    return () => {
      disconnect();
    };
  }, [connect, disconnect]);

  useEffect(() => {
    setConnectionStatus(connected ? 'connected' : 'disconnected');
  }, [connected]);

  return (
    <div className="app-container">
      <header className="app-header">
        <h1>WALL-E Wildlife Robot Control Panel</h1>
        <div className="connection-status">
          <span className={`status-indicator ${connectionStatus}`}></span>
          <span className="status-text">
            ROSBridge: {connectionStatus.charAt(0).toUpperCase() + connectionStatus.slice(1)}
          </span>
        </div>
      </header>

      <main className="dashboard-grid">
        <section className="panel camera-panel">
          <h2>Camera Feed</h2>
          <CameraFeed streamUrl="/api/camera/stream" />
        </section>

        <section className="panel maps-panel">
          <h2>Navigation Map</h2>
          <GoogleMapsPanel />
        </section>

        <section className="panel costmap-panel">
          <h2>Costmap Visualization</h2>
          <CostmapViewer />
        </section>

        <section className="panel stats-panel">
          <h2>Robot Statistics</h2>
          <StatsPanel />
        </section>

        <section className="panel joystick-panel">
          <h2>Manual Control</h2>
          <JoystickControl />
        </section>
      </main>

      <footer className="app-footer">
        <p>WALL-E Wildlife Monitoring System</p>
      </footer>
    </div>
  );
};

export default App;
