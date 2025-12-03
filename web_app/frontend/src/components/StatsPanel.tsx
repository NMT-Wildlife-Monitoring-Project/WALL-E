/**
 * StatsPanel Component - Robot statistics display
 * Generated/Edited by Claude
 */
import React, { useEffect, useState, useRef } from 'react';
import { useRobotStore } from '../services/rosbridge';
import { THRESHOLDS, TIMING } from '../constants';

// Format uptime to human readable string
const formatUptime = (seconds: number): string => {
  if (seconds < 60) return `${seconds}s`;
  if (seconds < 3600) return `${Math.floor(seconds / 60)}m ${seconds % 60}s`;
  const hours = Math.floor(seconds / 3600);
  const mins = Math.floor((seconds % 3600) / 60);
  return `${hours}h ${mins}m`;
};

// Format GPS coordinates
const formatGPS = (lat: number, lng: number): string => {
  if (lat === 0 && lng === 0) return 'No GPS Fix';
  return `${lat.toFixed(6)}, ${lng.toFixed(6)}`;
};

// Get status class based on value and thresholds
const getStatusClass = (
  value: number,
  thresholds: { warning: number; danger: number },
  inverse: boolean = false
): 'good' | 'warning' | 'danger' => {
  if (inverse) {
    // For values where lower is worse (battery, solar)
    if (value <= thresholds.danger) return 'danger';
    if (value <= thresholds.warning) return 'warning';
    return 'good';
  }
  // For values where higher is worse (temp, cpu, memory)
  if (value >= thresholds.danger) return 'danger';
  if (value >= thresholds.warning) return 'warning';
  return 'good';
};

interface StatItemProps {
  label: string;
  value: string | number;
  unit?: string;
  status?: 'good' | 'warning' | 'danger';
  progress?: number;
  icon?: React.ReactNode;
}

const StatItem: React.FC<StatItemProps> = ({
  label,
  value,
  unit = '',
  status = 'good',
  progress,
  icon,
}) => {
  return (
    <div className="stat-item">
      <div className="stat-header">
        {icon && <span className="stat-icon">{icon}</span>}
        <span className="stat-label">{label}</span>
      </div>
      <div className="stat-content">
        <span className={`stat-value ${status}`}>
          {value}
          {unit && <span className="stat-unit">{unit}</span>}
        </span>
        {progress !== undefined && (
          <div className="progress-bar">
            <div
              className={`progress-fill ${status}`}
              style={{ width: `${Math.min(Math.max(progress, 0), 100)}%` }}
            />
          </div>
        )}
      </div>
    </div>
  );
};

// SVG Icons
const Icons = {
  temperature: (
    <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <path d="M14 14.76V3.5a2.5 2.5 0 0 0-5 0v11.26a4.5 4.5 0 1 0 5 0z" />
    </svg>
  ),
  battery: (
    <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <rect x="1" y="6" width="18" height="12" rx="2" ry="2" />
      <line x1="23" y1="13" x2="23" y2="11" />
    </svg>
  ),
  solar: (
    <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <circle cx="12" cy="12" r="5" />
      <line x1="12" y1="1" x2="12" y2="3" />
      <line x1="12" y1="21" x2="12" y2="23" />
      <line x1="4.22" y1="4.22" x2="5.64" y2="5.64" />
      <line x1="18.36" y1="18.36" x2="19.78" y2="19.78" />
      <line x1="1" y1="12" x2="3" y2="12" />
      <line x1="21" y1="12" x2="23" y2="12" />
      <line x1="4.22" y1="19.78" x2="5.64" y2="18.36" />
      <line x1="18.36" y1="5.64" x2="19.78" y2="4.22" />
    </svg>
  ),
  cpu: (
    <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <rect x="4" y="4" width="16" height="16" rx="2" ry="2" />
      <rect x="9" y="9" width="6" height="6" />
      <line x1="9" y1="1" x2="9" y2="4" />
      <line x1="15" y1="1" x2="15" y2="4" />
      <line x1="9" y1="20" x2="9" y2="23" />
      <line x1="15" y1="20" x2="15" y2="23" />
      <line x1="20" y1="9" x2="23" y2="9" />
      <line x1="20" y1="14" x2="23" y2="14" />
      <line x1="1" y1="9" x2="4" y2="9" />
      <line x1="1" y1="14" x2="4" y2="14" />
    </svg>
  ),
  memory: (
    <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <rect x="2" y="6" width="20" height="12" rx="2" ry="2" />
      <line x1="6" y1="6" x2="6" y2="18" />
      <line x1="10" y1="6" x2="10" y2="18" />
      <line x1="14" y1="6" x2="14" y2="18" />
      <line x1="18" y1="6" x2="18" y2="18" />
    </svg>
  ),
  gps: (
    <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <path d="M21 10c0 7-9 13-9 13s-9-6-9-13a9 9 0 0 1 18 0z" />
      <circle cx="12" cy="10" r="3" />
    </svg>
  ),
  compass: (
    <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <circle cx="12" cy="12" r="10" />
      <polygon points="16.24 7.76 14.12 14.12 7.76 16.24 9.88 9.88 16.24 7.76" />
    </svg>
  ),
  speed: (
    <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <path d="M12 2L12 6" />
      <path d="M12 18L12 22" />
      <circle cx="12" cy="12" r="8" />
      <path d="M12 12L16 8" />
    </svg>
  ),
  clock: (
    <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
      <circle cx="12" cy="12" r="10" />
      <polyline points="12 6 12 12 16 14" />
    </svg>
  ),
};

const StatsPanel: React.FC = () => {
  const { stats, connected } = useRobotStore();
  const [lastUpdate, setLastUpdate] = useState<Date>(new Date());
  const isMountedRef = useRef(true);
  const lastUpdateTimeRef = useRef(0);

  // Track mounted state
  useEffect(() => {
    isMountedRef.current = true;
    return () => {
      isMountedRef.current = false;
    };
  }, []);

  // Update timestamp when stats change (debounced to prevent excessive re-renders)
  useEffect(() => {
    if (!connected || !isMountedRef.current) return;

    const now = Date.now();
    if (now - lastUpdateTimeRef.current >= TIMING.STATS_UPDATE_DEBOUNCE_MS) {
      lastUpdateTimeRef.current = now;
      setLastUpdate(new Date());
    }
  }, [stats, connected]);

  return (
    <div className="stats-panel-container">
      {!connected && (
        <div className="stats-offline-banner">
          <span>Robot Offline</span>
        </div>
      )}

      <div className="stats-grid">
        {/* Power Section */}
        <div className="stats-section">
          <h3 className="section-title">Power</h3>
          <StatItem
            label="Battery"
            value={stats.batteryLevel.toFixed(1)}
            unit="%"
            status={getStatusClass(stats.batteryLevel, THRESHOLDS.battery, true)}
            progress={stats.batteryLevel}
            icon={Icons.battery}
          />
          <StatItem
            label="Solar Input"
            value={stats.solarPower.toFixed(1)}
            unit="W"
            status={getStatusClass(stats.solarPower, THRESHOLDS.solar, true)}
            icon={Icons.solar}
          />
        </div>

        {/* System Section */}
        <div className="stats-section">
          <h3 className="section-title">System</h3>
          <StatItem
            label="Temperature"
            value={stats.temperature.toFixed(1)}
            unit="C"
            status={getStatusClass(stats.temperature, THRESHOLDS.temperature)}
            icon={Icons.temperature}
          />
          <StatItem
            label="CPU Usage"
            value={stats.cpuUsage.toFixed(0)}
            unit="%"
            status={getStatusClass(stats.cpuUsage, THRESHOLDS.cpu)}
            progress={stats.cpuUsage}
            icon={Icons.cpu}
          />
          <StatItem
            label="Memory"
            value={stats.memoryUsage.toFixed(0)}
            unit="%"
            status={getStatusClass(stats.memoryUsage, THRESHOLDS.memory)}
            progress={stats.memoryUsage}
            icon={Icons.memory}
          />
        </div>

        {/* Navigation Section */}
        <div className="stats-section">
          <h3 className="section-title">Navigation</h3>
          <StatItem
            label="GPS"
            value={formatGPS(stats.gpsLat, stats.gpsLon)}
            status={stats.gpsLat !== 0 || stats.gpsLon !== 0 ? 'good' : 'danger'}
            icon={Icons.gps}
          />
          <StatItem
            label="Heading"
            value={stats.heading.toFixed(1)}
            unit=" deg"
            icon={Icons.compass}
          />
          <StatItem
            label="Speed"
            value={stats.speed.toFixed(2)}
            unit=" m/s"
            icon={Icons.speed}
          />
        </div>

        {/* Runtime Section */}
        <div className="stats-section">
          <h3 className="section-title">Runtime</h3>
          <StatItem
            label="Uptime"
            value={formatUptime(stats.uptime)}
            icon={Icons.clock}
          />
        </div>
      </div>

      <div className="stats-footer">
        <span className="last-update">
          Last update: {lastUpdate.toLocaleTimeString()}
        </span>
      </div>

      <style>{`
        .stats-panel-container {
          display: flex;
          flex-direction: column;
          height: calc(100% - 40px);
          overflow-y: auto;
        }

        .stats-offline-banner {
          background-color: var(--accent-red);
          color: white;
          text-align: center;
          padding: var(--spacing-xs);
          font-size: 0.75rem;
          font-weight: 600;
          margin-bottom: var(--spacing-sm);
          border-radius: var(--radius-sm);
        }

        .stats-grid {
          display: flex;
          flex-direction: column;
          gap: var(--spacing-md);
          flex: 1;
        }

        .stats-section {
          padding-bottom: var(--spacing-sm);
          border-bottom: 1px solid var(--border-color);
        }

        .stats-section:last-child {
          border-bottom: none;
        }

        .section-title {
          font-size: 0.75rem;
          font-weight: 600;
          color: var(--text-secondary);
          text-transform: uppercase;
          letter-spacing: 0.5px;
          margin-bottom: var(--spacing-sm);
        }

        .stat-item {
          display: flex;
          justify-content: space-between;
          align-items: flex-start;
          padding: var(--spacing-xs) 0;
        }

        .stat-header {
          display: flex;
          align-items: center;
          gap: var(--spacing-xs);
        }

        .stat-icon {
          color: var(--text-secondary);
          display: flex;
          align-items: center;
        }

        .stat-label {
          color: var(--text-secondary);
          font-size: 0.8125rem;
        }

        .stat-content {
          display: flex;
          flex-direction: column;
          align-items: flex-end;
          gap: 2px;
        }

        .stat-value {
          font-weight: 600;
          font-size: 0.9375rem;
          font-family: monospace;
        }

        .stat-unit {
          font-size: 0.75rem;
          font-weight: 400;
          color: var(--text-secondary);
          margin-left: 2px;
        }

        .stat-value.good {
          color: var(--accent-green);
        }

        .stat-value.warning {
          color: var(--accent-yellow);
        }

        .stat-value.danger {
          color: var(--accent-red);
        }

        .progress-bar {
          width: 60px;
          height: 4px;
          background-color: var(--bg-tertiary);
          border-radius: 2px;
          overflow: hidden;
        }

        .progress-fill {
          height: 100%;
          transition: width 0.3s ease;
        }

        .progress-fill.good {
          background-color: var(--accent-green);
        }

        .progress-fill.warning {
          background-color: var(--accent-yellow);
        }

        .progress-fill.danger {
          background-color: var(--accent-red);
        }

        .stats-footer {
          margin-top: auto;
          padding-top: var(--spacing-sm);
          border-top: 1px solid var(--border-color);
        }

        .last-update {
          font-size: 0.6875rem;
          color: var(--text-secondary);
        }
      `}</style>
    </div>
  );
};

export default StatsPanel;
