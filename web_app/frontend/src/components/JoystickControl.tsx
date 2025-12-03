/**
 * JoystickControl Component - Virtual joystick using nipplejs
 * Generated/Edited by Claude
 */
import React, { useEffect, useRef, useState, useCallback } from 'react';
import nipplejs, { JoystickManager, JoystickOutputData } from 'nipplejs';
import { useRobotStore } from '../services/rosbridge';

interface VelocityState {
  linear: number;
  angular: number;
}

interface JoystickControlProps {
  maxLinearSpeed?: number;
  maxAngularSpeed?: number;
  deadzone?: number;
  publishRate?: number;
}

const JoystickControl: React.FC<JoystickControlProps> = ({
  maxLinearSpeed = 1.0,
  maxAngularSpeed = 1.5,
  deadzone = 0.1,
  publishRate = 100,
}) => {
  const joystickRef = useRef<HTMLDivElement>(null);
  const joystickManagerRef = useRef<JoystickManager | null>(null);
  const publishIntervalRef = useRef<NodeJS.Timeout | null>(null);
  const velocityRef = useRef<VelocityState>({ linear: 0, angular: 0 });

  const { sendVelocity, connected } = useRobotStore();
  const [currentVelocity, setCurrentVelocity] = useState<VelocityState>({
    linear: 0,
    angular: 0,
  });
  const [isActive, setIsActive] = useState(false);
  const [controlMode, setControlMode] = useState<'joystick' | 'keyboard'>('joystick');
  const [emergencyStop, setEmergencyStop] = useState(false);

  // Apply deadzone to value
  const applyDeadzone = useCallback(
    (value: number): number => {
      if (Math.abs(value) < deadzone) return 0;
      // Rescale value to account for deadzone
      const sign = value > 0 ? 1 : -1;
      return sign * ((Math.abs(value) - deadzone) / (1 - deadzone));
    },
    [deadzone]
  );

  // Publish velocity at regular intervals
  const startPublishing = useCallback(() => {
    if (publishIntervalRef.current) return;

    publishIntervalRef.current = setInterval(() => {
      if (!emergencyStop) {
        sendVelocity(velocityRef.current);
      }
    }, publishRate);
  }, [sendVelocity, publishRate, emergencyStop]);

  // Stop publishing
  const stopPublishing = useCallback(() => {
    if (publishIntervalRef.current) {
      clearInterval(publishIntervalRef.current);
      publishIntervalRef.current = null;
    }
    // Send zero velocity
    velocityRef.current = { linear: 0, angular: 0 };
    setCurrentVelocity({ linear: 0, angular: 0 });
    sendVelocity({ linear: 0, angular: 0 });
  }, [sendVelocity]);

  // Emergency stop handler
  const handleEmergencyStop = useCallback(() => {
    setEmergencyStop(true);
    velocityRef.current = { linear: 0, angular: 0 };
    setCurrentVelocity({ linear: 0, angular: 0 });
    sendVelocity({ linear: 0, angular: 0 });
  }, [sendVelocity]);

  // Reset emergency stop
  const resetEmergencyStop = useCallback(() => {
    setEmergencyStop(false);
  }, []);

  // Initialize nipplejs joystick
  useEffect(() => {
    if (!joystickRef.current || controlMode !== 'joystick') return;

    joystickManagerRef.current = nipplejs.create({
      zone: joystickRef.current,
      mode: 'static',
      position: { left: '50%', top: '50%' },
      color: '#58a6ff',
      size: 150,
      restOpacity: 0.5,
      fadeTime: 100,
    });

    const manager = joystickManagerRef.current;

    manager.on('start', () => {
      setIsActive(true);
      startPublishing();
    });

    manager.on('move', (_evt: unknown, data: JoystickOutputData) => {
      if (emergencyStop) return;

      // Convert joystick position to velocity
      // Forward/back on Y axis (negative because up is negative in screen coords)
      const linear = applyDeadzone(-data.vector.y) * maxLinearSpeed;
      // Left/right on X axis (negative for standard ROS convention - left positive)
      const angular = applyDeadzone(-data.vector.x) * maxAngularSpeed;

      velocityRef.current = { linear, angular };
      setCurrentVelocity({ linear, angular });
    });

    manager.on('end', () => {
      setIsActive(false);
      stopPublishing();
    });

    return () => {
      manager.destroy();
      joystickManagerRef.current = null;
    };
  }, [
    controlMode,
    maxLinearSpeed,
    maxAngularSpeed,
    applyDeadzone,
    startPublishing,
    stopPublishing,
    emergencyStop,
  ]);

  // Keyboard control handler
  useEffect(() => {
    if (controlMode !== 'keyboard') return;

    const keysPressed = new Set<string>();

    const updateVelocityFromKeys = () => {
      if (emergencyStop) return;

      let linear = 0;
      let angular = 0;

      if (keysPressed.has('w') || keysPressed.has('ArrowUp')) linear += maxLinearSpeed;
      if (keysPressed.has('s') || keysPressed.has('ArrowDown')) linear -= maxLinearSpeed;
      if (keysPressed.has('a') || keysPressed.has('ArrowLeft')) angular += maxAngularSpeed;
      if (keysPressed.has('d') || keysPressed.has('ArrowRight')) angular -= maxAngularSpeed;

      velocityRef.current = { linear, angular };
      setCurrentVelocity({ linear, angular });
    };

    const handleKeyDown = (e: KeyboardEvent) => {
      const key = e.key.toLowerCase();
      if (['w', 'a', 's', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(key)) {
        e.preventDefault();
        if (!keysPressed.has(key)) {
          keysPressed.add(key);
          if (keysPressed.size === 1) {
            setIsActive(true);
            startPublishing();
          }
          updateVelocityFromKeys();
        }
      }
      // Space for emergency stop
      if (e.key === ' ') {
        e.preventDefault();
        handleEmergencyStop();
      }
    };

    const handleKeyUp = (e: KeyboardEvent) => {
      const key = e.key.toLowerCase();
      keysPressed.delete(key);
      if (keysPressed.size === 0) {
        setIsActive(false);
        stopPublishing();
      } else {
        updateVelocityFromKeys();
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
      stopPublishing();
    };
  }, [
    controlMode,
    maxLinearSpeed,
    maxAngularSpeed,
    startPublishing,
    stopPublishing,
    emergencyStop,
    handleEmergencyStop,
  ]);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (publishIntervalRef.current) {
        clearInterval(publishIntervalRef.current);
      }
    };
  }, []);

  return (
    <div className="joystick-container">
      {/* Control mode selector */}
      <div className="control-mode-selector">
        <button
          className={`mode-btn ${controlMode === 'joystick' ? 'active' : ''}`}
          onClick={() => setControlMode('joystick')}
        >
          Joystick
        </button>
        <button
          className={`mode-btn ${controlMode === 'keyboard' ? 'active' : ''}`}
          onClick={() => setControlMode('keyboard')}
        >
          Keyboard
        </button>
      </div>

      {/* Emergency stop banner */}
      {emergencyStop && (
        <div className="emergency-stop-banner">
          <span>EMERGENCY STOP ACTIVE</span>
          <button onClick={resetEmergencyStop}>Reset</button>
        </div>
      )}

      {/* Joystick zone */}
      {controlMode === 'joystick' ? (
        <div
          ref={joystickRef}
          className={`joystick-zone ${isActive ? 'active' : ''} ${!connected ? 'disabled' : ''}`}
        />
      ) : (
        <div className="keyboard-instructions">
          <div className="key-row">
            <div className="key">W</div>
          </div>
          <div className="key-row">
            <div className="key">A</div>
            <div className="key">S</div>
            <div className="key">D</div>
          </div>
          <p className="keyboard-hint">Use WASD or Arrow keys</p>
          <p className="keyboard-hint">Space for Emergency Stop</p>
        </div>
      )}

      {/* Velocity display */}
      <div className="joystick-info">
        <div className="velocity-display">
          <span className="velocity-label">Linear:</span>
          <span className={`velocity-value ${currentVelocity.linear !== 0 ? 'active' : ''}`}>
            {currentVelocity.linear.toFixed(2)} m/s
          </span>
        </div>
        <div className="velocity-display">
          <span className="velocity-label">Angular:</span>
          <span className={`velocity-value ${currentVelocity.angular !== 0 ? 'active' : ''}`}>
            {currentVelocity.angular.toFixed(2)} rad/s
          </span>
        </div>
      </div>

      {/* Status indicator */}
      <div className="control-status">
        <span className={`status-dot ${isActive ? 'active' : ''}`}></span>
        <span>{isActive ? 'Controlling' : 'Idle'}</span>
      </div>

      {/* Emergency stop button */}
      <button
        className="emergency-stop-btn"
        onClick={handleEmergencyStop}
        disabled={emergencyStop}
      >
        STOP
      </button>

      <style>{`
        .joystick-container {
          display: flex;
          flex-direction: column;
          align-items: center;
          justify-content: space-between;
          height: calc(100% - 40px);
          gap: var(--spacing-sm);
        }

        .control-mode-selector {
          display: flex;
          gap: var(--spacing-xs);
          width: 100%;
        }

        .mode-btn {
          flex: 1;
          padding: var(--spacing-xs) var(--spacing-sm);
          background-color: var(--bg-tertiary);
          border: 1px solid var(--border-color);
          border-radius: var(--radius-sm);
          color: var(--text-secondary);
          font-size: 0.75rem;
          cursor: pointer;
          transition: all 0.2s;
        }

        .mode-btn:hover {
          background-color: var(--border-color);
        }

        .mode-btn.active {
          background-color: var(--accent-blue);
          border-color: var(--accent-blue);
          color: white;
        }

        .emergency-stop-banner {
          display: flex;
          align-items: center;
          justify-content: space-between;
          width: 100%;
          padding: var(--spacing-xs) var(--spacing-sm);
          background-color: var(--accent-red);
          border-radius: var(--radius-sm);
          color: white;
          font-size: 0.75rem;
          font-weight: 600;
        }

        .emergency-stop-banner button {
          padding: 2px 8px;
          background-color: rgba(255, 255, 255, 0.2);
          border: 1px solid white;
          border-radius: var(--radius-sm);
          color: white;
          cursor: pointer;
        }

        .joystick-zone {
          width: 150px;
          height: 150px;
          background-color: var(--bg-tertiary);
          border-radius: 50%;
          border: 2px solid var(--border-color);
          position: relative;
          transition: border-color 0.2s, box-shadow 0.2s;
        }

        .joystick-zone.active {
          border-color: var(--accent-blue);
          box-shadow: 0 0 20px rgba(88, 166, 255, 0.3);
        }

        .joystick-zone.disabled {
          opacity: 0.5;
          pointer-events: none;
        }

        .keyboard-instructions {
          display: flex;
          flex-direction: column;
          align-items: center;
          gap: var(--spacing-xs);
          padding: var(--spacing-md);
        }

        .key-row {
          display: flex;
          gap: var(--spacing-xs);
        }

        .key {
          width: 36px;
          height: 36px;
          display: flex;
          align-items: center;
          justify-content: center;
          background-color: var(--bg-tertiary);
          border: 1px solid var(--border-color);
          border-radius: var(--radius-sm);
          color: var(--text-primary);
          font-weight: 600;
          font-size: 0.875rem;
        }

        .keyboard-hint {
          font-size: 0.6875rem;
          color: var(--text-secondary);
          margin-top: var(--spacing-xs);
        }

        .joystick-info {
          display: flex;
          flex-direction: column;
          gap: var(--spacing-xs);
          width: 100%;
          font-family: monospace;
        }

        .velocity-display {
          display: flex;
          justify-content: space-between;
          padding: var(--spacing-xs);
          background-color: var(--bg-tertiary);
          border-radius: var(--radius-sm);
        }

        .velocity-label {
          color: var(--text-secondary);
          font-size: 0.75rem;
        }

        .velocity-value {
          color: var(--text-primary);
          font-size: 0.75rem;
        }

        .velocity-value.active {
          color: var(--accent-green);
        }

        .control-status {
          display: flex;
          align-items: center;
          gap: var(--spacing-xs);
          font-size: 0.75rem;
          color: var(--text-secondary);
        }

        .status-dot {
          width: 8px;
          height: 8px;
          border-radius: 50%;
          background-color: var(--text-secondary);
        }

        .status-dot.active {
          background-color: var(--accent-green);
          box-shadow: 0 0 8px var(--accent-green);
        }

        .emergency-stop-btn {
          width: 100%;
          padding: var(--spacing-sm);
          background-color: var(--accent-red);
          border: none;
          border-radius: var(--radius-sm);
          color: white;
          font-weight: 700;
          font-size: 0.875rem;
          cursor: pointer;
          transition: background-color 0.2s;
        }

        .emergency-stop-btn:hover:not(:disabled) {
          background-color: #da3633;
        }

        .emergency-stop-btn:disabled {
          opacity: 0.5;
          cursor: not-allowed;
        }
      `}</style>
    </div>
  );
};

export default JoystickControl;
