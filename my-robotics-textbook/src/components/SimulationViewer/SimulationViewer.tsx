import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './SimulationViewer.module.css';

interface SimulationViewerProps {
  title?: string;
  simulationType?: 'gazebo' | 'isaac' | 'unity' | 'custom';
  onSimulationChange?: (simulationState: any) => void;
}

const SimulationViewer: React.FC<SimulationViewerProps> = ({
  title = 'Robotics Simulation Viewer',
  simulationType = 'gazebo',
  onSimulationChange
}) => {
  const [isRunning, setIsRunning] = useState(false);
  const [simulationTime, setSimulationTime] = useState(0);
  const [messages, setMessages] = useState<string[]>([]);

  const handleRunSimulation = () => {
    setIsRunning(true);
    setMessages(prev => [...prev, `Starting ${simulationType} simulation...`]);

    // Simulate simulation running
    const interval = setInterval(() => {
      setSimulationTime(prev => prev + 0.1);
      setMessages(prev => [...prev, `Simulation time: ${prev.length * 0.1}s`]);

      if (simulationTime > 5) { // Stop after 5 simulated seconds
        clearInterval(interval);
        setIsRunning(false);
        setMessages(prev => [...prev, 'Simulation completed']);
      }
    }, 1000);

    setTimeout(() => {
      clearInterval(interval);
      setIsRunning(false);
    }, 6000); // Stop after 6 seconds of real time
  };

  const handleStopSimulation = () => {
    setIsRunning(false);
    setMessages(prev => [...prev, 'Simulation stopped by user']);
  };

  return (
    <div className={clsx('container', styles.simulationViewer)}>
      <div className={styles.simulationHeader}>
        <h3>{title}</h3>
        <div className={styles.simulationControls}>
          <button
            className={clsx('button button--primary', styles.controlButton)}
            onClick={handleRunSimulation}
            disabled={isRunning}
          >
            {isRunning ? 'Running...' : 'Start Simulation'}
          </button>
          <button
            className={clsx('button button--secondary', styles.controlButton)}
            onClick={handleStopSimulation}
            disabled={!isRunning}
          >
            Stop
          </button>
        </div>
      </div>

      <div className={styles.simulationDisplay}>
        <div className={styles.simulationCanvas}>
          {/* Placeholder for simulation visualization */}
          <div className={styles.simulationPlaceholder}>
            <div className={styles.robotPlaceholder}>
              <div className={styles.robotBase}></div>
              <div className={styles.robotArm}></div>
              <div className={styles.robotSensor}></div>
            </div>
            <div className={styles.environmentPlaceholder}>
              <div className={styles.obstacle}></div>
              <div className={styles.target}></div>
            </div>
          </div>
        </div>

        <div className={styles.simulationInfo}>
          <div className={styles.infoPanel}>
            <h4>Simulation Info</h4>
            <p>Type: {simulationType.toUpperCase()}</p>
            <p>Time: {simulationTime.toFixed(1)}s</p>
            <p>Status: {isRunning ? 'Running' : 'Stopped'}</p>
          </div>

          <div className={styles.messageLog}>
            <h4>Messages</h4>
            <div className={styles.messageList}>
              {messages.slice(-5).map((msg, index) => (
                <div key={index} className={styles.message}>
                  {msg}
                </div>
              ))}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default SimulationViewer;