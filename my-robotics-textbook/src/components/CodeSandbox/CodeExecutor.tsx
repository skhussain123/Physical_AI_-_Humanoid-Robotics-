import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './CodeExecutor.module.css';

interface CodeExecutorProps {
  code: string;
  language?: string;
  onExecute?: (result: any) => void;
}

const CodeExecutor: React.FC<CodeExecutorProps> = ({
  code,
  language = 'python',
  onExecute
}) => {
  const [isExecuting, setIsExecuting] = useState(false);
  const [output, setOutput] = useState('');
  const [error, setError] = useState('');

  const executeCode = async () => {
    setIsExecuting(true);
    setError('');
    setOutput('');

    try {
      // Simulate code execution with timeout
      await new Promise(resolve => setTimeout(resolve, 1500));

      // For this demo, we'll simulate execution results
      const simulatedOutput = `Code executed successfully!\nLanguage: ${language}\nExecution time: 0.123s\nResult: Simulation completed`;

      setOutput(simulatedOutput);

      if (onExecute) {
        onExecute({ success: true, output: simulatedOutput });
      }
    } catch (err) {
      const errorMsg = `Error: ${(err as Error).message || 'Execution failed'}`;
      setError(errorMsg);

      if (onExecute) {
        onExecute({ success: false, error: errorMsg });
      }
    } finally {
      setIsExecuting(false);
    }
  };

  return (
    <div className={clsx(styles.codeExecutor)}>
      <div className={styles.executorHeader}>
        <span className={styles.languageLabel}>{language.toUpperCase()}</span>
        <button
          className={clsx('button button--primary', styles.executeButton)}
          onClick={executeCode}
          disabled={isExecuting}
        >
          {isExecuting ? 'Executing...' : 'Execute'}
        </button>
      </div>

      <div className={styles.codeDisplay}>
        <pre className={styles.codeText}>
          <code>{code}</code>
        </pre>
      </div>

      {(output || error) && (
        <div className={styles.outputContainer}>
          <h4>Output:</h4>
          <pre className={clsx(styles.outputText, error && styles.errorOutput)}>
            {error || output}
          </pre>
        </div>
      )}
    </div>
  );
};

export default CodeExecutor;