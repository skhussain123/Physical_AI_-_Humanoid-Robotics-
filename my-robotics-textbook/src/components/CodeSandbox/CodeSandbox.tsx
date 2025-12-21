import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './CodeSandbox.module.css';
import { supportedLanguages, getLanguageById } from './LanguageSupport';

interface CodeSandboxProps {
  code?: string;
  language?: string;
  title?: string;
  onRun?: (code: string, language: string) => void;
}

const CodeSandbox: React.FC<CodeSandboxProps> = ({
  code = '// Write your robotics code here',
  language = 'python',
  title = 'Code Editor',
  onRun
}) => {
  const [currentCode, setCurrentCode] = useState(code);
  const [currentLanguage, setCurrentLanguage] = useState(language);
  const [output, setOutput] = useState('');
  const [isRunning, setIsRunning] = useState(false);

  const handleRun = () => {
    setIsRunning(true);
    setOutput('Running code...');

    // Simulate code execution based on language
    setTimeout(() => {
      let result = '';
      const langDef = getLanguageById(currentLanguage);
      if (langDef) {
        switch(currentLanguage) {
          case 'python':
            result = `Python code executed successfully!\nInput: ${currentCode.substring(0, 30)}...`;
            break;
          case 'javascript':
            result = `JavaScript code executed successfully!\nInput: ${currentCode.substring(0, 30)}...`;
            break;
          case 'xml':
            result = `XML/URDF processed successfully!\nInput: ${currentCode.substring(0, 30)}...`;
            break;
          case 'bash':
            result = `Bash script executed successfully!\nInput: ${currentCode.substring(0, 30)}...`;
            break;
          case 'c++':
            result = `C++ code executed successfully!\nInput: ${currentCode.substring(0, 30)}...`;
            break;
          default:
            result = `Code executed successfully!\nLanguage: ${langDef.displayName}\nInput: ${currentCode.substring(0, 30)}...`;
        }
      } else {
        result = `Code executed successfully!\nLanguage: ${currentLanguage}\nInput: ${currentCode.substring(0, 30)}...`;
      }
      setOutput(result);
      setIsRunning(false);
    }, 1000);

    if (onRun) {
      onRun(currentCode, currentLanguage);
    }
  };

  const handleCodeChange = (e: React.ChangeEvent<HTMLTextAreaElement>) => {
    setCurrentCode(e.target.value);
  };

  const handleLanguageChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    setCurrentLanguage(e.target.value);
  };

  return (
    <div className={clsx('container', styles.codeSandboxContainer)}>
      <div className={styles.codeSandboxHeader}>
        <div className={styles.headerLeft}>
          <h3>{title}</h3>
          <select
            value={currentLanguage}
            onChange={handleLanguageChange}
            className={clsx('form-select', styles.languageSelect)}
          >
            {supportedLanguages.map(lang => (
              <option key={lang.id} value={lang.id}>
                {lang.displayName}
              </option>
            ))}
          </select>
        </div>
        <button
          className={clsx('button button--primary', styles.runButton)}
          onClick={handleRun}
          disabled={isRunning}
        >
          {isRunning ? 'Running...' : 'Run Code'}
        </button>
      </div>

      <div className={styles.codeEditor}>
        <textarea
          value={currentCode}
          onChange={handleCodeChange}
          className={clsx('code', styles.textarea)}
          spellCheck="false"
        />
      </div>

      <div className={styles.codeOutput}>
        <h4>Output:</h4>
        <pre className={styles.outputText}>{output}</pre>
      </div>
    </div>
  );
};

export default CodeSandbox;