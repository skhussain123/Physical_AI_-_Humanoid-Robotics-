import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

type LearningObjectiveProps = {
  children: React.ReactNode;
};

export default function LearningObjective({ children }: LearningObjectiveProps): JSX.Element {
  return (
    <div className={clsx('alert alert--success', styles.admonition)}>
      <h4>Learning Objective</h4>
      <div>{children}</div>
    </div>
  );
}