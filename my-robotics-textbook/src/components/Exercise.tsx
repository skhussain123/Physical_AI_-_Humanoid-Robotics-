import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

type ExerciseProps = {
  children: React.ReactNode;
};

export default function Exercise({ children }: ExerciseProps): JSX.Element {
  return (
    <div className={clsx('alert alert--warning', styles.admonition)}>
      <h4>Exercise</h4>
      <div>{children}</div>
    </div>
  );
}