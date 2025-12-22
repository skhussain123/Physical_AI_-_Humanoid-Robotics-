import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

type CheckpointProps = {
  children: React.ReactNode;
};

export default function Checkpoint({ children }: CheckpointProps): JSX.Element {
  return (
    <div className={clsx('alert alert--info', styles.admonition)}>
      <h4>Checkpoint</h4>
      <div>{children}</div>
    </div>
  );
}