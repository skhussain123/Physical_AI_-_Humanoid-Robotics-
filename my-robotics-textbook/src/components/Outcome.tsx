import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

type OutcomeProps = {
  children: React.ReactNode;
};

export default function Outcome({ children }: OutcomeProps): JSX.Element {
  return (
    <div className={clsx('alert alert--primary', styles.admonition)}>
      <h4>Learning Outcome</h4>
      <div>{children}</div>
    </div>
  );
}