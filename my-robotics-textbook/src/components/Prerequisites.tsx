import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

type PrerequisitesProps = {
  children: React.ReactNode;
};

export default function Prerequisites({ children }: PrerequisitesProps): JSX.Element {
  return (
    <div className={clsx('alert alert--info', styles.admonition)}>
      <h4>Prerequisites</h4>
      <div>{children}</div>
    </div>
  );
}