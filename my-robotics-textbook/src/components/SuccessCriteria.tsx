import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

type SuccessCriteriaProps = {
  children: React.ReactNode;
};

export default function SuccessCriteria({ children }: SuccessCriteriaProps): JSX.Element {
  return (
    <div className={clsx('alert alert--secondary', styles.admonition)}>
      <h4>Success Criteria</h4>
      <div>{children}</div>
    </div>
  );
}