import React from 'react';
import clsx from 'clsx';
import styles from './Chapter.module.css';

interface ChapterProps {
  title: string;
  content: string;
  chapterNumber?: number;
  moduleTitle?: string;
  children?: React.ReactNode;
}

const Chapter: React.FC<ChapterProps> = ({
  title,
  content,
  chapterNumber,
  moduleTitle,
  children
}) => {
  return (
    <div className={clsx('container', styles.chapterContainer)}>
      <header className={styles.chapterHeader}>
        {moduleTitle && (
          <span className={styles.moduleTitle}>{moduleTitle}</span>
        )}
        <h1 className={styles.chapterTitle}>
          {chapterNumber && <span className={styles.chapterNumber}>Chapter {chapterNumber}: </span>}
          {title}
        </h1>
      </header>

      <div className={styles.chapterContent}>
        <div
          className={styles.contentText}
          dangerouslySetInnerHTML={{ __html: content }}
        />

        {children && (
          <div className={styles.chapterComponents}>
            {children}
          </div>
        )}
      </div>

      <footer className={styles.chapterFooter}>
        <div className={styles.navigation}>
          <button className={clsx('button button--secondary', styles.navButton)}>
            Previous
          </button>
          <button className={clsx('button button--primary', styles.navButton)}>
            Next
          </button>
        </div>
      </footer>
    </div>
  );
};

export default Chapter;