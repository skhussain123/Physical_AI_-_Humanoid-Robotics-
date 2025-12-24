import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Get Started with Robotics - 5min ⏱️
          </Link>
          <Link
            className="button button--primary button--lg"
            to="/docs/module1/chapter1">
            Explore Textbook
          </Link>
        </div>
      </div>
    </header>
  );
}

function FeaturesSection() {
  return (
    <section className={styles.featuresSection}>
      <div className="container">
        <Heading as="h2">Key Learning Areas</Heading>
        <div className={styles.featureRow}>
          <div className={styles.featureCard}>
            <div className={styles.featureIcon}>🤖</div>
            <Heading as="h3">Physical AI</Heading>
            <p>Learn how artificial intelligence integrates with physical systems and robotics to create intelligent autonomous agents.</p>
          </div>
          <div className={styles.featureCard}>
            <div className={styles.featureIcon}>🦾</div>
            <Heading as="h3">Humanoid Robotics</Heading>
            <p>Explore the principles behind designing and controlling humanoid robots with advanced mobility and interaction capabilities.</p>
          </div>
          <div className={styles.featureCard}>
            <div className={styles.featureIcon}>🔧</div>
            <Heading as="h3">ROS & Simulation</Heading>
            <p>Master Robot Operating System (ROS) and simulation environments like Gazebo and Unity for robotics development.</p>
          </div>
        </div>
        <div className={styles.featureRow}>
          <div className={styles.featureCard}>
            <div className={styles.featureIcon}>🌐</div>
            <Heading as="h3">Interactive Learning</Heading>
            <p>Engage with hands-on exercises, simulations, and real-world robotics challenges to reinforce theoretical concepts.</p>
          </div>
          <div className={styles.featureCard}>
            <div className={styles.featureIcon}>📚</div>
            <Heading as="h3">Comprehensive Curriculum</Heading>
            <p>Follow a structured learning path from fundamentals to advanced topics in robotics and AI integration.</p>
          </div>
          <div className={styles.featureCard}>
            <div className={styles.featureIcon}>🚀</div>
            <Heading as="h3">Industry Ready</Heading>
            <p>Prepare for real-world robotics applications with practical skills and industry-standard tools and practices.</p>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="An AI-Native Interactive Learning Experience for Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        <FeaturesSection />
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
