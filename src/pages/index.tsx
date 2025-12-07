import React, { JSX } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

import styles from './index.module.css';
import RoboticsConceptCards from '../components/RoboticsConceptCard';

const HomepageHeader: React.FC = () => {
  const { siteConfig } = useDocusaurusContext();

  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/docs/foundations/physical-ai"
          >
            Get Started - 5 min ⏱️
          </Link>
          <Link
            className="button button--secondary button--lg"
            to="https://https://github.com/Rizwanyaqoob/PhysicalAI-HumanoidRobotics"
            target="_blank"
            rel="noopener noreferrer"
          >
            GitHub
          </Link>
        </div>
      </div>
    </header>
  );
};

export default function Home(): JSX.Element {
  useDocusaurusContext();

  return (
    <Layout>
      <HomepageHeader />
      <main>
        <RoboticsConceptCards />
      </main>
    </Layout>
  );
}

