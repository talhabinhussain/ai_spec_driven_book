---
title: Home
hide_table_of_contents: true
---

import HomepageFeatures from '@site/src/components/HomepageFeatures';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './home.module.css';

<div className={styles.heroSection}>
  <div className={styles.heroContent}>
    <div className={styles.logoContainer}>
      <img src={useBaseUrl('/img/logo.svg')} alt="Physical AI Textbook" className={styles.heroLogo} />
    </div>
    <h1 className={styles.heroTitle}>
      Physical AI & Humanoid Robotics Textbook
    </h1>
    <p className={styles.heroSubtitle}>
      Bridging digital AI models with physical humanoid robots
    </p>
    <p className={styles.heroDescription}>
      Welcome to the comprehensive textbook that covers essential concepts from ROS 2 architecture to Vision-Language-Action (VLA) robotic systems.
    </p>
    <div className={styles.ctaSection}>
      <a href={useBaseUrl('/docs/introduction')} className={styles.ctaPrimary}>
        Get Started Now
      </a>
      <a href={useBaseUrl('/docs/module-0-foundations/')} className={styles.ctaSecondary}>
        Browse Modules
      </a>
    </div>
  </div>
</div>

<HomepageFeatures />

<div className={styles.whyStudySection}>
  <div className="container">
    <h2 className={styles.sectionTitle}>Why Study Physical AI & Robotics?</h2>
    <p className={styles.sectionSubtitle}>
      This textbook provides a comprehensive learning experience designed for both beginners and advanced practitioners
    </p>

    <div className={styles.benefitsGrid}>
      <div className={styles.benefitCard}>
        <div className={styles.benefitIcon}>📚</div>
        <h3 className={styles.benefitTitle}>Theoretical Explanations</h3>
        <p className={styles.benefitDescription}>
          Deep dive into core robotics concepts with clear, detailed explanations backed by research
        </p>
      </div>

      <div className={styles.benefitCard}>
        <div className={styles.benefitIcon}>💻</div>
        <h3 className={styles.benefitTitle}>Hands-on Code Labs</h3>
        <p className={styles.benefitDescription}>
          Practice with real-world coding exercises and implementation examples
        </p>
      </div>

      <div className={styles.benefitCard}>
        <div className={styles.benefitIcon}>🎮</div>
        <h3 className={styles.benefitTitle}>Simulation Exercises</h3>
        <p className={styles.benefitDescription}>
          Experiment safely in virtual environments before deploying to hardware
        </p>
      </div>

      <div className={styles.benefitCard}>
        <div className={styles.benefitIcon}>✅</div>
        <h3 className={styles.benefitTitle}>Assessment Components</h3>
        <p className={styles.benefitDescription}>
          Verify your understanding with quizzes, challenges, and projects
        </p>
      </div>

      <div className={styles.benefitCard}>
        <div className={styles.benefitIcon}>🚀</div>
        <h3 className={styles.benefitTitle}>Capstone Project</h3>
        <p className={styles.benefitDescription}>
          Integrate all concepts in a comprehensive end-to-end robotics project
        </p>
      </div>

      <div className={styles.benefitCard}>
        <div className={styles.benefitIcon}>🌟</div>
        <h3 className={styles.benefitTitle}>Industry Relevant</h3>
        <p className={styles.benefitDescription}>
          Learn technologies and practices used in cutting-edge robotics companies
        </p>
      </div>
    </div>

  </div>
</div>
