import React from "react";
import clsx from "clsx";
import styles from "./HomepageFeatures.module.css";
import Link from "@docusaurus/Link";

type FeatureItem = {
  title: string;
  description: JSX.Element;
  link: string;
  icon: string;
};

const FeatureList: FeatureItem[] = [
  {
    title: "Physical AI Foundations",
    description: (
      <>
        Understand the principles of embodied intelligence and
        digital-to-physical transitions. Learn about sensor grounding and
        perception loops.
      </>
    ),
    link: "/docs/module-0-foundations",
    icon: "🤖",
  },
  {
    title: "ROS 2 Architecture",
    description: (
      <>
        Build robotic nervous systems with nodes, topics, and services. Master
        the Robot Operating System for robotics development.
      </>
    ),
    link: "/docs/module-1-ros2",
    icon: "⚙️",
  },
  {
    title: "Simulation Environments",
    description: (
      <>
        Create digital twins with Gazebo and Unity. Learn to develop and test
        robots in safe simulation environments.
      </>
    ),
    link: "/docs/module-2-simulation",
    icon: "🌐",
  },
  {
    title: "NVIDIA Isaac",
    description: (
      <>
        Implement perception and navigation pipelines. Use Isaac Sim for
        photorealistic simulation and synthetic data generation.
      </>
    ),
    link: "/docs/module-3-isaac",
    icon: "🎯",
  },
  {
    title: "Vision-Language-Action Robotics",
    description: (
      <>
        Build conversational robotic systems using LLMs. Create
        Voice→Language→Action (VLA) robotic pipelines.
      </>
    ),
    link: "/docs/module-4-vla",
    icon: "💬",
  },
  {
    title: "Capstone Project",
    description: (
      <>
        Integrate all concepts in a complete humanoid robot system. Apply
        knowledge from all modules in a comprehensive project.
      </>
    ),
    link: "/docs/capstone/",
    icon: "🎓",
  },
];

function Feature({ title, description, link, icon }: FeatureItem) {
  return (
    <div className={clsx("col col--4", styles.featureCol)}>
      <div className={styles.featureCard}>
        <div className={styles.iconWrapper}>
          <span className={styles.icon}>{icon}</span>
        </div>
        <h3 className={styles.featureTitle}>{title}</h3>
        <p className={styles.featureDescription}>{description}</p>
        <Link className={styles.featureButton} to={link}>
          Learn More →
        </Link>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): JSX.Element {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.header}>
          <h2 className={styles.mainTitle}>
            Master Physical AI & Humanoid Robotics
          </h2>
          <p className={styles.subtitle}>
            From foundations to deployment - learn to build intelligent robotic
            systems
          </p>
          <div className={styles.ctaButtons}>
            <Link
              className={clsx(
                "button button--primary button--lg",
                styles.primaryBtn
              )}
              to="/docs/introduction"
            >
              Start Learning
            </Link>
            <Link
              className={clsx(
                "button button--outline button--lg",
                styles.outlineBtn
              )}
              to="/docs/module-0-foundations"
            >
              Course Modules
            </Link>
          </div>
        </div>

        <div className={styles.modulesSection}>
          <h3 className={styles.modulesTitle}>Course Modules</h3>
          <div className="row">
            {FeatureList.map((props, idx) => (
              <Feature key={idx} {...props} />
            ))}
          </div>
        </div>
      </div>
    </section>
  );
}
