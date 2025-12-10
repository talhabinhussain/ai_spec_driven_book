import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/ai_spec_driven_book/docs',
    component: ComponentCreator('/ai_spec_driven_book/docs', 'eab'),
    routes: [
      {
        path: '/ai_spec_driven_book/docs',
        component: ComponentCreator('/ai_spec_driven_book/docs', '51a'),
        routes: [
          {
            path: '/ai_spec_driven_book/docs',
            component: ComponentCreator('/ai_spec_driven_book/docs', 'd8b'),
            routes: [
              {
                path: '/ai_spec_driven_book/docs/',
                component: ComponentCreator('/ai_spec_driven_book/docs/', 'b69'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/capstone/',
                component: ComponentCreator('/ai_spec_driven_book/docs/capstone/', '011'),
                exact: true
              },
              {
                path: '/ai_spec_driven_book/docs/intro',
                component: ComponentCreator('/ai_spec_driven_book/docs/intro', '1b5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/introduction',
                component: ComponentCreator('/ai_spec_driven_book/docs/introduction', '2d3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-0-foundations/',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-0-foundations/', '193'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-1-ros2/',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-1-ros2/', 'b07'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-1-ros2/chapter-1-1-ros2-architecture',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-1-ros2/chapter-1-1-ros2-architecture', '096'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-1-ros2/chapter-1-2-first-ros2-node',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-1-ros2/chapter-1-2-first-ros2-node', '902'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-1-ros2/chapter-1-3-communication-patterns',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-1-ros2/chapter-1-3-communication-patterns', '13f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-1-ros2/chapter-1-4-message-types',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-1-ros2/chapter-1-4-message-types', 'f99'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-1-ros2/chapter-1-5-parameters',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-1-ros2/chapter-1-5-parameters', '79e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-1-ros2/chapter-1-6-launch-files',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-1-ros2/chapter-1-6-launch-files', 'd7d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-2-simulation/',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-2-simulation/', 'bc3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-2-simulation/chapter-2-1-introduction-to-physics-simulation',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-2-simulation/chapter-2-1-introduction-to-physics-simulation', '256'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-2-simulation/chapter-2-2-getting-started-with-gazebo',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-2-simulation/chapter-2-2-getting-started-with-gazebo', 'bd7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-2-simulation/chapter-2-3-physics-configuration-and-tuning',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-2-simulation/chapter-2-3-physics-configuration-and-tuning', '9ad'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-2-simulation/chapter-2-4-sensor-simulation-in-gazebo',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-2-simulation/chapter-2-4-sensor-simulation-in-gazebo', '2da'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-2-simulation/chapter-2-5-unity-for-robot-visualization',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-2-simulation/chapter-2-5-unity-for-robot-visualization', '062'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-2-simulation/chapter-2-6-building-complex-environments',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-2-simulation/chapter-2-6-building-complex-environments', 'bb7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-3-isaac/',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-3-isaac/', 'b4d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-3-isaac/chapter-3-1-isaac-ecosystem',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-3-isaac/chapter-3-1-isaac-ecosystem', 'f8f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-3-isaac/chapter-3-2-isaac-sim-basics',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-3-isaac/chapter-3-2-isaac-sim-basics', 'fb8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-3-isaac/chapter-3-3-photorealistic-rendering',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-3-isaac/chapter-3-3-photorealistic-rendering', 'af4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-3-isaac/chapter-3-4-robot-training',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-3-isaac/chapter-3-4-robot-training', 'd25'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-3-isaac/chapter-3-5-isaac-ros',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-3-isaac/chapter-3-5-isaac-ros', 'f77'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-4-vla/',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-4-vla/', 'f63'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-4-vla/chapter-4-1-vla-intro',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-4-vla/chapter-4-1-vla-intro', 'f73'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-4-vla/chapter-4-2-whisper-speech',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-4-vla/chapter-4-2-whisper-speech', '559'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-4-vla/chapter-4-3-llm-planning',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-4-vla/chapter-4-3-llm-planning', 'a1b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-4-vla/chapter-4-4-vision-language',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-4-vla/chapter-4-4-vision-language', 'cae'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-4-vla/chapter-4-5-action-primitives',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-4-vla/chapter-4-5-action-primitives', 'c28'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-4-vla/chapter-4-6-conversational-robots',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-4-vla/chapter-4-6-conversational-robots', 'cf1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-4-vla/chapter-4-7-capstone-project',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-4-vla/chapter-4-7-capstone-project', 'b42'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/ai_spec_driven_book/',
    component: ComponentCreator('/ai_spec_driven_book/', 'f8d'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
