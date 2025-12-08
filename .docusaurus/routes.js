import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/ai_spec_driven_book/__docusaurus/debug',
    component: ComponentCreator('/ai_spec_driven_book/__docusaurus/debug', '1db'),
    exact: true
  },
  {
    path: '/ai_spec_driven_book/__docusaurus/debug/config',
    component: ComponentCreator('/ai_spec_driven_book/__docusaurus/debug/config', '22b'),
    exact: true
  },
  {
    path: '/ai_spec_driven_book/__docusaurus/debug/content',
    component: ComponentCreator('/ai_spec_driven_book/__docusaurus/debug/content', '00a'),
    exact: true
  },
  {
    path: '/ai_spec_driven_book/__docusaurus/debug/globalData',
    component: ComponentCreator('/ai_spec_driven_book/__docusaurus/debug/globalData', '793'),
    exact: true
  },
  {
    path: '/ai_spec_driven_book/__docusaurus/debug/metadata',
    component: ComponentCreator('/ai_spec_driven_book/__docusaurus/debug/metadata', '265'),
    exact: true
  },
  {
    path: '/ai_spec_driven_book/__docusaurus/debug/registry',
    component: ComponentCreator('/ai_spec_driven_book/__docusaurus/debug/registry', 'e32'),
    exact: true
  },
  {
    path: '/ai_spec_driven_book/__docusaurus/debug/routes',
    component: ComponentCreator('/ai_spec_driven_book/__docusaurus/debug/routes', '6dc'),
    exact: true
  },
  {
    path: '/ai_spec_driven_book/',
    component: ComponentCreator('/ai_spec_driven_book/', '9f4'),
    routes: [
      {
        path: '/ai_spec_driven_book/',
        component: ComponentCreator('/ai_spec_driven_book/', '993'),
        routes: [
          {
            path: '/ai_spec_driven_book/',
            component: ComponentCreator('/ai_spec_driven_book/', '7d6'),
            routes: [
              {
                path: '/ai_spec_driven_book/capstone/',
                component: ComponentCreator('/ai_spec_driven_book/capstone/', '1c2'),
                exact: true
              },
              {
                path: '/ai_spec_driven_book/introduction',
                component: ComponentCreator('/ai_spec_driven_book/introduction', 'e94'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/module-0-foundations/',
                component: ComponentCreator('/ai_spec_driven_book/module-0-foundations/', '2fc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/module-1-ros2/',
                component: ComponentCreator('/ai_spec_driven_book/module-1-ros2/', 'd68'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/module-1-ros2/chapter-1-1-ros2-architecture',
                component: ComponentCreator('/ai_spec_driven_book/module-1-ros2/chapter-1-1-ros2-architecture', 'e64'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/module-1-ros2/chapter-1-2-first-ros2-node',
                component: ComponentCreator('/ai_spec_driven_book/module-1-ros2/chapter-1-2-first-ros2-node', '845'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/module-1-ros2/chapter-1-3-communication-patterns',
                component: ComponentCreator('/ai_spec_driven_book/module-1-ros2/chapter-1-3-communication-patterns', '76e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/module-1-ros2/chapter-1-4-message-types',
                component: ComponentCreator('/ai_spec_driven_book/module-1-ros2/chapter-1-4-message-types', '982'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/module-1-ros2/chapter-1-5-parameters',
                component: ComponentCreator('/ai_spec_driven_book/module-1-ros2/chapter-1-5-parameters', '290'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/module-1-ros2/chapter-1-6-launch-files',
                component: ComponentCreator('/ai_spec_driven_book/module-1-ros2/chapter-1-6-launch-files', '34a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/module-2-simulation/',
                component: ComponentCreator('/ai_spec_driven_book/module-2-simulation/', 'be5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/module-2-simulation/module-2-simu',
                component: ComponentCreator('/ai_spec_driven_book/module-2-simulation/module-2-simu', '772'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/module-3-isaac/',
                component: ComponentCreator('/ai_spec_driven_book/module-3-isaac/', 'd8f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/module-4-vla/',
                component: ComponentCreator('/ai_spec_driven_book/module-4-vla/', 'b19'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/',
                component: ComponentCreator('/ai_spec_driven_book/', '864'),
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
    path: '*',
    component: ComponentCreator('*'),
  },
];
