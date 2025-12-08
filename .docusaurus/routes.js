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
    path: '/ai_spec_driven_book/docs',
    component: ComponentCreator('/ai_spec_driven_book/docs', '2eb'),
    routes: [
      {
        path: '/ai_spec_driven_book/docs',
        component: ComponentCreator('/ai_spec_driven_book/docs', 'd93'),
        routes: [
          {
            path: '/ai_spec_driven_book/docs',
            component: ComponentCreator('/ai_spec_driven_book/docs', '275'),
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
                component: ComponentCreator('/ai_spec_driven_book/docs/module-1-ros2/chapter-1-1-ros2-architecture', '505'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-1-ros2/chapter-1-2-first-ros2-node',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-1-ros2/chapter-1-2-first-ros2-node', 'b91'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-1-ros2/chapter-1-3-communication-patterns',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-1-ros2/chapter-1-3-communication-patterns', 'b3d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-1-ros2/chapter-1-4-message-types',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-1-ros2/chapter-1-4-message-types', 'f0e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-1-ros2/chapter-1-5-parameters',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-1-ros2/chapter-1-5-parameters', '2f6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-1-ros2/chapter-1-6-launch-files',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-1-ros2/chapter-1-6-launch-files', '884'),
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
                path: '/ai_spec_driven_book/docs/module-2-simulation/chapter-1',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-2-simulation/chapter-1', 'e8e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_spec_driven_book/docs/module-2-simulation/chapter-2',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-2-simulation/chapter-2', '81b'),
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
                path: '/ai_spec_driven_book/docs/module-4-vla/',
                component: ComponentCreator('/ai_spec_driven_book/docs/module-4-vla/', 'f63'),
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
