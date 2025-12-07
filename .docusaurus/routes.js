import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/ai_native_book/__docusaurus/debug',
    component: ComponentCreator('/ai_native_book/__docusaurus/debug', '381'),
    exact: true
  },
  {
    path: '/ai_native_book/__docusaurus/debug/config',
    component: ComponentCreator('/ai_native_book/__docusaurus/debug/config', '2b9'),
    exact: true
  },
  {
    path: '/ai_native_book/__docusaurus/debug/content',
    component: ComponentCreator('/ai_native_book/__docusaurus/debug/content', '221'),
    exact: true
  },
  {
    path: '/ai_native_book/__docusaurus/debug/globalData',
    component: ComponentCreator('/ai_native_book/__docusaurus/debug/globalData', 'd19'),
    exact: true
  },
  {
    path: '/ai_native_book/__docusaurus/debug/metadata',
    component: ComponentCreator('/ai_native_book/__docusaurus/debug/metadata', '44a'),
    exact: true
  },
  {
    path: '/ai_native_book/__docusaurus/debug/registry',
    component: ComponentCreator('/ai_native_book/__docusaurus/debug/registry', 'c71'),
    exact: true
  },
  {
    path: '/ai_native_book/__docusaurus/debug/routes',
    component: ComponentCreator('/ai_native_book/__docusaurus/debug/routes', '543'),
    exact: true
  },
  {
    path: '/ai_native_book/',
    component: ComponentCreator('/ai_native_book/', '082'),
    routes: [
      {
        path: '/ai_native_book/',
        component: ComponentCreator('/ai_native_book/', '0e4'),
        routes: [
          {
            path: '/ai_native_book/',
            component: ComponentCreator('/ai_native_book/', 'd2f'),
            routes: [
              {
                path: '/ai_native_book/capstone/',
                component: ComponentCreator('/ai_native_book/capstone/', '3b9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_native_book/introduction',
                component: ComponentCreator('/ai_native_book/introduction', '60a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_native_book/module-0-foundations/',
                component: ComponentCreator('/ai_native_book/module-0-foundations/', '801'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_native_book/module-1-ros2/',
                component: ComponentCreator('/ai_native_book/module-1-ros2/', '30d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_native_book/module-1-ros2/chapter-1-1-ros2-architecture',
                component: ComponentCreator('/ai_native_book/module-1-ros2/chapter-1-1-ros2-architecture', '772'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_native_book/module-1-ros2/chapter-1-2-first-ros2-node',
                component: ComponentCreator('/ai_native_book/module-1-ros2/chapter-1-2-first-ros2-node', '4c8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_native_book/module-1-ros2/chapter-1-3-communication-patterns',
                component: ComponentCreator('/ai_native_book/module-1-ros2/chapter-1-3-communication-patterns', 'efb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_native_book/module-1-ros2/chapter-1-4-message-types',
                component: ComponentCreator('/ai_native_book/module-1-ros2/chapter-1-4-message-types', '6fc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_native_book/module-1-ros2/chapter-1-5-parameters',
                component: ComponentCreator('/ai_native_book/module-1-ros2/chapter-1-5-parameters', '392'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_native_book/module-1-ros2/chapter-1-6-launch-files',
                component: ComponentCreator('/ai_native_book/module-1-ros2/chapter-1-6-launch-files', '9fa'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_native_book/module-2-simulation/',
                component: ComponentCreator('/ai_native_book/module-2-simulation/', '736'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_native_book/module-2-simulation/module-2-simu',
                component: ComponentCreator('/ai_native_book/module-2-simulation/module-2-simu', 'fac'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_native_book/module-3-isaac/',
                component: ComponentCreator('/ai_native_book/module-3-isaac/', 'b6f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_native_book/module-4-vla/',
                component: ComponentCreator('/ai_native_book/module-4-vla/', '6e6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ai_native_book/',
                component: ComponentCreator('/ai_native_book/', 'e0b'),
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
