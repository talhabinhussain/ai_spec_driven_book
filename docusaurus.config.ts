import { themes as prismThemes } from "prism-react-renderer";

// With JSDoc @type annotations, IDEs can provide config autocompletion
/** @type {import('@docusaurus/types').Config} */
const config = {
  title: "Physical AI & Humanoid Robotics Textbook",
  tagline: "Bridging digital AI models with physical humanoid robots",
  favicon: "img/favicon.ico",

  // Set the production url of your site here
  url: "https://talhabinhussain.github.io",
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: "/ai_spec_driven_book/",

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: " talhabinhussain", // Usually your GitHub org/user name.
  projectName: "ai_spec_driven_book", // Usually your repo name.
  deploymentBranch: "gh-pages", // Branch that GitHub Pages will deploy from.

  onBrokenLinks: "warn",
  onBrokenMarkdownLinks: "warn",

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: "en",
    locales: ["en"],
  },

  presets: [
    [
      "classic",
      /** @type {import('@docusaurus/preset-classic').Options} */
      {
        docs: {
          sidebarPath: "./sidebars.js",
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            "https://github.com/talhabinhussain/ai_spec_driven_book/tree/main/",
          routeBasePath: "/docs", // Serve the docs at /docs path
        },
        blog: {
          showReadingTime: true,
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            "https://github.com/talhabinhussain/ai_spec_driven_book/tree/main/",
        },
        theme: {
          customCss: "./src/css/custom.css",
        },
      },
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    {
      // Replace with your project's social card
      image: "img/docusaurus-social-card.jpg",
      docs: {
        sidebar: {
          hideable: true,
          autoCollapseCategories: true,
        },
      },
      navbar: {
        title: "Physical AI Textbook",
        logo: {
          alt: "Physical AI & Humanoid Robotics Textbook Logo",
          src: "img/logo.svg",
          href: "/",
        },
        items: [
          {
            type: "docSidebar",
            sidebarId: "tutorialSidebar",
            position: "left",
            label: "Textbook",
          },
          {
            href: "/docs",
            position: "left",
            label: "Home",
          },
          {
            href: "https://github.com/talhabinhussain/ai_spec_driven_book",
            label: "GitHub",
            position: "right",
          },
        ],
      },
      footer: {
        style: "dark",
        links: [
          {
            title: "Modules",
            items: [
              {
                label: "Physical AI Foundations",
                to: "/docs/module-0-foundations",
              },
              {
                label: "ROS 2 Architecture",
                to: "/docs/module-1-ros2",
              },
              {
                label: "Simulation Environments",
                to: "/docs/module-2-simulation",
              },
              {
                label: "NVIDIA Isaac",
                to: "/docs/module-3-isaac",
              },
              {
                label: "VLA Robotics",
                to: "/docs/module-4-vla",
              },
            ],
          },
          {
            title: "Community",
            items: [
              {
                label: "Stack Overflow",
                href: "https://stackoverflow.com/questions/tagged/docusaurus",
              },
              {
                label: "Discord",
                href: "https://discordapp.com/invite/docusaurus",
              },
              {
                label: "Twitter",
                href: "https://twitter.com/docusaurus",
              },
            ],
          },
          {
            title: "More",
            items: [
              {
                label: "GitHub",
                href: "https://github.com/talhabinhussain/ai_spec_driven_book",
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    },
};

export default config;
