import type { Config } from '@docusaurus/types';
import {themes as prismThemes} from 'prism-react-renderer';

const config: Config = {
  title: 'Humanoid Robotics: A Practical Introduction',
  tagline: 'Physical AI & Humanoid Robotics',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://rizwanyaqoob.github.io',  // Replace with your actual domain
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub Pages deployment, it is often '/<projectName>/'
  baseUrl: '/PhysicalAI-HumanoidRobotics/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'Rizwanyaqoob', // Usually your GitHub org/user name.
  projectName: 'PhysicalAI-HumanoidRobotics', // Usually your repo name.

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'ignore', // Updated from deprecated config
  markdown: {
    mermaid: true,
  },

  // Environment variables for RAG integration
  customFields: {
    BACKEND_API_URL: process.env.BACKEND_API_URL || 'http://localhost:8009',
  },

  // Scripts to inject into the HTML
  scripts: [
    {
      src: '/rag-injector.js',
      defer: true,  // Use defer instead of async to ensure DOM is ready
    },
  ],

  // Inject BACKEND_API_URL as a global variable for client-side scripts
  headTags: [
    {
      tagName: 'script',
      attributes: {
        type: 'text/javascript',
      },
      innerHTML: `
        window.BACKEND_API_URL = '${process.env.BACKEND_API_URL || 'http://localhost:8009'}';
      `,
    },
  ],


  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/Rizwanyaqoob/Humanoid-Robotics-A-Practical-Introduction/tree/master/',
        },
        blog: false, // Disable blog if not needed
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    ({
      // Replace with your project's social card
      image: 'img/logo.svg', // Using existing logo as social card
      navbar: {
        title: 'Humanoid Robotics',
        logo: {
          alt: 'Robotics Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Documentation',
          },
          {
            href: 'https://github.com/Rizwanyaqoob/PhysicalAI-HumanoidRobotics',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Introduction',
                to: '/docs/docs/intro/',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/docusaurus',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/docusaurus',
              },
              {
                label: 'Twitter',
                href: 'https://twitter.com/docusaurus',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/facebook/docusaurus',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Humanoid Robotics Project. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
        additionalLanguages: ['python', 'bash', 'json', 'yaml', 'docker'],
      },
    }),
};

export default config;