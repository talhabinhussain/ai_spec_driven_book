# Docusaurus Physical AI & Humanoid Robotics Book - Issue Resolution Guide

## Project Overview

- **Tech Stack**: Docusaurus with TypeScript
- **Tools**: Claude Code, Speckit-Plus
- **Topic**: Physical AI and Humanoid Robotics

---

## Current Issues

### Issue 1: Course Module Sidebar Appearing on Home Page

**Problem**: The course module sidebar is rendering on the homepage when it should only appear on documentation/course pages.

**Root Cause**: Likely misconfiguration in `docusaurus.config.ts` or theme configuration causing the docs sidebar to display globally.

**Solution**:

1. **Check `docusaurus.config.ts`** - Ensure sidebar configuration is scoped correctly:

```typescript
// docusaurus.config.ts
export default {
  themeConfig: {
    docs: {
      sidebar: {
        hideable: true,
        autoCollapseCategories: true,
      },
    },
  },
  presets: [
    [
      "classic",
      {
        docs: {
          sidebarPath: "./sidebars.ts",
          routeBasePath: "docs", // Ensure this is NOT '/'
        },
      },
    ],
  ],
};
```

2. **Verify Homepage Component** (`src/pages/index.tsx`):

```typescript
// src/pages/index.tsx
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();

  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Physical AI and Humanoid Robotics"
      noFooter={false}
      wrapperClassName="homepage" // Custom class to target
    >
      {/* Your homepage content */}
    </Layout>
  );
}
```

3. **Add Custom CSS** to hide sidebar on homepage (`src/css/custom.css`):

```css
/* Hide sidebar on homepage */
.homepage aside.theme-doc-sidebar-container {
  display: none !important;
}

.homepage .main-wrapper {
  max-width: 100%;
}
```

---

### Issue 2: Homepage Buttons Not Working - "Start Learning" Redirects to 404

**Problem**: Navigation buttons on the homepage are broken, leading to "Page Not Found" errors.

**Root Cause**: Incorrect routing paths or missing documentation structure.

**Solution**:

1. **Fix Button Links** in `src/pages/index.tsx`:

```typescript
import Link from '@docusaurus/Link';

function HomepageHeader() {
  return (
    <header>
      <div className="container">
        <h1>Physical AI & Humanoid Robotics</h1>
        <div className="buttons">
          <Link
            className="button button--primary button--lg"
            to="/docs/intro"> {/* Correct path to first doc */}
            Start Learning
          </Link>
          <Link
            className="button button--secondary button--lg"
            to="/docs/module-1/chapter-1"> {/* Or specific chapter */}
            Get Started
          </Link>
        </div>
      </div>
    </header>
  );
}
```

2. **Verify Documentation Structure** - Ensure this file exists:

```
docs/
├── intro.md              ← Must exist for /docs/intro
├── module-1/
│   ├── _category_.json
│   ├── chapter-1.md
│   ├── chapter-2.md
│   └── chapter-3.md
├── module-2/
│   ├── _category_.json
│   ├── chapter-1.md
│   └── chapter-2.md
└── ...
```

3. **Create `docs/intro.md`** if missing:

```markdown
---
id: intro
title: Introduction
sidebar_position: 1
slug: /intro
---

# Welcome to Physical AI & Humanoid Robotics

This comprehensive course covers...

## What You'll Learn

- Fundamentals of Physical AI
- Humanoid robot design principles
- Advanced control systems
- Real-world applications

[Get Started with Module 1 →](/docs/module-1/chapter-1)
```

4. **Update `sidebars.ts`** to ensure proper routing:

```typescript
// sidebars.ts
import type { SidebarsConfig } from "@docusaurus/plugin-content-docs";

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: "doc",
      id: "intro",
      label: "Introduction",
    },
    {
      type: "category",
      label: "Module 1: Fundamentals",
      link: {
        type: "generated-index",
        title: "Module 1: Fundamentals",
        description: "Learn the basics of Physical AI",
      },
      items: ["module-1/chapter-1", "module-1/chapter-2", "module-1/chapter-3"],
    },
    // ... more modules
  ],
};

export default sidebars;
```

---

### Issue 3: Sidebar Module Hierarchy with Collapsible Chapters

**Problem**: Need proper hierarchical organization where modules contain chapters with dropdown/collapsible functionality.

**Required Structure**:

```
├── Module 1: Fundamentals (Collapsible)
│   ├── Chapter 1: Introduction to Physical AI
│   ├── Chapter 2: Core Concepts
│   └── Chapter 3: Prerequisites
├── Module 2: Hardware Design (Collapsible)
│   ├── Chapter 1: Actuators
│   ├── Chapter 2: Sensors
│   └── Chapter 3: Power Systems
└── Module 3: Control Systems (Collapsible)
    ├── Chapter 1: Kinematics
    └── Chapter 2: Dynamics
```

**Solution**:

1. **Organize File Structure**:

```
docs/
├── intro.md
├── module-1-fundamentals/
│   ├── _category_.json
│   ├── chapter-1-intro-physical-ai.md
│   ├── chapter-2-core-concepts.md
│   └── chapter-3-prerequisites.md
├── module-2-hardware-design/
│   ├── _category_.json
│   ├── chapter-1-actuators.md
│   ├── chapter-2-sensors.md
│   └── chapter-3-power-systems.md
└── module-3-control-systems/
    ├── _category_.json
    ├── chapter-1-kinematics.md
    └── chapter-2-dynamics.md
```

2. **Configure `_category_.json`** for each module:

```json
// docs/module-1-fundamentals/_category_.json
{
  "label": "Module 1: Fundamentals",
  "position": 2,
  "link": {
    "type": "generated-index",
    "description": "Learn the fundamental concepts of Physical AI and robotics."
  },
  "collapsed": true,
  "collapsible": true
}
```

```json
// docs/module-2-hardware-design/_category_.json
{
  "label": "Module 2: Hardware Design",
  "position": 3,
  "link": {
    "type": "generated-index",
    "description": "Explore hardware components and design principles."
  },
  "collapsed": true,
  "collapsible": true
}
```

3. **Add Frontmatter to Each Chapter**:

```markdown
---
id: chapter-1-intro-physical-ai
title: Chapter 1 - Introduction to Physical AI
sidebar_position: 1
sidebar_label: "Ch1: Intro to Physical AI"
---

# Introduction to Physical AI

Content here...
```

4. **Update `sidebars.ts`** for Auto-Generation:

```typescript
// sidebars.ts
import type { SidebarsConfig } from "@docusaurus/plugin-content-docs";

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: "doc",
      id: "intro",
      label: "Course Introduction",
    },
    {
      type: "category",
      label: "Module 1: Fundamentals",
      collapsible: true,
      collapsed: true,
      link: {
        type: "generated-index",
        title: "Module 1: Fundamentals",
        description: "Master the foundational concepts of Physical AI",
        slug: "/module-1",
      },
      items: [
        "module-1-fundamentals/chapter-1-intro-physical-ai",
        "module-1-fundamentals/chapter-2-core-concepts",
        "module-1-fundamentals/chapter-3-prerequisites",
      ],
    },
    {
      type: "category",
      label: "Module 2: Hardware Design",
      collapsible: true,
      collapsed: true,
      link: {
        type: "generated-index",
        title: "Module 2: Hardware Design",
        description: "Deep dive into robotics hardware",
        slug: "/module-2",
      },
      items: [
        "module-2-hardware-design/chapter-1-actuators",
        "module-2-hardware-design/chapter-2-sensors",
        "module-2-hardware-design/chapter-3-power-systems",
      ],
    },
    {
      type: "category",
      label: "Module 3: Control Systems",
      collapsible: true,
      collapsed: true,
      link: {
        type: "generated-index",
        title: "Module 3: Control Systems",
        description: "Learn advanced control techniques",
        slug: "/module-3",
      },
      items: [
        "module-3-control-systems/chapter-1-kinematics",
        "module-3-control-systems/chapter-2-dynamics",
      ],
    },
  ],
};

export default sidebars;
```

---

## Additional Configuration for Better UX

### Enable Sidebar Auto-Collapse

```typescript
// docusaurus.config.ts
export default {
  themeConfig: {
    docs: {
      sidebar: {
        hideable: true,
        autoCollapseCategories: true, // Collapse other categories when one opens
      },
    },
  },
};
```

### Add Custom Styling for Sidebar

```css
/* src/css/custom.css */

/* Module headers in sidebar */
.theme-doc-sidebar-item-category-level-1 > .menu__list-item-collapsible {
  font-weight: 600;
  font-size: 1.05rem;
  color: var(--ifm-color-primary);
}

/* Chapter items */
.theme-doc-sidebar-item-link-level-2 {
  font-size: 0.95rem;
  padding-left: 1.5rem;
}

/* Highlight active chapter */
.menu__link--active {
  background-color: var(--ifm-color-primary-lightest);
  border-left: 3px solid var(--ifm-color-primary);
}
```

---

## Verification Checklist

After implementing fixes, verify:

- [ ] Homepage loads without sidebar visible
- [ ] "Start Learning" button redirects to `/docs/intro`
- [ ] All module buttons/links work correctly
- [ ] Sidebar appears only on documentation pages
- [ ] Modules are collapsible/expandable
- [ ] Chapters are properly nested under modules
- [ ] Navigation between chapters works smoothly
- [ ] Active chapter is highlighted in sidebar
- [ ] Auto-collapse works (opening one module collapses others)
- [ ] Mobile responsive sidebar works correctly

---

## Testing Commands

```bash
# Clear cache and rebuild
npm run clear
npm run build

# Test locally
npm run start

# Check for broken links
npm run build && npx serve build
```

---

## Common Troubleshooting

### If sidebar still appears on homepage:

1. Clear browser cache
2. Run `npm run clear`
3. Check for conflicting CSS
4. Verify `routeBasePath` is not `/` in config

### If 404 errors persist:

1. Check file paths match exactly (case-sensitive)
2. Verify all `.md` files have proper frontmatter with `id`
3. Ensure `sidebars.ts` references match file IDs
4. Check `slug` values don't conflict

### If collapsible doesn't work:

1. Ensure `collapsible: true` and `collapsed: true` in config
2. Update to latest Docusaurus version: `npm update @docusaurus/core`
3. Check `autoCollapseCategories` is enabled

---

## Expected Final Behavior

1. **Homepage**: Clean landing page with working CTA buttons, no sidebar
2. **Course Pages**: Full sidebar with hierarchical module/chapter structure
3. **Navigation**: Smooth transitions between chapters with proper URLs
4. **Sidebar UX**: Collapsible modules, auto-collapse, active state highlighting

---

## File Reference Summary

Key files to check/modify:

- `docusaurus.config.ts` - Main configuration
- `sidebars.ts` - Sidebar structure definition
- `src/pages/index.tsx` - Homepage component
- `src/css/custom.css` - Custom styling
- `docs/intro.md` - Entry point document
- `docs/*/_category_.json` - Module metadata

---

_This documentation should be provided to Claude Code for systematic resolution of all three issues._
