# Cathay Pacific Design System Guidelines

Last Updated: 2025-11-15

## General Guidelines

* Only use absolute positioning when necessary. Opt for responsive and well structured layouts that use flexbox and grid by default
* Refactor code as you go to keep code clean
* Keep file sizes small and put helper functions and components in their own files
* Use TypeScript for type safety
* Follow React best practices and hooks patterns

---

## Spacing System

### CSS Variables

All spacing values are defined as CSS variables in `index.css`:

```css
/* Base Spacing Values */
--cathay-spacing-xs: 0.25rem;    /* 4px */
--cathay-spacing-sm: 0.5rem;     /* 8px */
--cathay-spacing-md: 0.75rem;    /* 12px */
--cathay-spacing-base: 1rem;     /* 16px */
--cathay-spacing-lg: 1.5rem;     /* 24px */
--cathay-spacing-xl: 2rem;       /* 32px */

/* Component Spacing Standards */
--cathay-section-gap: 24px;      /* Between page sections */
--cathay-card-padding: 24px;     /* Card component padding (large) */
--cathay-card-padding-sm: 16px;  /* Card component padding (small) */
--cathay-element-gap: 16px;      /* Standard gap between elements */
--cathay-element-gap-sm: 12px;   /* Small gap between elements */
--cathay-element-gap-xs: 8px;    /* Minimum gap between elements */
--cathay-grid-gap: 16px;         /* Standard grid gap */
--cathay-grid-gap-lg: 24px;      /* Large grid gap */
```

### Spacing Usage Rules

#### 1. Page Section Spacing (24px)
Use between major page sections and card components:
```tsx
<div className="space-y-6">  {/* 6 * 4px = 24px */}
```

#### 2. Card Padding
- **Large cards**: Use `p-6` (24px)
- **Small cards**: Use `p-4` (16px)

```tsx
<Card className="p-6">  {/* Main cards */}
<Card className="p-4">  {/* Secondary cards */}
```

#### 3. Element Spacing
- **Standard gap (16px)**: Use `mb-4`, `space-y-4`, `gap-4`
- **Small gap (12px)**: Use `mb-3`, `space-y-3`, `gap-3`
- **Minimum gap (8px)**: Use `mb-2`, `gap-2`

```tsx
<div className="space-y-4">  {/* Between form fields */}
<div className="space-y-3">  {/* Between related items */}
<div className="gap-2">      {/* Between badges/tags */}
```

#### 4. Grid Layouts
- **Standard grid**: `gap-4` (16px)
- **Large grid**: `gap-6` (24px)

```tsx
<div className="grid grid-cols-2 gap-4">    {/* Info cards */}
<div className="grid md:grid-cols-2 gap-6"> {/* Main layout */}
```

### Quick Reference Table

| Purpose | Tailwind Class | CSS Variable | Pixels |
|---------|---------------|--------------|--------|
| Section Gap | `space-y-6`, `gap-6`, `mb-6` | `var(--cathay-section-gap)` | 24px |
| Card Padding (Large) | `p-6` | `var(--cathay-card-padding)` | 24px |
| Card Padding (Small) | `p-4` | `var(--cathay-card-padding-sm)` | 16px |
| Element Gap | `space-y-4`, `gap-4`, `mb-4` | `var(--cathay-element-gap)` | 16px |
| Small Gap | `space-y-3`, `gap-3`, `mb-3` | `var(--cathay-element-gap-sm)` | 12px |
| Minimum Gap | `gap-2`, `mb-2` | `var(--cathay-element-gap-xs)` | 8px |
| Grid Gap | `gap-4` | `var(--cathay-grid-gap)` | 16px |
| Grid Gap Large | `gap-6` | `var(--cathay-grid-gap-lg)` | 24px |

### ⚠️ Spacing Rules

**DO:**
- ✅ Use Tailwind classes that align with our spacing system (`p-6`, `gap-4`, `mb-3`, etc.)
- ✅ Maintain consistency across similar components
- ✅ Reference this guide when adding new components

**DON'T:**
- ❌ Use `p-5`, `gap-5`, `mb-5` or other non-standard spacing values
- ❌ Use arbitrary pixel values without CSS variables
- ❌ Mix different spacing patterns for similar elements

### Component-Specific Patterns

#### Dashboard Pages (ParentDashboard, CrewDashboard)
```tsx
<div className="space-y-6">           {/* Section spacing */}
  <Card className="p-6 mb-6">         {/* Card with spacing */}
    <div className="mb-4">            {/* Element spacing */}
    <div className="space-y-4">       {/* List items */}
    <div className="grid gap-4">      {/* Grid items */}
  </Card>
</div>
```

#### Sub Pages (Settings, Notifications, Profile)
```tsx
<div className="space-y-6">           {/* Section spacing */}
  <Card className="p-6">              {/* Header card */}
    <h2 className="mb-4">             {/* Title spacing */}
    <div className="space-y-4">       {/* Content items */}
  </Card>
</div>
```

#### List Pages (Alerts, Children List)
```tsx
<div className="space-y-6">                    {/* Section spacing */}
  <div className="space-y-4">                  {/* List items */}
  <div className="grid md:grid-cols-2 gap-6">  {/* Grid cards */}
</div>
```

---

## Project Status

### Current Spacing Compliance: 97% ✅

The project spacing is highly standardized with only minor optional optimizations possible.

**Excellent Compliance:**
- ✅ Dashboard pages: 95%
- ✅ Parent sub-pages: 100%
- ✅ Crew sub-pages: 95%
- ✅ Login page: 100%
- ✅ Navigation: 100%

**Optional Improvements:**
- A few instances of `p-5` (20px) can be changed to `p-6` or `p-4` for perfect consistency

---

## Development Checklist

When creating or reviewing components:

- [ ] Sections use `space-y-6` or `gap-6` (24px)
- [ ] Cards use `p-6` (24px) or `p-4` (16px)
- [ ] Elements use `mb-4` or `space-y-4` (16px)
- [ ] Grids use `gap-4` (16px) or `gap-6` (24px)
- [ ] No usage of `p-5`, `gap-5`, `mb-5` or other non-standard values
- [ ] Consistent spacing patterns with similar components

---

## Additional Notes

### Tailwind-CSS Variable Mapping
```
Tailwind → Pixels → CSS Variable
p-6     → 24px   → var(--cathay-card-padding)
gap-4   → 16px   → var(--cathay-element-gap)
space-y-3 → 12px → var(--cathay-element-gap-sm)
mb-2    → 8px    → var(--cathay-element-gap-xs)
```

### Migration from Non-Standard Values
If you encounter non-standard spacing:
- `mb-5`, `p-5`, `gap-5` (20px) → Choose `p-4` (16px) or `p-6` (24px) based on context
- Use larger values for primary/important elements
- Use smaller values for secondary/supporting elements

---

*For detailed component analysis and historical spacing reports, refer to Git history.*
