# UI Layout Update

## Date: May 30, 2025

### Changes Implemented

#### 1. Header View Updates
- **Log Message Box**: Now always visible, displays "No messages" when empty
- Uses secondary text color when no messages
- Orange background with info icon remains constant

#### 2. New Visualization Layout

```
┌────────────────────────────────────────────────────────────────┐
│                         Header View (100px)                     │
│  Settings | Connection Status | Mode Controls | Start/Stop      │
│  [Info] No messages / Last log message                         │
├────────────────────────────────────────────────────────────────┤
│ Control │              Visualization Area                       │
│  Panel  ├───────────────────────────────────────────────────────┤
│  (30%)  │                ServerStateView (80px)                 │
│         │ Mode | Motor | Launches | Threshold | Target | ...   │
│         ├───────────────────────────────────────────────────────┤
│         │ Y Offset │           Camera View                      │
│         │ Control  │      (Large viewing area)                 │
│         │ (100px)  │                                           │
│         ├───────────────────────────────────────────────────────┤
│         │           X Offset Control (100px)                    │
│         │    X: [──────slider──────] [Reset] [Send]            │
└─────────┴───────────────────────────────────────────────────────┘
```

#### 3. ServerStateView Redesign
Horizontal layout with equal spacing for all items:

| Mode | Motor | Launches | Threshold | Target | Stop Thr | Motor Off | Def Speed | Cutoff |
|------|-------|----------|-----------|---------|----------|-----------|-----------|---------|
| Manual | Running | 5 | N: 10<br>EPS: 0.005 | X: 0.000<br>Y: 0.000 | 0.03 | 0.20 | 0.26 | 20.0 Hz |

**Features:**
- Each item has equal width
- Grouped items (Threshold, Target) show values vertically
- Compact font sizes for efficient space usage
- Clear visual separators between items

#### 4. Layout Benefits
- **Larger Camera View**: Primary focus with maximum viewing area
- **Better Organization**: Clear separation of state, controls, and visualization
- **Efficient Space Usage**: Horizontal ServerStateView maximizes information density
- **Improved Hierarchy**: Top-to-bottom flow matches importance (State → View → Controls)

### Technical Implementation

#### Component Sizes:
- Header: Fixed 100px height
- ServerStateView: Fixed 80px height
- Y Offset Control: Fixed 100px width
- X Offset Control: Fixed 100px height
- Camera View: Flexible (takes all remaining space)

#### Font Sizes:
- ServerStateView titles: 9pt
- ServerStateView values: 10-11pt
- Maintains readability while maximizing information density 