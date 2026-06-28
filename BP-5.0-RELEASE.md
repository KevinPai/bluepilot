# BluePilot 5.0

> **Base:** SunnyPilot 2025.003.0.0 | **AGNOS:** 13.1 | **Released:** November 25, 2025

BluePilot 5.0 is a major release featuring a brand new web interface, redesigned on-device UI, enhanced onroad visuals, and improved Ford lateral controls.

**[Watch the Release Overview on YouTube]**
https://www.youtube.com/watch?v=9ggGzCI-zx0

---

## Quick Install

**Fresh Install:**

```text
installer.comma.ai/BluePilotDev/bp-5.0
```

**Upgrading:** Settings → Software → CHECK → Select `bp-5.0` → Install

---

## What's New in 5.0

### BluePilot Portal

A new web-based control center accessible at `http://[device-ip]:8088`

- **Dashboard** — Real-time system status, drive stats, disk usage
- **Routes** — Browse drives, view GPS paths, play video, export footage
- **Settings** — Configure all parameters with live WebSocket updates
- **Diagnostics** — Live TMUX streaming and parameter debugging
- **PWA Support** — Install on your phone for app-like access

### Redesigned On-Device Settings

Settings have been reorganized into focused panels:

- **Core:** Device, Display, Network, Vehicle
- **Driving:** Toggles, Cruise, Steering
- **Visual:** Visuals, Developer
- **New:** Models Panel (driving model switching) and Software Panel (updates & branch management)

### Enhanced Onroad Visuals

- Improved lane lines, road edges, and path rendering with glow effects
- Smoother curve tracking with reduced path swaying
- Better color consistency and refined rendering

### Ford-Specific Improvements

- **Human Turn Detection** — Latch time increased to 3.0s for smoother handling
- **Panda Safety** — Curvature limits refined for better message handling
- **Bug Fixes** — Fixed blocked messages during speed transitions and turns

### Additional Changes

#### Vehicle Features

- Brake status indicator via CAN signals
- Radar overlay with correct speed units
- Improved stop sign detection display
- More accurate hybrid battery gauge
- Option to bypass BluePilot lateral controls

#### Developer Tools

- Customizable onroad debug panel
- UI crash detection system
- Unified logging throughout BluePilot code

#### Performance

- Refactored renderer for better performance
- Thread safety improvements
- Fixed web server startup crashes

---

## Recommended Models by Vehicle

### CAN Vehicles (Nevada, WD40 recommended)

- Ford Bronco Sport (2021-24)
- Ford Edge (2022)
- Ford Escape (2020-22)
- Ford Explorer (2020-24)
- Ford Focus (2018)
- Ford Maverick (2022-24)
- Lincoln Aviator (2020-24)

### CANFD Vehicles (Nevada, WD40 recommended)

- Ford Escape (2023-24)
- Ford Kuga (2020-24)
- Ford Mustang Mach-E (2021-24)
- Ford Ranger (2024)

### CANFD Vehicles (Nevada, WD40, FoF recommended)

- Ford Expedition (2022-24)
- Ford F-150 (2021-23)
- Ford F-150 Lightning (2022-23)

**Model Notes:**

- **Nevada / WD40** — Recommended for all Ford platforms
- **FoF** — Optimized specifically for F-150, F-150 Lightning, and Expedition

---

## Included SunnyPilot Features

All features from SunnyPilot 2025.003.0.0 are included:

- **MADS** — Modular Assistive Driving System
- **NNLC** — Neural Network Lateral Control
- **DEC** — Dynamic Experimental Control
- **SLA** — Speed Limit Assist
- **ICBM** — Intelligent Cruise Button Management *(not yet available for Ford)*
- **Model Manager** — 86+ driving models available
- **sunnylink** — Cloud integration

---

## Documentation

- [CHANGELOG.md](https://github.com/BluePilotDev/bluepilot/blob/bp-5.0/CHANGELOG.md) — Full changelog
- [README.md](https://github.com/BluePilotDev/bluepilot/blob/bp-5.0/README.md) — Feature documentation
- [CHANGELOG_SP.md](https://github.com/BluePilotDev/bluepilot/blob/bp-5.0/CHANGELOG_SP.md) — SunnyPilot upstream changes
- [README_SP.md](https://github.com/BluePilotDev/bluepilot/blob/bp-5.0/README_SP.md) — SunnyPilot features

---

## Known Issues

- Routes Panel video playback is in beta and may have performance issues on some devices

---

## Support

- **Community** — [SunnyPilot Forum](https://community.sunnypilot.ai) (preferred) or **#ford** on Discord
- **Bug Reports** — [GitHub Issues](https://github.com/BluePilotDev/bluepilot/issues)

### Reporting Issues

**A shared Route ID from comma connect is required for all bug reports.** Without route data, we cannot diagnose the issue.

To report a bug on GitHub, include:

1. **Route ID** — Shared via comma connect (required)
2. **Vehicle** — Make, model, and year
3. **Description** — What happened and what you expected
4. **Steps to reproduce** — If applicable

**How to share your route:**

1. Open [comma connect](https://connect.comma.ai)
2. Find the drive where the issue occurred
3. Click the share button and make the route public
4. Include the route link in your issue

---

## Thank You

Thanks to everyone who contributed through testing, feedback, and development. Special thanks to the SunnyPilot team and the Ford community.

**Safe driving!**
*— The BluePilot Team*
