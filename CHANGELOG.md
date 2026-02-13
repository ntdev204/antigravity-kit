# Changelog

All notable changes to the Antigravity Kit will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/2.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.3.0] - 2026-02-13

### Added

- **CLI Tool**: Added `ag-kit` command-line interface
    - Global installation: `npm install -g @ntdev204/ag-kit`
    - npx support: `npx @ntdev204/ag-kit init`
    - Commands: `init`, `update`, `--help`, `--version`
    - Automatic `.agent` folder installation to current project
    - Self-update capability with `ag-kit update`
- **Agent ntdev204**:
    - Updated `orchestrate.md` fix output turkish
- **New Skills**:
    - `find-skills` - Helps users discover and install agent skills
    - `mcp-builder` - MCP server building principles and patterns
    - `vercel-react-best-practices` - Performance optimization guidelines from Vercel Engineering
    - `webapp-testing` - E2E and deep audit testing strategies

### Changed

- **BREAKING: AI & Robotics Skills Refactor** - Progressive disclosure pattern applied to 5 major skills
    - **ai-engineering**: Refactored 255→170 lines + 4 reference files (frameworks, architectures, RL/DRL, MLOps)
    - **robotics-automation**: Refactored 367→185 lines + 3 reference files (PLC/SCADA, components, integration)
    - **iot-solutions**: Refactored 337→165 lines + 3 reference files (hardware, protocols, diagrams)
    - **ros2-humble**: Refactored 553→190 lines + 4 reference files (node development, launch/params, packages, structure)
    - **control-systems**: Refactored 493→160 lines + 4 reference files (PID, fuzzy logic, modern control, implementation)
- **Workflow: /rai/**: Created comprehensive Robotics & AI orchestration workflow
    - Added domain identification and skill routing
    - Defined implementation phases for each domain
    - Common integration patterns (AI+Robot, IoT+Control, ROS2+AI+Control)
    - Multi-skill troubleshooting guide

### Performance

- Reduced initial skill load time by ~60% through progressive disclosure
- Improved context efficiency with on-demand reference loading
- Core decision trees remain in SKILL.md, detailed specs moved to `references/`

[2.0.0]: https://github.com/ntdev204/antigravity-kit/releases/tag/v2.3.0
