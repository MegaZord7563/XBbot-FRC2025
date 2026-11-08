<!-- Copyright (c) 2026 Megazord 7563. All rights reserved. -->
<!-- This file is part of the XBbot FRC robot code. See LICENSE for license terms. -->

# XBbot — FIRST Robotics Competition

<p align="center">
  <img src="assets/DSCN2621.JPG" alt="XBbot robot" width="720">
</p>

<p align="center">
  <strong>Robot code, configuration, and engineering assets for the XBbot FRC project.</strong>
</p>

<p align="center">
  <a href="assets/DSCN2629.MP4">Watch the robot video</a>
  ·
  <a href="ascope/Robot_XBbot/config.json">AdvantageScope robot configuration</a>
</p>

---

## About the project

XBbot is a FIRST Robotics Competition robot project built with Java and WPILib. The codebase uses the command-based framework and includes a swerve-drive implementation with real-robot, simulation, and log-replay I/O paths.

The repository also contains robot media, AdvantageScope assets, PathPlanner files, and development automation.

> **Hardware note:** Treat the source code and configuration as the current implementation reference. Verify device models, CAN IDs, wiring, and calibration values against the robot before deploying. This README intentionally does not infer hardware specifications that are not documented in the project.

## Robot

The repository includes a photo of XBbot and a recorded video:

- **Robot photo:** [assets/DSCN2621.JPG](assets/DSCN2621.JPG)
- **Robot video:** [assets/DSCN2629.MP4](assets/DSCN2629.MP4)

Media files are stored with Git LFS. Make sure Git LFS is installed and initialized before cloning or pulling if you want the full media files rather than LFS pointer files.

## Software stack

| Component | Project configuration |
|---|---|
| Language | Java |
| Java compatibility | 17 |
| Build system | Gradle Wrapper |
| WPILib build plugin | GradleRIO 2026.2.1 |
| Robot framework | WPILib Command-Based |
| Drive architecture | Swerve drive |
| Logging and replay | AdvantageKit |
| Autonomous paths | PathPlanner |
| Code formatting | Spotless / Google Java Format |

The versions above reflect the checked-in Gradle configuration. Use a compatible WPILib development environment when opening or building the project.

## Repository layout

```text
.
├── .github/
│   ├── scripts/             # Vendordep update automation
│   └── workflows/           # CI build and dependency update workflows
├── ascope/
│   └── Robot_XBbot/          # AdvantageScope robot assets and configuration
├── assets/                   # Robot photo and video (Git LFS)
├── gradle/wrapper/           # Gradle Wrapper runtime
├── src/main/
│   ├── deploy/pathplanner/   # Autonomous paths and field navigation data
│   └── java/                 # Robot, commands, subsystems, and utilities
├── build.gradle
├── gradlew
└── gradlew.bat
```

## Getting started

### Requirements

- Git
- Git LFS (for the full-resolution media assets)
- WPILib development environment compatible with the project's GradleRIO version
- JDK 17

### Clone the repository

```bash
git lfs install
git clone https://github.com/Kaique-Sique/XBbot-FRC2025.git
cd XBbot-FRC2025
```

If the repository has already been cloned without LFS, run:

```bash
git lfs pull
```

### Build the project

The Gradle Wrapper is included in the repository, so a separate Gradle installation is not required.

**Linux / macOS**

```bash
chmod +x gradlew
./gradlew build
```

**Windows (PowerShell)**

```powershell
.\gradlew.bat build
```

## Common commands

Run commands from the repository root.

| Task | Linux / macOS | Windows |
|---|---|---|
| Build | `./gradlew build` | `.\gradlew.bat build` |
| Run tests | `./gradlew test` | `.\gradlew.bat test` |
| Deploy to the RoboRIO | `./gradlew deploy` | `.\gradlew.bat deploy` |
| List Gradle tasks | `./gradlew tasks` | `.\gradlew.bat tasks` |
| Assemble artifacts | `./gradlew assemble` | `.\gradlew.bat assemble` |
| Watch AdvantageKit replay | `./gradlew replayWatch` | `.\gradlew.bat replayWatch` |

Deployment requires a correctly configured team number and a connected/available RoboRIO. Confirm the target hardware and deployment settings before running the deploy task.

## Development tools and assets

- **AdvantageScope:** The robot configuration is located at [ascope/Robot_XBbot/config.json](ascope/Robot_XBbot/config.json). Open the configuration and associated model assets in AdvantageScope.
- **PathPlanner:** Autonomous path and navigation files are under [src/main/deploy/pathplanner](src/main/deploy/pathplanner).
- **Vendordeps:** Third-party robot libraries are managed through the project's `vendordeps/` directory.
- **GitHub Actions:** Build and vendordep update workflows are located in [.github/workflows](.github/workflows).

## Contributing

1. Create a feature branch from `main`.
2. Keep changes focused and document hardware-dependent assumptions.
3. Run the relevant Gradle build or test tasks.
4. Open a pull request with a concise description and validation notes.

When changing robot hardware, update the related code and configuration together, and verify all wiring, CAN IDs, offsets, and limits on the physical robot.

## License

See [LICENSE](LICENSE) and the third-party license notices in the repository for applicable terms.

## Simulação (MapleSim)

- Rode com *Simulate Robot Code* e marque **Sim GUI** (ou conecte um Driver Station / AdvantageScope). O `build.gradle` deixa a GUI desligada por padrão para permitir *replay* do AdvantageKit; sem ela não há joystick nem botão de *Enable*.
- O robô só se move com o Driver Station em **Enabled** (teleop).
- `Odometry/Robot` é a pose estimada; `FieldSimulation/RobotPosition` é a pose real da física do MapleSim. Elas devem coincidir.
- Resetar a odometria (botão Start ou início de auto do PathPlanner) também teleporta o robô simulado.
