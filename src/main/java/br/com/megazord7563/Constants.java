// Copyright (c) 2026 Megazord 7563
// All rights reserved.
//
// This file is part of the XBbot FRC robot code.
// See the LICENSE file in the project root for license terms.

// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package br.com.megazord7563;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final double periodicLoopPeriod = 0.01; // 20 ms

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum DriveMode {
    SLOW(0.3), // Very low speed for precise maneuvers (e.g., scoring)
    FAST(0.6), // Normal driving speed
    MAX(0.9), // Highest possible speed (e.g., traveling across the field)
    LIFT(0.2); // Extremely slow speed for shooting better precision on move

    private double speedValue;

    private DriveMode(double speedValue) {
      this.speedValue = speedValue;
    }

    public double getSpeedValue() {
      return speedValue;
    }
  }
}
