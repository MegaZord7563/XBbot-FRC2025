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

package br.com.megazord7563.util;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;

public class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  /**
   * Returns timestamps for the cached MapleSim odometry samples.
   *
   * <p>The timestamp span is derived from the arena's configured sub-tick count and duration,
   * keeping it aligned with MapleSim's default simulation timing.
   */
  public static double[] getSimulationOdometryTimeStamps() {
    int sampleCount = SimulatedArena.getSimulationSubTicksIn1Period();
    double simulationDtSeconds = SimulatedArena.getSimulationDt().in(Seconds);
    double simulationPeriodSeconds = sampleCount * simulationDtSeconds;
    double[] odometryTimeStamps = new double[sampleCount];
    double currentTimestamp = Timer.getFPGATimestamp();

    for (int i = 0; i < sampleCount; i++) {
      odometryTimeStamps[i] = currentTimestamp - simulationPeriodSeconds + i * simulationDtSeconds;
    }

    return odometryTimeStamps;
  }
}
