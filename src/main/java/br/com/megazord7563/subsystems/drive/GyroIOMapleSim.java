// Copyright (c) 2026 Megazord 7563
// All rights reserved.
//
// This file is part of the XBbot FRC robot code.
// See the LICENSE file in the project root for license terms.

package br.com.megazord7563.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import br.com.megazord7563.util.PhoenixUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOMapleSim implements GyroIO {
    private final GyroSimulation gyroSimulation;

    public GyroIOMapleSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;

        // Update current gyro readings.
        inputs.yawPosition = gyroSimulation.getGyroReading();
        inputs.yawVelocityRadPerSec =
                gyroSimulation.getMeasuredAngularVelocity()
                        .in(RadiansPerSecond);

        // Update high-frequency odometry readings.
        inputs.odometryYawTimestamps =
                PhoenixUtil.getSimulationOdometryTimeStamps();

        inputs.odometryYawPositions =
                gyroSimulation.getCachedGyroReadings();
    }
}