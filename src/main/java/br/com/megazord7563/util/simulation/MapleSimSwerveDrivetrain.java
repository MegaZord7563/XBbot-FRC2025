
// Copyright (c) 2026 Megazord 7563
// All rights reserved.
//
// This file is part of the XBbot FRC robot code.
// See the LICENSE file in the project root for license terms.

package br.com.megazord7563.util.simulation;

import br.com.megazord7563.Constants;
import br.com.megazord7563.Constants.Mode;
import br.com.megazord7563.subsystems.drive.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

/**
 * Manages the MapleSim physical simulation of the XBbot swerve drivetrain.
 *
 * <p>This class is exclusively intended for simulation mode. It creates the
 * physical drivetrain model and registers it in the simulated arena.
 *
 * <p>Communication between the simulated drivetrain and Phoenix 6 devices
 * will be handled by the simulation IO integration.
 */
public final class MapleSimSwerveDrivetrain {

    private final SwerveDriveSimulation mapleSimDrive;

    /**
     * Creates the drivetrain simulation at the origin of the field.
     *
     * @throws IllegalStateException if instantiated outside simulation mode
     */
    public MapleSimSwerveDrivetrain() {
        this(new Pose2d());
    }

    /**
     * Creates the drivetrain simulation at the specified initial pose.
     *
     * @param initialPose initial robot pose in the simulation field
     * @throws IllegalStateException if instantiated outside simulation mode
     */
    public MapleSimSwerveDrivetrain(Pose2d initialPose) {

        if (Constants.currentMode != Mode.SIM) {
            throw new IllegalStateException(
                "MapleSimSwerveDrivetrain can only be instantiated in simulation mode."
            );
        }

        // Create the physical drivetrain using the configuration defined
        // in DriveSubsystem.
        mapleSimDrive = new SwerveDriveSimulation(
            DriveSubsystem.getMapleSimConfig(),
            initialPose
        );

        // Register the drivetrain in the MapleSim arena.
        SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimDrive);
    }

    /**
     * Returns the MapleSim drivetrain simulation instance.
     */
    public SwerveDriveSimulation getSimulation() {
        return mapleSimDrive;
    }

    /**
     * Returns the actual simulated robot pose.
     */
    public Pose2d getSimulatedPose() {
        return mapleSimDrive.getSimulatedDriveTrainPose();
    }
}
