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
import edu.wpi.first.math.geometry.Rotation2d;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;

/**
 * Manages the MapleSim physical simulation of the XBbot swerve drivetrain.
 *
 * <p>This class is exclusively intended for simulation mode. It creates the physical drivetrain
 * model, registers it in the simulated arena, and exposes the four module simulations to the
 * drivetrain IO layer.
 */
public final class MapleSimSwerveDrivetrain {
  private static final int MODULE_COUNT = 4;

  // Module order matches DriveSubsystem.getModuleTranslations(): FL, FR, BL, BR.
  private static final int FRONT_LEFT = 0;
  private static final int FRONT_RIGHT = 1;
  private static final int BACK_LEFT = 2;
  private static final int BACK_RIGHT = 3;

  private final SwerveDriveSimulation mapleSimDrive;
  private final SwerveModuleSimulation[] modules;

  /**
   * Creates the drivetrain simulation at a safe position inside the field.
   *
   * @throws IllegalStateException if instantiated outside simulation mode
   */
  public MapleSimSwerveDrivetrain() {
    this(new Pose2d(3.0, 3.0, new Rotation2d()));
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
          "MapleSimSwerveDrivetrain can only be instantiated in simulation mode.");
    }

    // Select the 2025 REEFSCAPE field. This must run before the first call to
    // SimulatedArena.getInstance(), otherwise MapleSim keeps its default arena.
    SimulatedArena.overrideInstance(new Arena2025Reefscape());

    // Create the physical drivetrain using the configuration defined in DriveSubsystem.
    mapleSimDrive = new SwerveDriveSimulation(DriveSubsystem.getMapleSimConfig(), initialPose);

    // MapleSim creates modules in the same order as the configured module translations.
    modules = mapleSimDrive.getModules();
    if (modules.length != MODULE_COUNT) {
      throw new IllegalStateException(
          "Expected " + MODULE_COUNT + " MapleSim swerve modules, got " + modules.length);
    }

    // Register the complete drivetrain once the model has been validated.
    SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimDrive);
  }

  /** Returns the MapleSim drivetrain simulation instance. */
  public SwerveDriveSimulation getSimulation() {
    return mapleSimDrive;
  }

  /** Returns the actual simulated robot pose. */
  public Pose2d getSimulatedPose() {
    return mapleSimDrive.getSimulatedDriveTrainPose();
  }

  /**
   * Teleports the simulated robot to the given pose.
   *
   * <p>Keeps the physical simulation in sync when odometry is reset (driver button, PathPlanner
   * auto start, etc.). Without this, the estimated pose and the MapleSim pose diverge.
   */
  public void resetPose(Pose2d pose) {
    mapleSimDrive.setSimulationWorldPose(pose);
  }

  /** Returns the front-left module simulation. */
  public SwerveModuleSimulation getFrontLeftModule() {
    return modules[FRONT_LEFT];
  }

  /** Returns the front-right module simulation. */
  public SwerveModuleSimulation getFrontRightModule() {
    return modules[FRONT_RIGHT];
  }

  /** Returns the back-left module simulation. */
  public SwerveModuleSimulation getBackLeftModule() {
    return modules[BACK_LEFT];
  }

  /** Returns the back-right module simulation. */
  public SwerveModuleSimulation getBackRightModule() {
    return modules[BACK_RIGHT];
  }
}
