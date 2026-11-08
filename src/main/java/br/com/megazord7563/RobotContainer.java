// Copyright (c) 2026 Megazord 7563
// All rights reserved.
//
// This file is part of the XBbot FRC robot code.
// See the LICENSE file in the project root for license terms.

package br.com.megazord7563;

import br.com.megazord7563.Constants.DriveMode;
import br.com.megazord7563.commands.drive.DriveCommands;
import br.com.megazord7563.generated.TunerConstants;
import br.com.megazord7563.subsystems.drive.DriveSubsystem;
import br.com.megazord7563.subsystems.drive.GyroIO;
import br.com.megazord7563.subsystems.drive.GyroIOMapleSim;
import br.com.megazord7563.subsystems.drive.GyroIOPigeon2;
import br.com.megazord7563.subsystems.drive.ModuleIO;
import br.com.megazord7563.subsystems.drive.ModuleIOSim;
import br.com.megazord7563.subsystems.drive.ModuleIOTalonFX;
import br.com.megazord7563.subsystems.lift.Lift;
import br.com.megazord7563.subsystems.lift.LiftIO;
import br.com.megazord7563.subsystems.lift.LiftIOTalonFX;
import br.com.megazord7563.util.simulation.MapleSimSwerveDrivetrain;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final DriveSubsystem driveSubsystem;
  private MapleSimSwerveDrivetrain mapleSimDrivetrain;
  private final Lift lift;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        driveSubsystem =
            new DriveSubsystem(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        lift = new Lift(new LiftIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        mapleSimDrivetrain = new MapleSimSwerveDrivetrain();

        driveSubsystem =
            new DriveSubsystem(
                new GyroIOMapleSim(mapleSimDrivetrain.getSimulation().getGyroSimulation()),
                new ModuleIOSim(mapleSimDrivetrain.getFrontLeftModule()),
                new ModuleIOSim(mapleSimDrivetrain.getFrontRightModule()),
                new ModuleIOSim(mapleSimDrivetrain.getBackLeftModule()),
                new ModuleIOSim(mapleSimDrivetrain.getBackRightModule()));
        lift = new Lift(new LiftIOTalonFX());

        // Keep odometry and the physical simulation in the same starting pose,
        // and make every odometry reset also move the simulated robot.
        driveSubsystem.setSimulationPoseResetter(mapleSimDrivetrain::resetPose);
        driveSubsystem.setSimulationPoseSupplier(mapleSimDrivetrain::getSimulatedPose);
        driveSubsystem.setPose(mapleSimDrivetrain.getSimulatedPose());
        break;

      default:
        // Replayed robot, disable IO implementations
        driveSubsystem =
            new DriveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        lift = new Lift(new LiftIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up Drive SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        DriveCommands.wheelRadiusCharacterization(driveSubsystem));
    autoChooser.addOption(
        "Drive Simple FF Characterization",
        DriveCommands.feedforwardCharacterization(driveSubsystem));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)",
        driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)",
        driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Set up Outtake SysId routines
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    driveSubsystem.setDefaultCommand(
        DriveCommands.joystickDrive(
            driveSubsystem,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            () -> controller.leftStick().getAsBoolean()));

    // Lock to 0° when A button is held
    /**
     * controller .a() .whileTrue( DriveCommands.joystickDriveAtAngle( driveSubsystem, () ->
     * -controller.getLeftY(), () -> -controller.getLeftX(), () -> Rotation2d.kZero));
     */

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(driveSubsystem::stopWithX,
    // driveSubsystem));

    // POSITION PID NORMAL
    controller
        .y()
        .and(controller.leftBumper())
        .whileTrue(lift.setPositionMettersCommand(1.54, false));
    controller
        .b()
        .and(controller.leftBumper())
        .whileTrue(lift.setPositionMettersCommand(0.98, false));
    controller
        .x()
        .and(controller.leftBumper())
        .whileTrue(lift.setPositionMettersCommand(0.40, false));
    controller
        .a()
        .and(controller.leftBumper())
        .whileTrue(lift.setPositionMettersCommand(0.0, false));

    /** USE MOTION MAGIC */
    controller
        .y()
        .and(controller.rightBumper())
        .whileTrue(lift.setPositionMettersCommand(1.54, true));
    controller
        .b()
        .and(controller.rightBumper())
        .whileTrue(lift.setPositionMettersCommand(0.98, true));
    controller
        .x()
        .and(controller.rightBumper())
        .whileTrue(lift.setPositionMettersCommand(0.40, true));
    controller
        .a()
        .and(controller.rightBumper())
        .whileTrue(lift.setPositionMettersCommand(0.0, true));

    // Reset heading to zero when the Start button is pressed
    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        driveSubsystem.setPose(
                            new Pose2d(
                                driveSubsystem.getPose().getTranslation(), Rotation2d.kZero)),
                    driveSubsystem)
                .ignoringDisable(true));

    Trigger liftHomed = new Trigger(() -> lift.isHomed());

    liftHomed
        .negate()
        .onTrue(new InstantCommand(() -> RobotState.getInstance().setDriveMode(DriveMode.LIFT)));

    controller
        .rightBumper()
        .and(liftHomed)
        .onTrue(new InstantCommand(() -> RobotState.getInstance().setDriveMode(DriveMode.MAX)));

    controller
        .leftBumper()
        .and(liftHomed)
        .onTrue(new InstantCommand(() -> RobotState.getInstance().setDriveMode(DriveMode.SLOW)));

    controller
        .leftBumper()
        .negate()
        .and(controller.rightBumper().negate())
        .and(liftHomed)
        .onTrue(new InstantCommand(() -> RobotState.getInstance().setDriveMode(DriveMode.FAST)));
  }

  /** Returns the real simulated (MapleSim) robot pose, or null outside simulation. */
  public Pose2d getMapleSimPose() {
    return mapleSimDrivetrain != null ? mapleSimDrivetrain.getSimulatedPose() : null;
  }

  /** Use this to pass the drive subsystem to the main {@link Robot} class. */
  public DriveSubsystem getDrive() {
    return driveSubsystem;
  }

  /** Use this to pass the lift subsystem to the main {@link Robot} class. */
  public Lift getLift() {
    return lift;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * Use this to pass the test routine command to the main {@link Robot} class.
   *
   * @return the command to run in test mode
   */
  public Command getTestRoutineCommand() {
    return Commands.none();
  }
}
