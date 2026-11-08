// Copyright (c) 2026 Megazord 7563
// All rights reserved.
//
// This file is part of the XBbot FRC robot code.
// See the LICENSE file in the project root for license terms.

package br.com.megazord7563;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.AutoLogOutput;

import br.com.megazord7563.Constants.DriveMode;

public class RobotState {
  private static Pose2d robotPose = Pose2d.kZero;
  private static DriveMode driveMode = DriveMode.FAST; // Default drive mode, can be changed by commands
  private static ChassisSpeeds robotVelocity = new ChassisSpeeds();

  // ── Singleton ─────────────────────────────────────────────────────────────────

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  // ── Getters ─────────────────────────────────────────────────────────────────

  @AutoLogOutput(key = "RobotState/Rotation2d")
  public Rotation2d getRobotRotation() {
    return robotPose.getRotation();
  }

  @AutoLogOutput(key = "RobotState/Pose2d")
  public Pose2d getRobotPose() {
    return robotPose;
  }

  @AutoLogOutput(key = "RobotState/ChassisSpeeds")
  public ChassisSpeeds getRobotVelocity() {
    return robotVelocity;
  }

  @AutoLogOutput(key = "RobotState/FieldVelocity")
  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getRobotRotation());
  }

  @AutoLogOutput(key = "RobotState/DriveMode")
  public DriveMode getDriveMode() {
    return driveMode;
  }

  // ── Setters ─────────────────────────────────────────────────────────────────

  public void setRobotPose(Pose2d robotPose) {
    RobotState.robotPose = robotPose;
  }

  public void setDriveMode(DriveMode driveMode)
  {
    RobotState.driveMode = driveMode;
  }

  public void setRobotVelocity(ChassisSpeeds robotVelocity) {
    RobotState.robotVelocity = robotVelocity;
  }

   public void toggleDriveMode() {
    switch (driveMode) {
      case SLOW:
        driveMode = DriveMode.FAST;
        break;
      case FAST:
        driveMode = DriveMode.MAX;
        break;
      case MAX:
        driveMode = DriveMode.SLOW;
        break;
      default:
        driveMode = DriveMode.FAST; // Default to FAST if somehow in an unknown state
    }
  }
}
