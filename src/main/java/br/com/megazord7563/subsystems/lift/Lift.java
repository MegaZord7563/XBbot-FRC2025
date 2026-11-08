// Copyright (c) 2026 Megazord 7563
// All rights reserved.
//
// This file is part of the XBbot FRC robot code.
// See the LICENSE file in the project root for license terms.

package br.com.megazord7563.subsystems.lift;

import br.com.megazord7563.Constants;
import br.com.megazord7563.Constants.Mode;
import br.com.megazord7563.XBbotMechanism3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Subsystem responsible for the robot elevator. */
public class Lift extends SubsystemBase {
  private final LiftIO io;
  private final LiftIOInputsAutoLogged inputs = new LiftIOInputsAutoLogged();
  private final Alert leaderDisconnectedAlert;
  private final Alert followerDisconnectedAlert;
  private double goalPositionRad;
  private boolean hasActiveGoal;
  private XBbotMechanism3d mechanism3d;

  /** Creates a new Lift subsystem. */
  public Lift(LiftIO io) {
    this.io = io;
    io.setVoltage(0.0);
    leaderDisconnectedAlert =
        new Alert("Lift leader motor is disconnected or not configured.", AlertType.kError);
    followerDisconnectedAlert =
        new Alert("Lift follower motor is disconnected or not configured.", AlertType.kError);

    mechanism3d = XBbotMechanism3d.getInstance();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Lift", inputs);

    if (DriverStation.isDisabled()) {
      stop();
    }

    leaderDisconnectedAlert.set(!inputs.leaderMotorConnected && Constants.currentMode != Mode.SIM);
    followerDisconnectedAlert.set(
        !inputs.followerMotorConnected && Constants.currentMode != Mode.SIM);

    mechanism3d.setL1Height(getPositionMetters());
    mechanism3d.setL2Height(
        getPositionMetters() > LiftConstants.LEVEL1_HARDLIMIT
            ? getPositionMetters() - LiftConstants.LEVEL1_HARDLIMIT
            : 0.0);
    mechanism3d.setL3Height(
        getPositionMetters() > LiftConstants.LEVEL2_HARDLIMIT + LiftConstants.LEVEL1_HARDLIMIT
            ? getPositionMetters() - LiftConstants.LEVEL2_HARDLIMIT - LiftConstants.LEVEL1_HARDLIMIT
            : 0.0);
  }

  // ── Setters ─────────────────────────────────────────────────────────────────

  /** Moves the lift to the requested mechanism position in radians using the leader motor. */
  public void setPosition(double positionRad, boolean useMotionMagic) {
    goalPositionRad = positionRad;
    hasActiveGoal = true;
    io.setPosition(positionRad, useMotionMagic);
  }

  /** Runs the leader motor with the requested open-loop voltage. */
  public void setVoltage(double volts) {
    hasActiveGoal = false;
    io.setVoltage(volts);
  }

  /** Stops the lift and clears the active position goal. */
  public void stop() {
    setCoast();
  }

  /** Disengages the leader output and allows the coupled motors to coast. */
  public void setCoast() {
    hasActiveGoal = false;
    io.setCoast();
  }

  /** Engages brake mode through the leader motor. */
  public void setBrake() {
    hasActiveGoal = false;
    io.setBrake();
  }

  // ── Getters ─────────────────────────────────────────────────────────────────

  /** Returns whether the lift is within tolerance of its active position goal. */
  @AutoLogOutput(key = "Lift/AtGoal")
  public boolean atGoal() {
    return hasActiveGoal
        && Math.abs(goalPositionRad - getPositionRad()) <= LiftConstants.POSITION_TOLERANCE_RAD;
  }

  @AutoLogOutput(key = "Lift/Metters")
  public double getPositionMetters() {
    return inputs.leaderPositionRad * LiftConstants.PULLEY_RADIUS_METERS;
  }

  @AutoLogOutput(key = "Lift/LeaderPositionRad")
  public double getPositionRad() {
    return inputs.leaderPositionRad;
  }

  @AutoLogOutput(key = "Lift/LeaderVelocityRadPerSec")
  public double getVelocityRadPerSec() {
    return inputs.leaderVelocityRadPerSec;
  }

  @AutoLogOutput(key = "Lift/LeaderAppliedVolts")
  public double getAppliedVolts() {
    return inputs.leaderAppliedVolts;
  }

  @AutoLogOutput(key = "Lift/LeaderCurrentAmps")
  public double getCurrentAmps() {
    return inputs.leaderCurrentAmps;
  }

  public boolean isLeaderConnected() {
    return inputs.leaderMotorConnected;
  }

  public boolean isFollowerConnected() {
    return inputs.followerMotorConnected;
  }

  public boolean isHomed() {
    return getPositionMetters() < 0.05;
  }

  // ── Commands ────────────────────────────────────────────────────────────────

  /** Holds the lift at the requested position until interrupted. */
  public Command setPositionRadCommand(double positionRad, boolean useMotionMagic) {
    return run(() -> setPosition(positionRad, useMotionMagic));
  }

  /** Holds the lift at the requested position until interrupted. */
  public Command setPositionMettersCommand(double positionMetters, boolean useMotionMagic) {
    return run(
        () -> setPosition(positionMetters / LiftConstants.PULLEY_RADIUS_METERS, useMotionMagic));
  }

  /** Runs the lift at the requested voltage and stops when interrupted. */
  public Command runVoltageCommand(double volts) {
    return runEnd(() -> setVoltage(volts), this::stop);
  }

  /** Stops the lift. */
  public Command stopCommand() {
    return runOnce(this::stop);
  }
}
