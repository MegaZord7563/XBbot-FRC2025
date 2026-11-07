// Copyright (c) 2026 Megazord 7563
// All rights reserved.
//
// This file is part of the XBbot FRC robot code.
// See the LICENSE file in the project root for license terms.

package br.com.megazord7563.subsystems.lift;

import static br.com.megazord7563.util.PhoenixUtil.*;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import br.com.megazord7563.Constants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/**
 * Hardware IO implementation for the Lift subsystem using two CTRE Talon FX controllers, with
 * built-in simulation support through a WPILib {@link ElevatorSim}.
 */
public class LiftIOTalonFX implements LiftIO {
  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;
  private final TalonFXSimState leaderSim;
  private final TalonFXSimState followerSim;

  private final ElevatorSim elevatorMotorSim =
      new ElevatorSim(
          LiftConstants.DC_MOTOR,
          LiftConstants.MOTOR_GEAR_RATIO,
          LiftConstants.LIFT_CARRIAGE_MASS_KG,
          LiftConstants.PULLEY_RADIUS_METERS,
          LiftConstants.SIM_MIN_HEIGHT_METERS,
          LiftConstants.SIM_MAX_HEIGHT_METERS,
          true, // simulate gravity
          LiftConstants.SIM_STARTING_HEIGHT_METERS);

  private final PositionVoltage positionRequest = new PositionVoltage(0.0).withSlot(0);
  private final MotionMagicVoltage motionRequest = new MotionMagicVoltage(0.0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final NeutralOut neutralRequest = new NeutralOut();
  private final CoastOut coastRequest = new CoastOut();
  private final Follower followerRequest;

  private final StatusSignal<Angle> leaderPosition;
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Current> leaderCurrent;

  private final StatusSignal<Angle> followerPosition;
  private final StatusSignal<AngularVelocity> followerVelocity;
  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> followerCurrent;

  private final Debouncer leaderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer followerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private boolean wasFollowerConnected = true;
  private NeutralModeValue currentNeutralMode = NeutralModeValue.Brake;

  public LiftIOTalonFX() {
    leaderMotor = new TalonFX(LiftConstants.LEADER_MOTOR_ID, LiftConstants.CAN_BUS);
    followerMotor = new TalonFX(LiftConstants.FOLLOWER_MOTOR_ID, LiftConstants.CAN_BUS);

    var leaderConfig = new TalonFXConfiguration();
    leaderConfig.Voltage.PeakForwardVoltage = LiftConstants.MAX_VOLTAGE;
    leaderConfig.Voltage.PeakReverseVoltage = -LiftConstants.MAX_VOLTAGE;
    leaderConfig.Feedback.SensorToMechanismRatio = LiftConstants.MOTOR_GEAR_RATIO;
    leaderConfig.Slot0.kP = LiftConstants.FeedBackController.kP;
    leaderConfig.Slot0.kI = LiftConstants.FeedBackController.kI;
    leaderConfig.Slot0.kD = LiftConstants.FeedBackController.kD;
    leaderConfig.Slot0.kS = LiftConstants.FeedForwardController.kS;
    leaderConfig.Slot0.kV = LiftConstants.FeedForwardController.kV;
    leaderConfig.Slot0.kA = LiftConstants.FeedForwardController.kA;
    leaderConfig.Slot0.kG = LiftConstants.FeedForwardController.kG;
    leaderConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = LiftConstants.SOFTLIMIT_TOP;
    leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = LiftConstants.SOFTLIMIT_BACK;
    leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    leaderConfig.withMotorOutput(
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake));
    leaderConfig.withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(LiftConstants.STATOR_CURRENT_LIMIT_AMPS)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(LiftConstants.SUPPLY_CURRENT_LIMIT_AMPS)
            .withSupplyCurrentLimitEnable(true));

    MotionMagicConfigs mm = new MotionMagicConfigs();
    mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(LiftConstants.MM_CRUISE_VELOCITY_RPS))
        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(LiftConstants.MM_ACCELERATION_RPS2))
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(LiftConstants.MM_JERK_RPS3));
    leaderConfig.MotionMagic = mm;

    tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(leaderConfig, 0.25));

    var followerConfig = new TalonFXConfiguration();
    followerConfig.withMotorOutput(
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake));
    followerConfig.withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(LiftConstants.STATOR_CURRENT_LIMIT_AMPS)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(LiftConstants.SUPPLY_CURRENT_LIMIT_AMPS)
            .withSupplyCurrentLimitEnable(true));

    tryUntilOk(5, () -> followerMotor.getConfigurator().apply(followerConfig, 0.25));

    leaderPosition = leaderMotor.getPosition();
    leaderVelocity = leaderMotor.getVelocity();
    leaderAppliedVolts = leaderMotor.getMotorVoltage();
    leaderCurrent = leaderMotor.getStatorCurrent();

    followerPosition = followerMotor.getPosition();
    followerVelocity = followerMotor.getVelocity();
    followerAppliedVolts = followerMotor.getMotorVoltage();
    followerCurrent = followerMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leaderPosition,
        leaderVelocity,
        leaderAppliedVolts,
        leaderCurrent,
        followerPosition,
        followerVelocity,
        followerAppliedVolts,
        followerCurrent);
    ParentDevice.optimizeBusUtilizationForAll(leaderMotor, followerMotor);

    followerRequest =
        new Follower(
            leaderMotor.getDeviceID(),
            LiftConstants.FOLLOWER_OPPOSES_LEADER
                ? MotorAlignmentValue.Opposed
                : MotorAlignmentValue.Aligned);
    tryUntilOk(5, () -> followerMotor.setControl(followerRequest));

    leaderSim = leaderMotor.getSimState();
    followerSim = followerMotor.getSimState();
    if (RobotBase.isSimulation()) {
      leaderSim.Orientation = ChassisReference.CounterClockwise_Positive;
      followerSim.Orientation = ChassisReference.CounterClockwise_Positive;
    }

    tryUntilOk(5, () -> leaderMotor.setPosition(0.0, 0.25));
    tryUntilOk(5, () -> followerMotor.setPosition(0.0, 0.25));
  }

  private void stepSimulation() {
    double batteryVolts = RobotController.getBatteryVoltage();
    leaderSim.setSupplyVoltage(batteryVolts);
    followerSim.setSupplyVoltage(batteryVolts);

    elevatorMotorSim.setInput(leaderSim.getMotorVoltage());
    elevatorMotorSim.update(Constants.periodicLoopPeriod);

    double radius = LiftConstants.PULLEY_RADIUS_METERS;
    double relativeHeightMeters =
        elevatorMotorSim.getPositionMeters() - LiftConstants.SIM_STARTING_HEIGHT_METERS;

    double mechanismRotations = relativeHeightMeters / (2.0 * Math.PI * radius);
    double mechanismRps = elevatorMotorSim.getVelocityMetersPerSecond() / (2.0 * Math.PI * radius);

    double rotorRotations = mechanismRotations * LiftConstants.MOTOR_GEAR_RATIO;
    double rotorRps = mechanismRps * LiftConstants.MOTOR_GEAR_RATIO;

    leaderSim.setRawRotorPosition(rotorRotations);
    leaderSim.setRotorVelocity(rotorRps);

    double followerSign = LiftConstants.FOLLOWER_OPPOSES_LEADER ? -1.0 : 1.0;
    followerSim.setRawRotorPosition(followerSign * rotorRotations);
    followerSim.setRotorVelocity(followerSign * rotorRps);
  }

  @Override
  public void updateInputs(LiftIOInputs inputs) {
    if (RobotBase.isSimulation()) {
      stepSimulation();
    }

    var leaderStatus =
        BaseStatusSignal.refreshAll(
            leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent);
    var followerStatus =
        BaseStatusSignal.refreshAll(
            followerPosition, followerVelocity, followerAppliedVolts, followerCurrent);

    inputs.leaderMotorConnected = leaderConnectedDebounce.calculate(leaderStatus.isOK());
    inputs.leaderPositionRad = Units.rotationsToRadians(leaderPosition.getValueAsDouble());
    inputs.leaderVelocityRadPerSec = Units.rotationsToRadians(leaderVelocity.getValueAsDouble());
    inputs.leaderAppliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.leaderCurrentAmps = leaderCurrent.getValueAsDouble();

    inputs.followerMotorConnected = followerConnectedDebounce.calculate(followerStatus.isOK());
    inputs.followerPositionRad = Units.rotationsToRadians(followerPosition.getValueAsDouble());
    inputs.followerVelocityRadPerSec =
        Units.rotationsToRadians(followerVelocity.getValueAsDouble());
    inputs.followerAppliedVolts = followerAppliedVolts.getValueAsDouble();
    inputs.followerCurrentAmps = followerCurrent.getValueAsDouble();

    if (inputs.followerMotorConnected && !wasFollowerConnected) {
      followerMotor.setControl(followerRequest);
    }
    wasFollowerConnected = inputs.followerMotorConnected;
  }

  @Override
  public void setPosition(double positionRad, boolean useMotionMagic) {
    if (Double.isNaN(positionRad)) {
      return;
    }
    double rotations = Units.radiansToRotations(positionRad);
    if (useMotionMagic) {
      leaderMotor.setControl(motionRequest.withPosition(rotations));
    } else {
      leaderMotor.setControl(positionRequest.withPosition(rotations));
    }
  }

  @Override
  public void setVoltage(double volts) {
    if (Double.isNaN(volts)) {
      volts = 0.0;
    }
    double clampedVolts =
        Math.max(-LiftConstants.MAX_VOLTAGE, Math.min(LiftConstants.MAX_VOLTAGE, volts));
    leaderMotor.setControl(voltageRequest.withOutput(clampedVolts));
  }

  @Override
  public void setCoast() {
    applyNeutralMode(NeutralModeValue.Coast);
    leaderMotor.setControl(coastRequest);
  }

  @Override
  public void setBrake() {
    applyNeutralMode(NeutralModeValue.Brake);
    leaderMotor.setControl(neutralRequest);
  }

  private void applyNeutralMode(NeutralModeValue mode) {
    if (mode == currentNeutralMode) {
      return;
    }
    leaderMotor.setNeutralMode(mode);
    followerMotor.setNeutralMode(mode);
    currentNeutralMode = mode;
  }
}
