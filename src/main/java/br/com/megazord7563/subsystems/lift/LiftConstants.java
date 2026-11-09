// Copyright (c) 2026 Megazord 7563
// All rights reserved.
//
// This file is part of the XBbot FRC robot code.
// See the LICENSE file in the project root for license terms.

package br.com.megazord7563.subsystems.lift;

import br.com.megazord7563.generated.TunerConstants;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;

/** Constants for the Lift subsystem. */
public final class LiftConstants {
  // The CAN bus follows the existing mechanism convention. Confirm wiring before enabling Lift.
  public static final CANBus CAN_BUS = TunerConstants.kCANBus;

  public static final int LEADER_MOTOR_ID = 21;
  public static final int FOLLOWER_MOTOR_ID = 22;

  // The follower is mechanically mirrored relative to the leader.
  public static final boolean FOLLOWER_OPPOSES_LEADER = true;

  // This placeholder only allows the simulation model to be constructed; it is not a validated
  // ratio.
  public static final double MOTOR_GEAR_RATIO = 4.5; // considering the gear ratio

  public static final double PULLEY_RADIUS_METERS = 0.05732 / 2; // considering the pulley radius

  // Both Kraken X60 motors contribute torque to the same elevator mechanism.
  public static final DCMotor DC_MOTOR = DCMotor.getKrakenX60Foc(2);

  public static final double MAX_VOLTAGE = 12.0;

  public static final class FeedBackController {
    public static final double kP = 15;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final class SIM {
      public static final double kP = 0.09;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
    }
  }

  public static final class FeedForwardController {
    public static final double kS = 20.0;
    public static final double kV = 0.5;
    public static final double kA = 0.0;
    public static final double kG = 0.0;

    public static final class SIM {
      public static final double kS = 0.25;
      public static final double kV = 0.002;
      public static final double kA = 0.01;
      public static final double kG = 0.3;
    }
  }

  public static final double MOTOR_INERTIA = 0.6;

  public static final double LEVEL1_HARDLIMIT = 0.75;
  public static final double LEVEL2_HARDLIMIT = 0.75;

  public static final double LEVEL1_HARDLIMIT_TOP = 0.75;
  public static final double LEVEL2_HARDLIMIT_TOP = 1.50;

  public static final double POSITION_TOLERANCE_RAD = Math.toRadians(2.0);

  public static final double SOFTLIMIT_TOP = 800.0;
  public static final double SOFTLIMIT_BACK = 0.0;

  public static final double LIFT_CARRIAGE_MASS_KG = 11;

  public static final double SIM_MIN_HEIGHT_METERS = 0.0;
  public static final double SIM_MAX_HEIGHT_METERS = LEVEL2_HARDLIMIT_TOP;
  public static final double SIM_STARTING_HEIGHT_METERS = 0.0;

  public static final double STATOR_CURRENT_LIMIT_AMPS = 60.0;
  public static final double SUPPLY_CURRENT_LIMIT_AMPS = 40.0;

  public static final double MM_CRUISE_VELOCITY_RPS = 100.0;
  public static final double MM_ACCELERATION_RPS2 = 600.0;
  public static final double MM_JERK_RPS3 = 4000.0;

  private LiftConstants() {}
}
