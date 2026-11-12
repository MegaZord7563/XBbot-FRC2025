// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.com.megazord7563.subsystems.outtake;

public interface OuttakeIO {
    public boolean pivotMotorConnected = false;
    public double pivotPositionRad = 0.0;
    public double pivotVelocityRadPerSec = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotCurrentAmps = 0.0;

    public boolean outtakeMotorConnected = false;
    public double outtakePositionRad = 0.0;
    public double outtakeVelocityRadPerSec = 0.0;
    public double outtakeAppliedVolts = 0.0;
    public double outtakeCurrentAmps = 0.0;
}
