// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

/** Add your docs here. */
public interface MotorSimModel {

    public void setMotorVoltage(double voltage);

    public void update(double period);

    public double getPosition();
    public double getVelocity();
}
