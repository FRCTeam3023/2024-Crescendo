// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import frc.robot.Constants;

/** Add your docs here. */
public class TalonFXSimModel implements MotorSimModel{

    private final double supplyVoltage = Constants.SIMULATION_SUPPLY_VOLTAGE;
    private final double maxVelocity;
    private final double maxAcceleration;

    private double currentVoltage = 0;
    private double currentPosition = 0;
    private double currentVelocity = 0;
    private double currentAcceleration = 0;

    public TalonFXSimModel(double maxVelocity, double maxAcceleration){
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }
    @Override
    public void setMotorVoltage(double voltage) {
        currentVoltage = voltage;
    }

    @Override
    public void update(double period) {
        double voltageRatio = currentVoltage/supplyVoltage;

        currentAcceleration = voltageRatio * maxAcceleration;

        if(Math.abs(voltageRatio * maxAcceleration * period + currentVelocity) < maxVelocity){
            currentVelocity = currentAcceleration * period + currentVelocity;
        }else{
            currentVelocity = maxVelocity;
        }

        currentPosition = currentVelocity * period + currentPosition;
    }
    @Override
    public double getPosition() {
        return currentPosition;
    }
    @Override
    public double getVelocity() {
        return currentVelocity;
    }

}
