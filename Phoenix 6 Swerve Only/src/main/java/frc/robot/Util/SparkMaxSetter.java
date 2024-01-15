// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import java.util.List;

import com.revrobotics.SparkMaxPIDController;

import frc.robot.Util.PIDDisplay.PIDSetter;

/** Add your docs here. */
public class SparkMaxSetter implements PIDSetter {

    private List<SparkMaxPIDController> motors;
    
    public SparkMaxSetter(List<SparkMaxPIDController> motors){
        this.motors = motors;
    }

    @Override
    public void setPID(double p, double i, double d, double s, double v) {
        motors.forEach(motor -> {
            motor.setP(p);
            motor.setI(i);
            motor.setD(d);
            motor.setFF(v);
        });
    }



    @Override
    public double[] getPID() {
        SparkMaxPIDController motor = motors.get(0);
        double p = motor.getP();
        double i = motor.getI();
        double d = motor.getD();
        double s = 0;
        double v = motor.getFF();

        double[] all = {p, i, d, s, v};
        return all;
    }
}
