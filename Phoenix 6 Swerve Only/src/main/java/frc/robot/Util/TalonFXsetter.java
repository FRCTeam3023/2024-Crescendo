// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;

import frc.robot.Util.PIDDisplay.PIDSetter;

/** Add your docs here. */
public class TalonFXsetter implements PIDSetter {

    List<TalonFXConfigurator> configurators = new ArrayList<TalonFXConfigurator>();
    Slot0Configs slot0Configs; 

    public TalonFXsetter(List<TalonFXConfigurator> configurators, TalonFXConfiguration configuration){
        this.configurators.addAll(configurators);
        slot0Configs = configuration.Slot0;
    }

    @Override
    public void setPID(double p, double i, double d, double s, double v) {
        slot0Configs.kP = p;
        slot0Configs.kI = i;
        slot0Configs.kD = d;
        slot0Configs.kS = s;
        slot0Configs.kV = v;
        

        configurators.forEach(configurator -> configurator.apply(slot0Configs));
    }

    @Override
    public double[] getPID() {
        double p = slot0Configs.kP;
        double i = slot0Configs.kI;
        double d = slot0Configs.kD;
        double s = slot0Configs.kS;
        double v = slot0Configs.kV;

        double[] all = {p, i, d, s, v};
        return all;
    }
}
