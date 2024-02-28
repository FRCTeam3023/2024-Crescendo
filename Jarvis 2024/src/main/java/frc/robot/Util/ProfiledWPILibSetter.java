// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import java.util.List;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Util.PIDDisplay.PIDSetter;

/** Add your docs here. */
public class ProfiledWPILibSetter implements PIDSetter{
    private List<ProfiledPIDController> controllers;

    public ProfiledWPILibSetter(List<ProfiledPIDController> controllers){
        this.controllers = controllers;
    }

    @Override
    public void setPID(double p, double i, double d, double s, double v) {
        controllers.forEach(controller -> controller.setPID(p, i, d));
    }

    @Override
    public double[] getPID() {
        ProfiledPIDController controller = controllers.get(0);
        double p = controller.getP();
        double i = controller.getI();
        double d = controller.getD();
        double s = 0;
        double v = 0;

        double[] all = {p, i, d, s, v};
        return all;
    }

    public void addMotor(ProfiledPIDController controller) {
        controllers.add(controller);
    }
}
