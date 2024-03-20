// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

/** Add your docs here. */
public class AutoAimCalculator {
    public static Rotation2d theta;
    public static double translationTime;
    public static Pose3d translatedPose;

    public static void computeAngle(Pose2d robotPose, ChassisSpeeds velocity) {
        Pose3d target = Constants.blueSpeakerPose;
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            target = Constants.redSpeakerPose;
        }

        Pose3d relativeTarget = new Pose3d(target.getX() - robotPose.getX(), target.getY() - robotPose.getY(), target.getZ(), new Rotation3d());

        newtonApproximation(relativeTarget, velocity);
        // if (relativeTarget.getX() * relativeTarget.getX() + relativeTarget.getY() * relativeTarget.getY()
        //         > Constants.ArmConstants.MAX_AIM_DISTANCE * Constants.ArmConstants.MAX_AIM_DISTANCE)
        //     theta = Rotation2d.fromDegrees(45);
    }

    private static void newtonApproximation(Pose3d relativeTarget, ChassisSpeeds relativeVelocity) {
        // All measures are in radians for this function
        double last = Math.PI / 4.0; // Initial guess
        double evaluation = 0;
        double derivative = 0;

        double t_x = relativeTarget.getX();
        double t_y = relativeTarget.getY();
        double t_z = relativeTarget.getZ();
        double l = Constants.ArmConstants.PIVOT_LENGTH;
        double h = Constants.ArmConstants.PIVOT_HEIGHT;
        double beta = Constants.ArmConstants.LAUNCHER_ANGLE_WITH_PIVOT.getRadians();
        double v_l = Constants.ArmConstants.NOTE_LAUNCH_SPEED;
        double v_x = relativeVelocity.vxMetersPerSecond;
        double v_y = relativeVelocity.vyMetersPerSecond;
        
        for (int i = 0; i < Constants.ArmConstants.PIVOT_APPROXIMATION_PRECISION; i++) {
            double sin = Math.sin(last);
            double cos = Math.cos(last);
            double cosB = Math.cos(last - beta);
            double sinB = Math.sin(last - beta);

            if (Constants.ArmConstants.VELOCITY_BASED_AIMING) {
                double A = t_x * v_x + t_y * v_y - v_l * l * cosB * cos;
                double A_d = v_l * l * (sinB * cos + cosB * sin);
                double B = v_l * v_l * cosB * cosB - v_x * v_x - v_y * v_y;
                double B_d = -2 * v_l * v_l * cosB * sinB;
                double J = l * l * cos * cos - t_x * t_x - t_y * t_y;
                double J_d = 2 * l * l * cos * sin;
                double E = A * A - B * J;
                double sqrtE = Math.sqrt(E);
                double E_d = 2 * A * A_d - B_d * J + B * J_d;
                double G = sqrtE - A;
                double G_d = E_d / (2 * sqrtE) - A_d;
                double T = G / B;
                double T_d = (G_d * B - G * B_d) / (B * B);
                double C = t_z - l * sin - h + 4.9 * T * T;
                double C_d = -l * cos + 9.8 * T * T_d;
                double H = t_x - v_x * T;
                double H_d = -v_x * T_d;
                double I = t_y - v_y * T;
                double I_d = -v_y * T_d;
                double D = H * H + I * I;
                double sqrtD = Math.sqrt(D);
                double D_d = 2 * (H * H_d + I * I_d);
                double F = sqrtD + l * cos;
                double F_d = D_d / (2 * sqrtD) - l * sin;

                
                evaluation = beta - last - Math.atan2(C, F);
                derivative = (C * F_d - C_d * F) / (C * C + F * F) - 1;

                // System.out.println("A:" + A + " A_d:" + A_d + " B:" + B + " B_d:" + B_d + " E:" + E + " E_d:" + E_d 
                //     + " T:" + T + " T_d:" + T_d + " C:" + C + " C_d:" + C_d + " D:" + D + " D_d:" + D_d + " F:" + F 
                //     + " F_d:" + F_d + " G:" + G + " G_d:" + G_d + " H:" + H + " H_d:" + H_d + " I:" + I + " I_d:" + I_d 
                //     + " J:" + J + " J_d:" + J_d + " angle:" + evaluation + " derivative:" + derivative);

                translationTime = T;
            } else {
                double groundDistance = Math.sqrt(t_x * t_x + t_y * t_y) + l * cos;
                double totalHeight = t_z - l * sin - h + Math.max(0, (groundDistance - 1) / 6);
                evaluation = evaluateAngleStationary(last, totalHeight, groundDistance);
                derivative = evaluateAngleDerivativeStationary(last, l * sin, l * cos, groundDistance, totalHeight);
            }
            last = last - evaluation / derivative;
        }
        theta = Rotation2d.fromRadians(last);
        translatedPose = new Pose3d(t_x - v_x * translationTime, t_y - v_y * translationTime, t_z + 4.9 * translationTime * translationTime, new Rotation3d());
    }

    private static double evaluateAngleStationary(double theta, double totalHeight, double groundDistance) {
        return Constants.ArmConstants.LAUNCHER_ANGLE_WITH_PIVOT.getRadians() - theta
                - Math.atan2(totalHeight, groundDistance);
    }

    private static double evaluateAngleDerivativeStationary(double theta, double lsinTheta, double lcosTheta,
            double groundDistance, double totalHeight) {
        double numerator = groundDistance * lcosTheta - totalHeight * lsinTheta;
        double denominator = totalHeight * totalHeight + groundDistance * groundDistance;
        return numerator / denominator - 1;
    }
}
