// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Robot;

/**Calculates the launch parameters to make a parbolic shot at a target */
public class AutoAimCalculatorV2 {
    //**Global angle of the pivot */
    public static Rotation2d theta;
    //*Robot heading */
    public static Rotation2d alpha;
    //*Time until note reaches target */
    public static double time;
    //**Distance from zero - a measure of how close the approximation is to the solution */
    public static double error;

    //#region symbols
    private static final double beta = Constants.ArmConstants.LAUNCHER_ANGLE_WITH_PIVOT.getRadians();
    private static final double l = Constants.ArmConstants.PIVOT_LENGTH;
    private static final double h = Constants.ArmConstants.PIVOT_HEIGHT;
    private static final double v_s = Constants.ArmConstants.NOTE_LAUNCH_SPEED;
    private static final double g = 9.8;
    //#endregion

    /**
     * Calculate theta, alpha, and time
     */
    public static void compute(Pose2d robotPose, ChassisSpeeds velocity) {
        Pose3d target = Constants.blueSpeakerPose;
        SolutionEntry guess = new SolutionEntry(0.7, Math.PI, 0.5);

        if (Robot.alliance == Alliance.Red) {
            target = Constants.redSpeakerPose;
            guess = new SolutionEntry(0.7, 0, 0.5);
        }
        
        Pose3d relativeTarget = new Pose3d(target.getX() - robotPose.getX(), target.getY() - robotPose.getY(), target.getZ(), new Rotation3d());
        for (int i = 0; i < Constants.ArmConstants.PIVOT_APPROXIMATION_PRECISION; i++)
            guess = getNextEntry(guess, relativeTarget.getX(), relativeTarget.getY(), relativeTarget.getZ(), velocity.vxMetersPerSecond, velocity.vyMetersPerSecond);
        theta = Rotation2d.fromRadians(guess.x);
        alpha = Rotation2d.fromRadians(guess.y);
        time = guess.z;
    }

    private static double[][] computeJacobian(SolutionEntry p, double v_x, double v_y) {
        return new double[][] {
            {l * p.sinX * p.sinY + v_s * p.z * p.sinB * p.sinY,
            v_s * p.z * p.cosY * p.cosB - l * p.cosX * p.cosY,
            v_x + v_s * p.sinY * p.cosB},
            {l * p.cosY * p.sinX + v_s * p.z * p.sinB * p.cosY,
            l * p.cosX * p.sinY - v_s * p.z * p.sinY * p.cosB,
            v_y + v_s * p.cosY * p.cosB},
            {l * p.cosX - v_s * p.z * p.cosB,
            0,
            v_s * p.sinB - g * p.z}
        };
    }

    private static double[][] invert(double[][] m) {
        double inv_det = 1.0 / (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            + m[0][1] * (m[1][2] * m[2][0] - m[1][0] * m[2][2])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]));
        
        double[][] m_result = new double[][] {
            {m[1][1] * m[2][2] - m[1][2] * m[2][1],
            m[0][2] * m[2][1] - m[0][1] * m[2][2],
            m[0][1] * m[1][2] - m[0][2] * m[1][1]},
            {m[1][2] * m[2][0] - m[1][0] * m[2][2],
            m[0][0] * m[2][2] - m[0][2] * m[2][0],
            m[0][2] * m[1][0] - m[0][0] * m[1][2]},
            {m[1][0] * m[2][1] - m[1][1] * m[2][0],
            m[0][1] * m[2][0] - m[0][0] * m[2][1],
            m[0][0] * m[1][1] - m[0][1] * m[1][0]}
        };

        for (int x = 0; x < 3; x++)
            for (int y = 0; y < 3; y++)
                m_result[x][y] *= inv_det;

        return m_result;
    }

    private static double[] transformLinear(SolutionEntry p, double[][] inv, double t_x, double t_y, double t_z, double v_x, double v_y) {
        double x = v_x * p.z + v_s * p.z * p.cosB * p.sinY - t_x - l * p.cosX * p.sinY;
        double y = v_y * p.z + v_s * p.z * p.cosB * p.cosY - t_y - l * p.cosX * p.cosY;
        double z = v_s * p.z * p.sinB - 0.5 * g * p.z * p.z - t_z + l * p.sinX + h;

        double[] prod = new double[] {
            x * inv[0][0] + y * inv[0][1] + z * inv[0][2],
            x * inv[1][0] + y * inv[1][1] + z * inv[1][2],
            x * inv[2][0] + y * inv[2][1] + z * inv[2][2],
        };

        error = x * x + y * y + z * z;
        return new double[] {p.x - prod[0], p.y - prod[1], p.z - prod[2]};
    }

    private static SolutionEntry getNextEntry(SolutionEntry p, double t_x, double t_y, double t_z, double v_x, double v_y) {
        p.compute();
        double[][] j = computeJacobian(p, v_x, v_y);
        double[][] inv = invert(j);
        double[] next = transformLinear(p, inv, t_x, t_y, t_z, v_x, v_y);
        return new SolutionEntry(next[0], next[1], next[2]);
    }

    private static class SolutionEntry {
        public final double x, y, z;
        public double sinX, cosX, sinY, cosY, sinB, cosB;
        
        public SolutionEntry(double _x, double _y, double _z) {
            x = _x;
            y = _y;
            z = _z;
        }

        public void compute() {
            sinX = Math.sin(x);
            cosX = Math.cos(x);
            sinY = Math.sin(y);
            cosY = Math.cos(y);
            sinB = Math.sin(beta - x);
            cosB = Math.cos(beta - x);
        }
    }
}
