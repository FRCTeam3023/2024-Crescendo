// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    /** Width between robot wheels in meters */ 
    public static final double ROBOT_WHEEL_BASE = Units.inchesToMeters(21.5);
    public static final double MAX_DRIVE_SPEED = 1;
    public static final double MAX_ANGULAR_SPEED = 3;
    public static final double DRIVE_TOLERANCE_PERCENT = 0.03;

    public class ModuleConstants{
        /** Overall max speed of the module in m/s */
        public static final double MAX_SPEED = 5;

        /** rotational offset in radians of module 1 during homing */
        public static final double MODULE1_OFFSET = -Math.PI/4;
        /** rotational offset in radians of module 2 during homing */
        public static final double MODULE2_OFFSET = Math.PI/4;
        /** rotational offset in radians of module 3 during homing */
        public static final double MODULE3_OFFSET = Math.PI/4;
        /** rotational offset in radians of module 4 during homing */
        public static final double MODULE4_OFFSET = -Math.PI/4;

        /** Gear Ratio of the drive motor */
        public static final double DRIVE_GEARING = 8;

        /**Gear ratio of the turning motor */
        public static final double TURN_GEARING = 2.89 * 2.89 * 6;

        /**Diameter of the billet wheel */
        public static final double WHEEL_DIA = 3.875;

    }

    public static class PhotonConstants {

        //Camera position relative to the bot to translate vision data to the center of the robot instead of based around the camera's reference
        public final static double CAM_PITCH = 15; //degrees
        public final static Transform3d CAMERA_TO_ROBOT = 
            new Transform3d(
                new Translation3d(0, Units.inchesToMeters(5.5), -Units.inchesToMeters(11.75)), 
                new Rotation3d(Units.degreesToRadians(CAM_PITCH),0,0)
            );

        public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();



        //All of the target April Tag poses on the field to determin where the robot is
        
        public static final Pose3d TARGET_1_POSE = new Pose3d(15.54 , 1.06, Units.inchesToMeters(18.13), new Rotation3d(0,0,Math.PI));
        public static final Pose3d TARGET_2_POSE = new Pose3d(15.54 , 2.75, Units.inchesToMeters(18.13), new Rotation3d(0,0,Math.PI));
        public static final Pose3d TARGET_3_POSE = new Pose3d(15.54 , 4.42, Units.inchesToMeters(18.13), new Rotation3d(0,0,Math.PI));

        public static final Pose3d TARGET_4_POSE = new Pose3d(16.21 , 6.72, Units.inchesToMeters(27.38), new Rotation3d(0,0,Math.PI));

        

        public static final Pose3d TARGET_5_POSE = new Pose3d(0.33 , 6.75, Units.inchesToMeters(27.38), new Rotation3d());

        public static final Pose3d TARGET_6_POSE = new Pose3d(1 , 4.42, Units.inchesToMeters(18.13), new Rotation3d());
        public static final Pose3d TARGET_7_POSE = new Pose3d(1,2.75, Units.inchesToMeters(18.13), new Rotation3d());
        public static final Pose3d TARGET_8_POSE = new Pose3d(1 , 1.06, Units.inchesToMeters(18.13), new Rotation3d());
    } 
    
    public static class ArmConstants{
        public static final InvertedValue pivotInverted = InvertedValue.Clockwise_Positive;
        public static final double pivotGearRatio = 50;
        public static final Rotation2d pivotInitializePosition = Rotation2d.fromDegrees(-15);
        public static final double pivotFeedForward = 0.5;
        //The angle made by the launcher's output with the pivot "arms" as the positive x-axis
        public static final Rotation2d launcherAngleWithPivot = Rotation2d.fromDegrees(45);
    }
}
