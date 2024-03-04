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
    public static final double ROBOT_WHEEL_BASE_WIDTH = Units.inchesToMeters(21.5);
    public static final double ROBOT_WHEEL_BASE_LENGTH = Units.inchesToMeters(23.5);
    public static final double MAX_DRIVE_SPEED = 1.5;
    public static final double FAST_DRIVE_SPEED = 3;
    public static final double MAX_ANGULAR_SPEED = 2;
    public static final double DRIVE_TOLERANCE_PERCENT = 0.015;


    public static final int LED_LENGTH = 200;

    public static final Pose3d redSpeakerPose = new Pose3d(16.58 , 4.98, Units.inchesToMeters(83), new Rotation3d());
    public static final Pose3d blueSpeakerPose = new Pose3d();

    public static final Pose3d speakerPose = new Pose3d(Units.inchesToMeters(-54), 0, Units.inchesToMeters(83), new Rotation3d());

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
                new Translation3d(-Units.inchesToMeters(-13.5), 0, -Units.inchesToMeters(14.5)), 
                new Rotation3d(Units.degreesToRadians(CAM_PITCH),0, Math.PI)
            );

        public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();



        //All of the target April Tag poses on the field to determin where the robot is
        
        public static final Pose3d TARGET_1_POSE = new Pose3d(15.08 , .25, 1.36, new Rotation3d(0,0,2*Math.PI/3));
        public static final Pose3d TARGET_2_POSE = new Pose3d(16.19 , .88, 1.36, new Rotation3d(0,0,2*Math.PI/3));
        public static final Pose3d TARGET_3_POSE = new Pose3d(16.58 , 4.98, 1.45, new Rotation3d(0,0,Math.PI));
        public static final Pose3d TARGET_4_POSE = new Pose3d(16.58, 5.55, 1.45, new Rotation3d(0,0,Math.PI));
        public static final Pose3d TARGET_5_POSE = new Pose3d(14.7,8.2,1.36, new Rotation3d(0,0,-Math.PI/4));
        public static final Pose3d TARGET_6_POSE = new Pose3d(1.84 , 8.2, 1.36, new Rotation3d(0,0,-Math.PI/4));
        public static final Pose3d TARGET_7_POSE = new Pose3d(-.04,5.55, 1.45, new Rotation3d());
        public static final Pose3d TARGET_8_POSE = new Pose3d(-.04 , 4.98, 1.45, new Rotation3d());
        public static final Pose3d TARGET_9_POSE = new Pose3d(.36,0.88,1.36, new Rotation3d(0,0,Math.PI/3));
        public static final Pose3d TARGET_10_POSE = new Pose3d(1.46 , 0.25,1.36, new Rotation3d(0,0,Math.PI/3));
        public static final Pose3d TARGET_11_POSE = new Pose3d(11.9,3.71,1.32, new Rotation3d(0,0,-Math.PI/3));
        public static final Pose3d TARGET_12_POSE = new Pose3d(11.9,4.5,1.32, new Rotation3d(0,0,Math.PI/3));
        public static final Pose3d TARGET_13_POSE = new Pose3d(11.22,4.11,1.32, new Rotation3d(0,0,Math.PI));
        public static final Pose3d TARGET_14_POSE = new Pose3d(5.32,4.11,1.32, new Rotation3d(0,0,0));
        public static final Pose3d TARGET_15_POSE = new Pose3d(4.64,4.5,1.32, new Rotation3d(0,0,2*Math.PI/3));
        public static final Pose3d TARGET_16_POSE = new Pose3d(4.64,3.71,1.32, new Rotation3d(0,0,-Math.PI/1.5));
        

    } 
    
    public static class ArmConstants{
        public static final InvertedValue PIVOT_INVERTED = InvertedValue.CounterClockwise_Positive;
        public static final double PIVOT_GEAR_RATIO = 30 * (72/24) * (72/24);
        public static final Rotation2d PIVOT_INITIALIZE_POSITION = Rotation2d.fromDegrees(-15);
        public static final double PIVOT_FEED_FORWARD = 0;
        //The angle made by the launcher's output with the pivot "arms" as the positive x-axis
        public static final Rotation2d LAUNCHER_ANGLE_WITH_PIVOT = Rotation2d.fromDegrees(45);
        public static final double PIVOT_HEIGHT = Units.inchesToMeters(16);
        public static final double PIVOT_LENGTH = Units.inchesToMeters(19);
        public static final int PIVOT_APPROXIMATION_PRECISION = 4; //Number of iterations for newton's method
        public static final Rotation2d PIVOT_MAX = Rotation2d.fromDegrees(115);
        public static final double PIVOT_SENSOR_OFFSET = -.202;
        public static final double NOTE_LAUNCH_SPEED = 5; //m/s
        public static final double NOTE_RETRACTION_TIME = 0.5;
        public static final boolean USE_REMOTE_PIVOT_SENSOR = false;
        public static final double PIVOT_ENCODER_DEADZONE = 0.01; //rotations
        public static final double MAX_PIVOT_DEVIATION = 0.1;
    }
}
