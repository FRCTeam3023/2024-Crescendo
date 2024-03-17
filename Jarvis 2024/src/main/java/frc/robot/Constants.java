// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Pose2d;
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
    public static final double FAST_DRIVE_SPEED = 5;
    public static final double MAX_ANGULAR_SPEED = 2;
    public static final double DRIVE_TOLERANCE_PERCENT = 0.015;

    public static final double MAX_GYRO_DIFFERENCE = 20; //Degrees
    public static final int MAX_ALLOWED_GYRO_FAULTS = 10;

    /**Total length of the LED strip */
    public static final int LED_LENGTH = 41;

    //target pose for the opening of the speakers
    public static final Pose3d redSpeakerPose = new Pose3d(16.58, 5.55, Units.inchesToMeters(83), new Rotation3d());
    public static final Pose3d blueSpeakerPose = new Pose3d(0 , 5.55, Units.inchesToMeters(83), new Rotation3d());

    //target positions for auto alignment into the amps
    public static final Pose2d blueAmpPose = new Pose2d(1.83,7.65, Rotation2d.fromDegrees(90));
    public static final Pose2d redAmpPose = new Pose2d(14.7,7.65,Rotation2d.fromDegrees(90));
    // public static final Pose3d speakerPose = new Pose3d(Units.inchesToMeters(-54), 0, Units.inchesToMeters(83), new Rotation3d());

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
        public static final double WHEEL_DIA = 3.88;

    }

    public static class PhotonConstants {

        //Camera position relative to the bot to translate vision data to the center of the robot instead of based around the camera's reference
        public final static double CAM_PITCH = Units.degreesToRadians(50); //degrees
        public final static Transform3d ROBOT_TO_CAMERA = 
            new Transform3d(
                new Translation3d(-Units.inchesToMeters(15.0), 0, Units.inchesToMeters(14.5)), 
                new Rotation3d(0, -CAM_PITCH, Math.PI)
            );

        public static final Transform3d CAMERA_TO_ROBOT = ROBOT_TO_CAMERA.inverse();



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

        /**Angle for floor pickup */
        public static final Rotation2d PICKUP_POSITION = new Rotation2d();
        /**Angle for speaker shot from the subwoofer */
        public static final Rotation2d SPEAKER_POSITION = Rotation2d.fromDegrees(13);
        /**Angle for shooting down into the Amp */
        public static final Rotation2d AMP_POSITION = Rotation2d.fromDegrees(110);

        /**Which way the pivot direction is positive*/
        public static final InvertedValue PIVOT_INVERTED = InvertedValue.CounterClockwise_Positive;

        /**Gear ratio from pivot motor rotor to mechanism */
        public static final double PIVOT_GEAR_RATIO = 30.0 * (72.0/24.0) * (36.0/12.0);

        /**Relative to horizontal in global angle where is 0 degrees local*/
        public static final Rotation2d PIVOT_INITIALIZE_POSITION = Rotation2d.fromDegrees(-15);

        
        public static final double PIVOT_FEED_FORWARD = 0;

        /**The angle made by the launcher's output with the pivot "arms" as the positive x-axis*/
        public static final Rotation2d LAUNCHER_ANGLE_WITH_PIVOT = Rotation2d.fromDegrees(45);

        /**What speed the shooter flywheels should spin at*/
        public static final double SHOOTER_RPM = 2600;
        /**The maximum difference between the shooter's RPM and the target RPM*/
        public static final double MAX_SHOOTER_RPM_ERROR = 100;

        /**Height of the pivot point in meters */
        public static final double PIVOT_HEIGHT = Units.inchesToMeters(16);
        /**Length of the pivot arm in meters */
        public static final double PIVOT_LENGTH = Units.inchesToMeters(19);
        /**Number of iterations for newton's method*/
        public static final int PIVOT_APPROXIMATION_PRECISION = 4; 
        /**Max allowable pivot angle relative to local origin */
        public static final Rotation2d PIVOT_MAX = Rotation2d.fromDegrees(115);
        /**Cancoder Magnet offset in rotations relative to local origin */
        public static final double PIVOT_SENSOR_OFFSET = .181;

        public static final double NOTE_LAUNCH_SPEED = 5; //m/s
        
        /**True if set to using remote CANCoder, false if rotor sensor */
        public static final boolean USE_REMOTE_PIVOT_SENSOR = true;
        /**Maximum movement speed in rad/s still allowable to reset the position of the rotor relative to the cancoder*/
        public static final double PIVOT_REST_AMBIGUITY = 0.01;
        /**Minimum wait time for the encoder to reset rotor relative position */ 
        public static final double REST_TIME = 2; //s
        /**Minimum amount in radians the relative positions of the remote sensor and the rotor sensor are differing for the code to reset the offset between the two*/
        public static final double MAX_PIVOT_SENSOR_DEVIATION = 0.1; //rad
        /**The maximum difference between the pivot and target angle*/
        public static final double MAX_PIVOT_DEVIATION = 0.1; //rad
    }
}
