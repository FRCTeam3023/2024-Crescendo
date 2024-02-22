// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Util.PIDDisplay;
import frc.robot.Util.SparkMaxSetter;
import frc.robot.Util.TalonFXsetter;

public class Drivetrain extends SubsystemBase {

  public static boolean allModuleHomeStatus = false;
  
  ADIS16470_IMU gyro = new ADIS16470_IMU();

  //module objects
  private static final SwerveModule frontLeft = new SwerveModule(1, 1, 5, ModuleConstants.MODULE1_OFFSET, InvertedValue.Clockwise_Positive, 0); // Module 1
  private static final SwerveModule frontRight = new SwerveModule(2, 2, 6, ModuleConstants.MODULE2_OFFSET, InvertedValue.CounterClockwise_Positive, 1); // Module 2
  private static final SwerveModule backLeft = new SwerveModule(3, 3, 7, ModuleConstants.MODULE3_OFFSET, InvertedValue.Clockwise_Positive, 2); // Module 3
  private static final SwerveModule backRight = new SwerveModule(4, 4, 8, ModuleConstants.MODULE4_OFFSET, InvertedValue.CounterClockwise_Positive, 3); // Module 4

  //module positions
  private static final Translation2d frontLeftLocation = new Translation2d(Constants.ROBOT_WHEEL_BASE_LENGTH/2 , Constants.ROBOT_WHEEL_BASE_WIDTH/2);
  private static final Translation2d frontRightLocation = new Translation2d(Constants.ROBOT_WHEEL_BASE_LENGTH/2, -Constants.ROBOT_WHEEL_BASE_WIDTH/2);
  private static final Translation2d backLeftLocation = new Translation2d(-Constants.ROBOT_WHEEL_BASE_LENGTH/2,Constants.ROBOT_WHEEL_BASE_WIDTH/2);
  private static final Translation2d backRightLocation = new Translation2d(-Constants.ROBOT_WHEEL_BASE_LENGTH/2, -Constants.ROBOT_WHEEL_BASE_WIDTH/2);

  //kinimatics object for swerve drive storing module positions
  private static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

   //Swerve Drive Pose Estimator, uses the combination of encoder data and vision to estimate pose on the field
  private static final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
    kinematics, 
    new Rotation2d(), 
    new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()},
    new Pose2d(0,0, new Rotation2d()),
    MatBuilder.fill(Nat.N3(), Nat.N1(),0.05,0.05,0.05), //Standard deviations for state estimate, (m,m,rad). Increase to trust less
    MatBuilder.fill(Nat.N3(), Nat.N1(),0.9,0.9,0.9) //Standard deviations for vision estimate, (m,m,rad). Increase to trust less
    );

  /** Shuffelboard tab to display telemetry such as heading, homing status, gyro drift, etc*/
  private static final ShuffleboardTab telemTab = Shuffleboard.getTab("Telemetry");
  /** Shuffleboard entry to display robot heading */
  private static GenericEntry headingEntry = telemTab.add("Robot Heading", 0).withPosition(9, 0).getEntry();
  private static GenericEntry poseEntry = telemTab.add("Pose", new Pose2d().toString()).withPosition(9, 1).getEntry();


  public Drivetrain() {
    gyro.configCalTime(CalibrationTime._32ms);
    //recalibrate gyro and initialize starting pose
    calibrateGyro();


    AutoBuilder.configureHolonomic(
      this::getPose,
      this::setPose, 
      this::getChassisSpeeds , 
      this::driveRobotRelative, 
      new HolonomicPathFollowerConfig(
        new PIDConstants(4), 
        new PIDConstants(4),
        ModuleConstants.MAX_SPEED, 
        Units.inchesToMeters(16.5), 
        new ReplanningConfig()),
      () -> {
          var alliance = DriverStation.getAlliance();
          if(alliance.isPresent()){
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
      this
    );

    TalonFXsetter swerveDriveMotors = new TalonFXsetter(SwerveModule.getDriveConfigurators(), SwerveModule.getTalonFXConfig());
    SparkMaxSetter swerveTurnMotors = new SparkMaxSetter(SwerveModule.getTurnPIDControllers());
    
    PIDDisplay.PIDList.addOption("Swerve Drive Motors", swerveDriveMotors);
    PIDDisplay.PIDList.addOption("Swerve Turn Motors", swerveTurnMotors);


  }

  @Override
  public void periodic() {
    headingEntry.setDouble(getPose().getRotation().getDegrees());

    poseEstimator.update(getChassisAngle(), getModulePositions());
    poseEntry.setString(getPose().toString());
  }


  /**
   * Primary drive method for the robot
   * @param speeds Speeds that the robot should be moving in - positive x is foreward, y is left, omega is CCW
   * @param isFieldRelative true if running in field relative mode
   */
  public void drive(ChassisSpeeds speeds, boolean isFieldRelative){

    double omega = speeds.omegaRadiansPerSecond;

    omega = Math.signum(omega) * Math.min(Math.abs(omega), Constants.MAX_ANGULAR_SPEED);

    if(isFieldRelative){
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation());
    } 

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

    setModuleStates(moduleStates);
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds){
    drive(chassisSpeeds, false);
  }

  /** Sets drivetrain to 0 speeds */
  public void stop(){
    drive(new ChassisSpeeds(), false);
  }



  /**
   * Sets the desired module states for all of the modules - desaturates the max speeds first
   * @param moduleStates array of target module states
   */
  public void setModuleStates(SwerveModuleState[] moduleStates){
    //Desaturate module speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, ModuleConstants.MAX_SPEED);

    //assign states to the modules
    frontLeft.setDesiredState(moduleStates[0]);
    frontRight.setDesiredState(moduleStates[1]);
    backLeft.setDesiredState(moduleStates[2]);
    backRight.setDesiredState(moduleStates[3]);
  }

  /**
   * Returns the module positions of all four modules, frontleft to backright
   * @return array of swerve module positions
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    };
  }

  public void homeAllModules(){
    frontLeft.home();
    frontRight.home();
    backLeft.home();
    backRight.home();

    //if all modules are done then change variable state to true
    allModuleHomeStatus = frontLeft.homeStatus && frontRight.homeStatus && backLeft.homeStatus && backRight.homeStatus;

  }

  /**
   * Resets all the modules' home status to false
   */
  public void resetHomeStatus(){
    frontLeft.setHomeStatus(false);
    frontRight.setHomeStatus(false);
    backLeft.setHomeStatus(false);
    backRight.setHomeStatus(false);

    allModuleHomeStatus = false;
  }

  /**
   * Reset robot pose to new pose
   * @param newPose new pose
   */
  public void setPose(Pose2d newPose){
    poseEstimator.resetPosition(getChassisAngle(), getModulePositions(), newPose);
  }

  /**
   * @return Current pose of the robot on the field
   */
  public Pose2d getPose(){
    return poseEstimator.getEstimatedPosition();
  }

  public static void addVisionMeasurement(Pose2d visionPose, double timestamp){
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /**
   * Method to directly get gyro angle, use robot pose for actual heading on the field
   * @return Gyro angle
   */
  public Rotation2d getChassisAngle(){
    return Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kZ));
  }

  /**
   * Returns robot relative speeds
   * @return chassis speeds
   */
  public ChassisSpeeds getChassisSpeeds(){
    return kinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
  }

  /** Calibrates gyro - will take significant time so put in beginning of code */
  public void calibrateGyro(){
    gyro.calibrate();
  }



}
