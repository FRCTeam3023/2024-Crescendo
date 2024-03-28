// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Commands.JoystickDrive;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Util.AutoAimCalculator;
import frc.robot.Util.PIDDisplay;
import frc.robot.Util.ProfiledWPILibSetter;
import frc.robot.Util.SparkMaxSetter;
import frc.robot.Util.TalonFXsetter;

public class Drivetrain extends SubsystemBase {

  public static boolean allModuleHomeStatus = false;

  public static boolean autoAimDrivetrain = false;
  
  //ADIS16470_IMU gyro = new ADIS16470_IMU();
  Pigeon2 pigeonGyro = new Pigeon2(20);
  private static double lastGyroReading = 4; //4 is never within the rotation; this is the initialization value
  private static int consecutiveFaults = 0; //-1 when failed

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
  ProfiledPIDController turnController = new ProfiledPIDController(4, 0, 0, new Constraints(5, 12));

  ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
  GenericEntry targetPositionEntry = armTab.add("Aiming Target Position", "placeholder").withPosition(1, 0).withSize(2, 1).getEntry();
  GenericEntry relativePositionEntry = armTab.add("Relative Target Position", "placeholder").withPosition(3, 0).withSize(2, 1).getEntry();
  GenericEntry robotPoseEntry = armTab.add("Robot Pose", "placeholder").withPosition(1, 1).withSize(2, 1).getEntry();
  GenericEntry targetRotationEntry = armTab.add("Aim Heading", 0).withPosition(4, 1).getEntry();

   //Swerve Drive Pose Estimator, uses the combination of encoder data and vision to estimate pose on the field
  private static final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
    kinematics, 
    new Rotation2d(), 
    new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()},
    new Pose2d(0,0, new Rotation2d()),
    MatBuilder.fill(Nat.N3(), Nat.N1(),0.05,0.05,0.05), //Standard deviations for state estimate, (m,m,rad). Increase to trust less
    MatBuilder.fill(Nat.N3(), Nat.N1(),0.3,0.3,0.1) //Standard deviations for vision estimate, (m,m,rad). Increase to trust less
    );

  /** Shuffelboard tab to display telemetry such as heading, homing status, gyro drift, etc*/
  private static final ShuffleboardTab telemTab = Shuffleboard.getTab("Telemetry");
  /** Shuffleboard entry to display robot heading */
  private static GenericEntry headingEntry = telemTab.add("Robot Heading", 0).withPosition(0, 0).getEntry();
  private static GenericEntry gyroRawEntry = telemTab.add("Raw Gyro", 0).withPosition(0, 1).getEntry();
  private static GenericEntry gyroDifferenceEntry = telemTab.add("Gyro Difference", 0).withPosition(0, 2).getEntry();
  private static GenericEntry isFieldRelative = telemTab.add("Field Relative", false).withPosition(2, 2).getEntry();
  private static GenericEntry poseEntry = telemTab.add("Pose", new Pose2d().toString()).withPosition(1, 0).withSize(3, 1).getEntry();
  private static GenericEntry poseX = telemTab.add("Override Pose X", 0).withPosition(1, 1).getEntry();
  private static GenericEntry poseY = telemTab.add("Override Pose Y", 0).withPosition(2, 1).getEntry();
  private static GenericEntry poseH = telemTab.add("Override Pose H", 0).withPosition(3, 1).getEntry();
  private static GenericEntry gyroFaultEntry = telemTab.add("Gyro Fault", false).withPosition(3, 2).getEntry();
  //private static Field2d field = new Field2d();

  public Drivetrain() {
    poseX.setDouble(0);
    poseY.setDouble(0);
    poseH.setDouble(0);
    turnController.enableContinuousInput(-Math.PI, Math.PI);
    turnController.setTolerance(Units.degreesToRadians(1));
    PIDDisplay.PIDList.addOption("Aim Turn PID", new ProfiledWPILibSetter(List.of(turnController)));

    // gyro.configCalTime(CalibrationTime._32ms);
    //recalibrate gyro and initialize starting pose
    calibrateGyro();


    AutoBuilder.configureHolonomic(
      this::getPose,
      this::setPose, 
      this::getChassisSpeeds , 
      this::driveRobotAuto, 
      new HolonomicPathFollowerConfig(
        new PIDConstants(4), 
        new PIDConstants(4),
        ModuleConstants.MAX_SPEED, 
        Units.inchesToMeters(16.5), 
        new ReplanningConfig()),
      () -> {
          return Robot.alliance == Alliance.Red;
        },
      this
    );

    TalonFXsetter swerveDriveMotors = new TalonFXsetter(SwerveModule.getDriveConfigurators(), SwerveModule.getTalonFXConfig());
    SparkMaxSetter swerveTurnMotors = new SparkMaxSetter(SwerveModule.getTurnPIDControllers());
    
    PIDDisplay.PIDList.addOption("Swerve Drive Motors", swerveDriveMotors);
    PIDDisplay.PIDList.addOption("Swerve Turn Motors", swerveTurnMotors);

    //telemTab.add(field).withPosition(4, 0).withSize(5, 3);
  }

  @Override
  public void periodic() {
    //computes the auto aim positions periodically
    AutoAimCalculator.computeAngle(getPose(), getFieldRelativeSpeeds());



    //field.setRobotPose(new Pose2d(3, 3, new Rotation2d(Math.PI / 4)));
    headingEntry.setDouble(getPose().getRotation().getDegrees());
    gyroRawEntry.setDouble(getChassisAngle().getDegrees());
    gyroDifferenceEntry.setDouble(getChassisAngle().getDegrees() - getPose().getRotation().getDegrees());
    isFieldRelative.setBoolean(JoystickDrive.fieldRelativeDrive);
    
    // if (poseX.getDouble(0) != 0 || poseY.getDouble(0) != 0 || poseH.getDouble(0) != 0)
    //   setPose(new Pose2d(poseX.getDouble(0), poseY.getDouble(0), Rotation2d.fromDegrees(poseH.getDouble(0))));
    // else
      poseEstimator.update(getChassisAngle(), getModulePositions());

    poseEntry.setString(getPose().toString());    
    displayAllModuleSwitches();
  }


//#region Driving
  /**
   * Primary drive method for the robot
   * @param speeds Speeds that the robot should be moving in - positive x is foreward, y is left, omega is CCW
   * @param isFieldRelative true if running in field relative mode
   */
  public void drive(ChassisSpeeds speeds, boolean isFieldRelative){
    
    if(isFieldRelative){
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation());
    } 

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

    setModuleStates(moduleStates);
  }

  public void resetTurnController() {
    turnController.reset(getPose().getRotation().getRadians());
  }

  public void driveFacingTarget(ChassisSpeeds speeds, boolean isFieldRelative) {
    Translation2d relativeTargetTranslation = AutoAimCalculator.translatedPose.toPose2d().getTranslation();//getPose().getTranslation().minus(targetPose.toPose2d().getTranslation());
    Rotation2d targetRotation = Rotation2d.fromRadians(Math.atan2(relativeTargetTranslation.getY(), relativeTargetTranslation.getX())).plus(Rotation2d.fromDegrees(180));
    double rotationSpeed = turnController.calculate(getPose().getRotation().getRadians(), targetRotation.getRadians());
    drive(new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, rotationSpeed), isFieldRelative);

    targetPositionEntry.setString(getPose().plus(new Transform2d(relativeTargetTranslation.getX(), relativeTargetTranslation.getY(), new Rotation2d())).toString());
    targetRotationEntry.setDouble(targetRotation.getDegrees());
    relativePositionEntry.setString(relativeTargetTranslation.toString());
    robotPoseEntry.setString(getPose().toString());

          // double horizontalSpeed = Constants.ArmConstants.NOTE_LAUNCH_SPEED * Math.cos(Pivot.holdPosition.getRadians() - Constants.ArmConstants.launcherAngleWithPivot.getRadians());
      // double airTime = (Math.pow(relativeTargetTranslation.getY(),2) + Math.pow(relativeTargetTranslation.getX(),2)) / horizontalSpeed;
      // relativeTargetTranslation = new Translation2d(relativeTargetTranslation.getX() - drivetrain.getChassisSpeeds().vxMetersPerSecond * airTime,
      //   relativeTargetTranslation.getY() - drivetrain.getChassisSpeeds().vyMetersPerSecond * airTime); 
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds){
    drive(chassisSpeeds, false);
  }


  public void driveRobotAuto(ChassisSpeeds chassisSpeeds){
    if(autoAimDrivetrain){
      driveFacingTarget(chassisSpeeds, false);
    } else {
      drive(chassisSpeeds, false);
    }
  }
//#endregion

  /** Sets drivetrain to 0 speeds */
  public void stop(){
    drive(new ChassisSpeeds(), false);
  }

  public boolean atHeadingTarget(){
    return turnController.atGoal();

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

  public void displayAllModuleSwitches () {
    frontLeft.displayStatus();
    frontRight.displayStatus();
    backLeft.displayStatus();
    backRight.displayStatus();
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
    double reading = -pigeonGyro.getAngle();
    if (consecutiveFaults == -1) {
      //use another gyro or reference?
      return Rotation2d.fromDegrees(reading);
    }
    else if (consecutiveFaults > Constants.MAX_ALLOWED_GYRO_FAULTS) {
      onGyroFault();
      return Rotation2d.fromDegrees(lastGyroReading);
    }
    else if (lastGyroReading != 4 && Math.abs(reading - lastGyroReading) > Constants.MAX_GYRO_DIFFERENCE) {
      consecutiveFaults++;
      return Rotation2d.fromDegrees(lastGyroReading);
    }
    consecutiveFaults = 0;
    lastGyroReading = reading;

    return Rotation2d.fromDegrees(reading);
    //return Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kZ));
  }

  public void onGyroFault() {
    consecutiveFaults = -1;
    gyroFaultEntry.setBoolean(true);
    // LED.interuptSignal(new SequentialCommandGroup(
    //   new InstantCommand(() -> LED.setLEDColor(0, Constants.LED_LENGTH, LED.COLORS.OFF)),
    //   new WaitCommand(0.25),
    //   new InstantCommand(() -> LED.setLEDColor(0, Constants.LED_LENGTH, LED.COLORS.HOTPINK)),
    //   new WaitCommand(0.25),
    //   new InstantCommand(() -> LED.setLEDColor(0, Constants.LED_LENGTH, LED.COLORS.OFF)),
    //   new WaitCommand(0.25),
    //   new InstantCommand(() -> LED.setLEDColor(0, Constants.LED_LENGTH, LED.COLORS.HOTPINK)),
    //   new WaitCommand(0.25)
    // ));


  }

  /**
   * Returns robot relative speeds
   * @return chassis speeds
   */
  public ChassisSpeeds getChassisSpeeds(){
    return kinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getPose().getRotation());
  }

  /** Calibrates gyro - will take significant time so put in beginning of code */
  public void calibrateGyro(){
    //gyro.calibrate();
    pigeonGyro.reset();
    Timer.delay(1);
  }

}
