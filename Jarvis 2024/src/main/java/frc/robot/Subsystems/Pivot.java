// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.List;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Util.Gains;
import frc.robot.Util.PIDDisplay;
import frc.robot.Util.TalonFXsetter;

public class Pivot extends SubsystemBase {
  private final TalonFX pivotMotor = new TalonFX(10);
  private TalonFXConfiguration pivotConfiguration;
  private final CANcoder pivotSensor = new CANcoder(9);
  private double lastPivotPosition = 0;
  private double pivotRestTime = -1;
  private final CANcoderConfiguration pivotEncoderConfig = new CANcoderConfiguration();
  private final Gains pivotGains = new Gains(25, 0, 0, 0, 10);

  private final TalonFX climberMotor = new TalonFX(14);
  private TalonFXConfiguration climberConfig;

  public static Rotation2d holdPosition = new Rotation2d();
  public static boolean climbMode = false;
  public static boolean previousClimbMode = false;

  private static final ShuffleboardTab armTab = Shuffleboard.getTab("Arm");

  private static final GenericEntry angleEntry = armTab.add("Local Angle", 0).withPosition(0, 0).getEntry();
  private static final GenericEntry sensorAngleEntry = armTab.add("Sensor Angle", 0).withPosition(1, 0).getEntry();
  private static final GenericEntry angleOffsetEntry = armTab.add("Angle Offset", 0).withPosition(0, 1).getEntry();
  private static final GenericEntry targetAngle = armTab.add("Target Angle",0).withPosition(0, 2).getEntry();
  private static final GenericEntry angleError = armTab.add("Angle Error",0).withPosition(0, 3).getEntry();
  private static final GenericEntry aimAngleEntry = armTab.add("Aim Angle",0).withPosition(3, 1).getEntry();
  private static final GenericEntry climberModeEntry = armTab.add("Climb Mode",false).withPosition(3, 2).getEntry();


  public Pivot() {

    //basic configuration for the pivot motor
    pivotConfiguration = new TalonFXConfiguration();

    pivotConfiguration.MotorOutput.Inverted = ArmConstants.PIVOT_INVERTED;
    pivotConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivotConfiguration.Voltage.PeakForwardVoltage = pivotGains.peakOutput;
    pivotConfiguration.Voltage.PeakReverseVoltage = pivotGains.peakOutput;

    pivotConfiguration.MotionMagic.MotionMagicCruiseVelocity = 5;
    pivotConfiguration.MotionMagic.MotionMagicAcceleration = 14;
    pivotConfiguration.MotionMagic.MotionMagicJerk = 200;

    pivotConfiguration.Slot0.kP = pivotGains.P;
    pivotConfiguration.Slot0.kD = pivotGains.D;
    pivotConfiguration.Slot0.kS = pivotGains.S;
    pivotConfiguration.Slot0.kD = pivotGains.V;

    pivotConfiguration.Audio.BeepOnBoot = false;
    pivotConfiguration.Audio.BeepOnConfig = false;

    //set feedback sensor as a remote CANcoder, resets the rotor sensor position every time it publishes values

    if (Constants.ArmConstants.USE_REMOTE_PIVOT_SENSOR) {
      pivotConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
      pivotConfiguration.Feedback.FeedbackRemoteSensorID = pivotSensor.getDeviceID();
      pivotConfiguration.Feedback.SensorToMechanismRatio = 1 / (2 * Math.PI);
    } else {
      pivotConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
      pivotConfiguration.Feedback.SensorToMechanismRatio = ArmConstants.PIVOT_GEAR_RATIO / (2.0 * Math.PI);
    }
    
    pivotMotor.getConfigurator().apply(pivotConfiguration);

    //---------------------------------------------------------------------------------------------------

    pivotEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    pivotEncoderConfig.MagnetSensor.MagnetOffset = ArmConstants.PIVOT_SENSOR_OFFSET;
    pivotSensor.getConfigurator().apply(pivotEncoderConfig);



    PIDDisplay.PIDList.addOption("Pivot", new TalonFXsetter(List.of(pivotMotor.getConfigurator()), pivotConfiguration));

    holdPosition = getLocalAngle();


    climberConfig = new TalonFXConfiguration();
    climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    climberMotor.getConfigurator().apply(climberConfig);

    pivotMotor.setPosition(getPivotEncoderPosition().getRadians());
  }

  @Override
  public void periodic() {
    telemUpdate();
    checkClimbStatus();
    // checkHoldPositionDisabled();
    //if (!Constants.ArmConstants.USE_REMOTE_PIVOT_SENSOR) checkRotorEncoder();
  }

//#region Angle Logic
  /**
   * Convert between local robot space and global world space
   * @param angle Local angle
   * @return Global angle
   */
  public static Rotation2d globalToLocalAngle(Rotation2d angle) {
    return angle.minus(ArmConstants.PIVOT_INITIALIZE_POSITION);
  }

  
  /**
   * Convert between local robot space and global world space
   * @param angle Global angle
   * @return Local angle
   */
  public static Rotation2d localToGlobalAngle(Rotation2d angle) {
    return angle.plus(ArmConstants.PIVOT_INITIALIZE_POSITION);
  }

  /**
   * Angle of the arm positive from starting configuration
   * @return angle of arm
   */
  public Rotation2d getLocalAngle(){
    return Rotation2d.fromRadians(pivotMotor.getPosition().getValue());
  }

  /**
   * The angle of the arm positive from horizontal based on the field
   * @return angle of arm
   */
  public Rotation2d getGlobalAngle(){
    return localToGlobalAngle(getLocalAngle());
  }
//#endregion

/**
 * Returns the position of the cancoder for the pivot 
 * @return Rotation2d of cancoder position - local angle
 */
  public Rotation2d getPivotEncoderPosition(){
    return Rotation2d.fromRotations(pivotSensor.getPosition().getValue());
  }

  /**
   * Returns the postion of the rotor sensor for the pivot - local angle
   * @return Current pivot position
   */
  public Rotation2d getPivotMotorPosition(){
    return Rotation2d.fromRadians(pivotMotor.getPosition().getValue());
  }

  /**
   * Set the closed loop motion magic control target of the pivot joint, adjustable between global and local control.
   * @param angle target angle to move to
   * @param isGlobal if angle being passed in is global or local
   */
  public void setPivotAngle(Rotation2d angle, boolean isGlobal) {
    climbMode = false;
    if (isGlobal) angle = globalToLocalAngle(angle);
    holdPosition = angle;
    angle = Rotation2d.fromRadians(Math.min(Math.max(angle.getRadians(), 0), Constants.ArmConstants.PIVOT_MAX.getRadians()));
    pivotMotor.setControl(new MotionMagicVoltage(angle.getRadians()));
  }

  /**
   * Check if pivot motor is at holdPosition
   * @return True if the difference is small enough
   */
  public boolean isAtTargetAngle() {
    return Math.abs(holdPosition.getRadians() - pivotMotor.getPosition().getValueAsDouble()) < Constants.ArmConstants.MAX_PIVOT_DEVIATION;
  }

  /**
   * Check if prepping the shooter will cause the note to hit the ground
   * @return True if the note does not hit the ground when prepping the shooter
   */
  public boolean noteClearsGround() {
    return pivotMotor.getPosition().getValueAsDouble() > Math.toRadians(10);
  }
  
  public void setPivotDutyCycle(double speed){
    pivotMotor.setControl(new DutyCycleOut(speed));
    holdPosition = getLocalAngle();
  }

  /**
   * Compare the rotor encoder's position with the remote sensor and update when minimal motion is detected
   */
  public void checkRotorEncoder() {
    double sensorPosition = pivotSensor.getPosition().getValueAsDouble();
    double rotorPosition = pivotMotor.getPosition().getValueAsDouble() * 2 * Math.PI;
    double currentTime = Timer.getFPGATimestamp();

    if (Math.abs(sensorPosition - lastPivotPosition) < ArmConstants.PIVOT_REST_AMBIGUITY)
      pivotRestTime = currentTime;
    else
      pivotRestTime = -1;

    if ((currentTime - pivotRestTime > ArmConstants.REST_TIME && pivotRestTime != -1) 
        || Math.abs(sensorPosition - rotorPosition) > ArmConstants.MAX_PIVOT_SENSOR_DEVIATION
        && sensorPosition < Math.toRadians(80) && sensorPosition > Math.toRadians(2))
      pivotMotor.setPosition(sensorPosition);

    lastPivotPosition = sensorPosition;
  }

//#region Auto-Aim
  public static void faceSpeaker(Pose2d robotPose) {
    Pose3d target;
    if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue){
      target = Constants.blueSpeakerPose;
    }else{
      target = Constants.redSpeakerPose;
    }

    Pose3d relativeTarget = new Pose3d(target.getX() - robotPose.getX(), target.getY() - robotPose.getY(), target.getZ(), new Rotation3d());
    Rotation2d newtonApproximation = newtonApproximation(relativeTarget);
    aimAngleEntry.setDouble(newtonApproximation.getDegrees());
    holdPosition = globalToLocalAngle(newtonApproximation);//setPivotAngle(newtonApproximation, true);
  }

  private static Rotation2d newtonApproximation(Pose3d relativeTarget) {
    //All measures are in radians for this function
    double last = Math.PI / 4; //Initial guess

    for (int i = 0; i < Constants.ArmConstants.PIVOT_APPROXIMATION_PRECISION; i++) {
      //Compute variables that appear more than once
      double lsinTheta = Constants.ArmConstants.PIVOT_LENGTH * Math.sin(last);
      double lcosTheta = Constants.ArmConstants.PIVOT_LENGTH * Math.cos(last);
      double groundDistance = Math.sqrt(relativeTarget.getX() * relativeTarget.getX() + relativeTarget.getY() * relativeTarget.getY()) + lcosTheta;
      double totalHeight = relativeTarget.getZ() - lsinTheta - Constants.ArmConstants.PIVOT_HEIGHT + Math.max(0, (groundDistance - 1)/6);
        //4.9 * Math.pow((groundDistance / (Constants.ArmConstants.NOTE_LAUNCH_SPEED * Math.cos(last - Constants.ArmConstants.launcherAngleWithPivot.getRadians()))), 2);//
      double evaluation = evaluateAngle(last, totalHeight, groundDistance);
      double derivative = evaluateAngleDerivative(last, lsinTheta, lcosTheta, groundDistance, totalHeight);

      last = last - evaluation / derivative;
    }

    return Rotation2d.fromRadians(last);
  }

  private static double evaluateAngle(double theta, double totalHeight, double groundDistance) {
    return Constants.ArmConstants.LAUNCHER_ANGLE_WITH_PIVOT.getRadians() - theta - Math.atan2(totalHeight, groundDistance);
  }

  private static double evaluateAngleDerivative(double theta, double lsinTheta, double lcosTheta, double groundDistance, double totalHeight) {
    double numerator = groundDistance * lcosTheta - totalHeight * lsinTheta;
    double denominator = totalHeight * totalHeight + groundDistance * groundDistance;
    return numerator / denominator - 1;
  }
//#endregion

  public void setPivotNeutralMode(NeutralModeValue mode){
    pivotConfiguration.MotorOutput.NeutralMode = mode;
    pivotMotor.getConfigurator().apply(pivotConfiguration);
  }

  public void setClimberNeutralMode(NeutralModeValue mode){
    climberConfig.MotorOutput.NeutralMode = mode;
    climberMotor.getConfigurator().apply(climberConfig);
  }

  public void checkClimbStatus(){
    if(previousClimbMode != climbMode){
      if(climbMode){
        System.out.println("Climb Mode");
        setPivotNeutralMode(NeutralModeValue.Coast);
        setClimberNeutralMode(NeutralModeValue.Brake);
      }else{
        System.out.println("Normal Mode");
        setPivotNeutralMode(NeutralModeValue.Brake);
        setClimberNeutralMode(NeutralModeValue.Coast);
      }
    }
    previousClimbMode = climbMode;

  }


  public void setClimberOutput(double speed){
    climberMotor.set(speed);
  }

  public void checkHoldPositionDisabled(){
    if(DriverStation.isDisabled()){
      holdPosition = getPivotMotorPosition();
    }

  }
  
  public void telemUpdate(){
    angleEntry.setDouble(getPivotMotorPosition().getDegrees());
    sensorAngleEntry.setDouble(getPivotEncoderPosition().getDegrees());
    angleOffsetEntry.setDouble(pivotEncoderConfig.MagnetSensor.MagnetOffset);
    angleError.setDouble(getLocalAngle().getDegrees() - holdPosition.getDegrees());
    targetAngle.setDouble(holdPosition.getDegrees());
    climberModeEntry.setBoolean(climbMode);
  }
}
