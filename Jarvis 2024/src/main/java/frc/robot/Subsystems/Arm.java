// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.util.RootNameLookup;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Util.Gains;
import frc.robot.Util.PIDDisplay;
import frc.robot.Util.TalonFXsetter;

public class Arm extends SubsystemBase {

  private final CANSparkMax leftShooterMotor = new CANSparkMax(11, MotorType.kBrushless);
  private SparkPIDController leftPID;
  private RelativeEncoder leftEncoder;


  private final CANSparkMax rightShooterMotor = new CANSparkMax(12, MotorType.kBrushless);  
  private SparkPIDController rightPID;
  private RelativeEncoder rightEncoder;


  private final Gains shooterGains = new Gains(0,0,1);

  private final TalonSRX intakeMotor = new TalonSRX(13);

  private final TalonFX pivotMotor = new TalonFX(10);
  private final CANcoder pivotSensor = new CANcoder(9);
  private final Gains pivotGains = new Gains(0, 0, 0, 0, 12);
  
  /** Creates a new Arm. */
  public Arm() {

    //basic configuration for the pivot motor
    TalonFXConfiguration pivotConfiguration = new TalonFXConfiguration();

    pivotConfiguration.MotorOutput.Inverted = ArmConstants.pivotInverted;
    pivotConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivotConfiguration.Voltage.PeakForwardVoltage = pivotGains.peakOutput;
    pivotConfiguration.Voltage.PeakReverseVoltage = pivotGains.peakOutput;

    pivotConfiguration.MotionMagic.MotionMagicCruiseVelocity = 1;
    pivotConfiguration.MotionMagic.MotionMagicAcceleration = 10;
    pivotConfiguration.MotionMagic.MotionMagicJerk = 50;

    pivotConfiguration.Slot0.kP = pivotGains.P;
    pivotConfiguration.Slot0.kD = pivotGains.D;
    pivotConfiguration.Slot0.kS = pivotGains.S;
    pivotConfiguration.Slot0.kD = pivotGains.V;

    //set feedback sensor as a remote CANcoder, resets the rotor sensor position every time it publishes values
    pivotConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    //Im not sure which one I should use, if it is using the remote sensor to adjust the rotor or if 
    // pivotConfiguration.Feedback.SensorToMechanismRatio = ArmConstants.pivotGearRatio / (Math.PI * 2);
    pivotConfiguration.Feedback.SensorToMechanismRatio = 1 / (Math.PI * 2);
    pivotConfiguration.Feedback.FeedbackRemoteSensorID = pivotSensor.getDeviceID();

    pivotMotor.getConfigurator().apply(pivotConfiguration);

    //---------------------------------------------------------------------------------------------------

    CANcoderConfiguration sensorConfig = new CANcoderConfiguration();
    sensorConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    sensorConfig.MagnetSensor.MagnetOffset = -ArmConstants.PIVOT_SENSOR_OFFSET.getRadians() / (Math.PI * 2);
    pivotSensor.getConfigurator().apply(sensorConfig);

    //-----------------------------------------------------------------------------------------


    leftPID = leftShooterMotor.getPIDController();
    leftEncoder = leftShooterMotor.getEncoder();

    rightPID = rightShooterMotor.getPIDController();
    rightEncoder = rightShooterMotor.getEncoder();

    leftPID.setP(shooterGains.P);
    leftPID.setFF(shooterGains.F);
    leftPID.setOutputRange(-shooterGains.peakOutput,shooterGains.peakOutput);

    rightPID = leftPID;

    leftShooterMotor.setInverted(false);
    rightShooterMotor.setInverted(true);

    leftEncoder.setVelocityConversionFactor(0.5);
    rightEncoder.setVelocityConversionFactor(0.5);

    leftShooterMotor.enableVoltageCompensation(12);
    rightShooterMotor.enableVoltageCompensation(12);



    PIDDisplay.PIDList.addOption("Pivot", new TalonFXsetter(List.of(pivotMotor.getConfigurator()), pivotConfiguration));
    
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // faceTarget(new Pose3d(7, -5, 10, new Rotation3d()), new Pose2d(0, 0, Rotation2d.fromRadians(-0.95)));
    System.out.println(getLocalAngle());
  }

  /**
   * Closed loop control of the shooting wheels' RPM
   * @param rpm target rpm
   */
  public void setShooterRPM(double rpm){
    leftPID.setReference(rpm, ControlType.kVelocity);
    rightPID.setReference(rpm, ControlType.kVelocity);
  }


  public void setIntakeSpeed(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setShooterDutyCycle(double speed){
    leftShooterMotor.set(speed);
    rightShooterMotor.set(speed); 
  }

  //Convert world angle with ground to local angle with pivot's starting position
  public Rotation2d localizeAngle(Rotation2d angle) {
    return Rotation2d.fromRadians(angle.getRadians() + Constants.ArmConstants.pivotInitializePosition.getRadians());
  }
  //Convert local angle with pivot's starting position to world angle with ground 
  public Rotation2d globalizeAngle(Rotation2d angle) {
    return Rotation2d.fromRadians(angle.getRadians() - Constants.ArmConstants.pivotInitializePosition.getRadians());
  }

  /**
   * Angle of the arm positive from starting configuration
   * @return angle of arm
   */
  public Rotation2d getLocalAngle(){
    return new Rotation2d(pivotMotor.getPosition().getValue());
  }

  /**
   * The angle of the arm positive from horizontal based on the field
   * @return angle of arm
   */
  public Rotation2d getGlobalAngle(){
    return globalizeAngle(getLocalAngle());
  }

  /**
   * Set the closed loop motion magic control target of the pivot joint, adjustable between global and local control.
   * @param angle target angle to move to
   * @param isGlobal if angle being passed in is global or local
   */
  public void setPivotAngle(Rotation2d angle, boolean isGlobal) {
    Rotation2d globalAngle;
    if(isGlobal){
      globalAngle = angle;
      angle = localizeAngle(angle);
    }else{
      globalAngle = globalizeAngle(angle);
    }


    pivotMotor.setControl(new MotionMagicVoltage(angle.getRadians())
    .withFeedForward(
      Constants.ArmConstants.pivotFeedForward * Math.cos(globalAngle.getRadians())
      ));
  }
  
  public void setPivotDutyCycle(double speed){
    pivotMotor.setControl(new DutyCycleOut(speed));
  }

  //Face the shooter output towards the target point
  public void faceTarget(Pose3d target, Pose2d robotPose) {
    // double xDistance = target.getX() - robotPose.getX();
    // double yDistance = target.getY() - robotPose.getY();
    // double groundDistance = Math.sqrt(xDistance*xDistance + yDistance*yDistance);
    // double targetAngle = Constants.ArmConstants.launcherAngleWithPivot.getDegrees() - Math.atan(target.getZ() / groundDistance);
    Pose3d relativeTarget = new Pose3d(target.getX() - robotPose.getX(), target.getY() - robotPose.getY(), target.getZ(), new Rotation3d());
    Rotation2d newtonApproximation = newtonApproximation(relativeTarget, robotPose.getRotation());
    System.out.println(newtonApproximation.getRadians());
    setPivotAngle(newtonApproximation, true);
  }

  //Solve for the correct angle using an approximation 
  //Newton's method is fast, but may not always converge; we might have to use a less efficient method if issues arise
  private Rotation2d newtonApproximation(Pose3d relativeTarget, Rotation2d heading) {
    Rotation2d last = Rotation2d.fromDegrees(180); //Initial guess
    for (int i = 0; i < Constants.ArmConstants.pivotApproximationPrecision; i++) {
      Rotation2d evaluation = evaluateAngle(relativeTarget, heading, last);
      Rotation2d derivative = evaluateAngleDerivative(relativeTarget, heading, last);
      last = last.minus(Rotation2d.fromRadians(evaluation.getRadians() / derivative.getRadians()));
    }
    return last;
  }

  private Rotation2d evaluateAngle(Pose3d relativeTarget, Rotation2d heading, Rotation2d theta) {
    //Precompute repeated variables
    double lcosTheta = Constants.ArmConstants.pivotLength * theta.getCos();

    double xDistance = relativeTarget.getX() - lcosTheta * heading.getSin();
    double yDistance = relativeTarget.getY() - lcosTheta * heading.getCos();
    double zDistance = relativeTarget.getZ() - Constants.ArmConstants.pivotLength * theta.getSin() - Constants.ArmConstants.pivotHeight;
    double groundDistance = Math.sqrt(xDistance * xDistance + yDistance * yDistance);

    return Rotation2d.fromRadians(Constants.ArmConstants.launcherAngleWithPivot.getRadians() - theta.getRadians() - Math.atan2(zDistance, groundDistance));
  }

  private Rotation2d evaluateAngleDerivative(Pose3d relativeTarget, Rotation2d heading, Rotation2d theta) {
    //Precompute repeated variables
    double lcosTheta = Constants.ArmConstants.pivotLength * theta.getCos();
    double lsinTheta = Constants.ArmConstants.pivotLength * theta.getSin();
    double cosHeading = heading.getCos();
    double sinHeading = heading.getSin();
    double zDistance = relativeTarget.getZ() - lsinTheta - Constants.ArmConstants.pivotHeight;
    double groundDistanceSquared = Math.pow(relativeTarget.getX() - lcosTheta * sinHeading,2) + Math.pow(relativeTarget.getY() - lcosTheta * cosHeading,2);

    double numerator = lcosTheta * groundDistanceSquared + lsinTheta * zDistance * (relativeTarget.getX() * sinHeading + relativeTarget.getY() * cosHeading - lcosTheta);
    double denominator = (zDistance * zDistance + groundDistanceSquared) * Math.sqrt(groundDistanceSquared);

    return Rotation2d.fromRadians(numerator / denominator - 1);
  }
}
