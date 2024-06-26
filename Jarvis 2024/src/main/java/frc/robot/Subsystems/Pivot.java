// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.List;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Util.AutoAimCalculator;
import frc.robot.Util.AutoAimCalculatorV2;
import frc.robot.Util.Gains;
import frc.robot.Util.PIDDisplay;
import frc.robot.Util.TalonFXSimModel;
import frc.robot.Util.TalonFXsetter;

public class Pivot extends SubsystemBase {
  private final TalonFX pivotMotor = new TalonFX(10);
  private final TalonFXSimState pivotSimState = pivotMotor.getSimState();
  private final TalonFXSimModel pivotSimModel;
  private TalonFXConfiguration pivotConfiguration;
  private final CANcoder pivotSensor = new CANcoder(9);
  private final CANcoderSimState pivotSensorSimState = pivotSensor.getSimState();
  private double lastPivotPosition = 0;
  private double pivotRestTime = -1;
  private final CANcoderConfiguration pivotEncoderConfig = new CANcoderConfiguration();
  private final Gains pivotGains = new Gains(25, 0, 0, 0, 10);

  private final TalonFX climberMotor = new TalonFX(14);
  private TalonFXConfiguration climberConfig;

  // public static Rotation2d targetPosition = new Rotation2d();
  public static boolean climbMode = false;
  public static boolean previousClimbMode = false;

  private static PivotState pivotState = PivotState.NOTHING;
  private static PivotState previousState = PivotState.NOTHING;
  private double count = 0;

  private static final ShuffleboardTab armTab = Shuffleboard.getTab("Arm");

  private static final GenericEntry angleEntry = armTab.add("Local Angle", 0).withPosition(0, 0).getEntry();
  private static final GenericEntry sensorAngleEntry = armTab.add("Sensor Angle", 0).withPosition(0, 1).getEntry();
  private static final GenericEntry angleOffsetEntry = armTab.add("Angle Offset", 0).withPosition(0, 3).getEntry();
  private static final GenericEntry targetAngle = armTab.add("Target Angle",0).withPosition(0, 2).getEntry();
  private static final GenericEntry angleError = armTab.add("Angle Error",0).withPosition(0, 3).getEntry();
  private static final GenericEntry approximationErrorEntry = armTab.add("Approximator Error", 0).withPosition(1, 3).getEntry();
  private static final GenericEntry aimAngleEntry = armTab.add("Aim Angle",0).withPosition(3, 1).getEntry();
  private static final GenericEntry climberModeEntry = armTab.add("Climb Mode",false).withPosition(3, 3).getEntry();
  private static final GenericEntry noteLoadedEntry = armTab.add("Note Loaded", true).getEntry();
  private static final ShuffleboardTab simTab = Shuffleboard.getTab("Simulation");
  private static final GenericEntry voltageInput = simTab.add("Input", 0).getEntry();
  private static final GenericEntry pivotStateEntry = armTab.add("Pivot State", "Placeholder").getEntry();

  public Pivot() {


    //basic configuration for the pivot motor
    pivotConfiguration = new TalonFXConfiguration();

    pivotConfiguration.MotorOutput.Inverted = ArmConstants.PIVOT_INVERTED;
    pivotConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivotConfiguration.Voltage.PeakForwardVoltage = pivotGains.peakOutput;
    pivotConfiguration.Voltage.PeakReverseVoltage = pivotGains.peakOutput;

    pivotConfiguration.MotionMagic.MotionMagicCruiseVelocity = 7;
    pivotConfiguration.MotionMagic.MotionMagicAcceleration = 15;
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

    // targetPosition = getLocalAngle();


    climberConfig = new TalonFXConfiguration();
    climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    climberMotor.getConfigurator().apply(climberConfig);

    pivotSimState.setSupplyVoltage(Constants.SIMULATION_SUPPLY_VOLTAGE);
    pivotSimModel = new TalonFXSimModel(pivotConfiguration.MotionMagic.MotionMagicCruiseVelocity, pivotConfiguration.MotionMagic.MotionMagicAcceleration);

    pivotMotor.setPosition(getPivotEncoderPosition().getRadians());
    PivotState.HOLD.angle = getLocalAngle();
  }

  @Override
  public void periodic() {
    telemUpdate();
    checkClimbStatus();
    //if (!Constants.ArmConstants.USE_REMOTE_PIVOT_SENSOR) checkRotorEncoder();
  }

  @Override
  public void simulationPeriodic(){
    pivotSimModel.setMotorVoltage(pivotSimState.getMotorVoltage());
    pivotSimModel.update(.02);
    pivotSimState.setRawRotorPosition(pivotSimModel.getPosition());
    pivotSimState.setRotorVelocity(pivotSimModel.getVelocity());

    voltageInput.setDouble(pivotSimState.getMotorVoltage());

    pivotSensorSimState.setRawPosition(pivotSimModel.getPosition() / ArmConstants.PIVOT_GEAR_RATIO);

  }


  public static void setPivotState(PivotState state){
    pivotState = state;
  }

  public static PivotState getPivotState(){
    return pivotState;
  }

  public void approachCurrentState(){
    if(pivotState == PivotState.AUTOAIM){
      setPivotAngle(AutoAimCalculator.theta, true);
    } else if(pivotState == PivotState.LOB){
      setPivotAngle(pivotState.angle, true);
    } else if(pivotState == PivotState.HOLD){
      if(pivotState != previousState){
        pivotState.angle = Rotation2d.fromRadians(getLocalAngle().getRadians() + getPivotMotorVelocity() / pivotConfiguration.MotionMagic.MotionMagicAcceleration);
      }
      setPivotAngle(pivotState.angle, false);
    }else if(pivotState == PivotState.NOTHING){
      setPivotDutyCycle(0);
    } else {
      setPivotAngle(pivotState.angle, false);
    }
    previousState = pivotState;
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

  public double getPivotMotorVelocity(){
    return pivotMotor.getVelocity().getValue();
  }

  /**
   * Set the closed loop motion magic control target of the pivot joint, adjustable between global and local control.
   * @param angle target angle to move to
   * @param isGlobal if angle being passed in is global or local
   */
  public void setPivotAngle(Rotation2d angle, boolean isGlobal) {
    climbMode = false;
    if (isGlobal) angle = globalToLocalAngle(angle);
    angle = Rotation2d.fromRadians(Math.min(Math.max(angle.getRadians(), 0), Constants.ArmConstants.PIVOT_MAX.getRadians()));
    pivotMotor.setControl(new MotionMagicVoltage(angle.getRadians()));
  }


  /**
   * Check if pivot motor is at holdPosition
   * @return True if the difference is small enough
   */
  public boolean isAtTargetAngle() {
    return Math.abs(pivotState.angle.getRadians() - pivotMotor.getPosition().getValueAsDouble()) < Constants.ArmConstants.MAX_PIVOT_DEVIATION;
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
  }

  public void setPivotVoltage(double voltage){
    pivotMotor.setControl(new VoltageOut(voltage));
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
  
  public void telemUpdate(){
    if(++count > 50){
      count = 0;
      angleEntry.setDouble(getPivotMotorPosition().getDegrees());
      sensorAngleEntry.setDouble(getPivotEncoderPosition().getDegrees());
      angleOffsetEntry.setDouble(pivotEncoderConfig.MagnetSensor.MagnetOffset);
      angleError.setDouble(getLocalAngle().getDegrees() - pivotState.angle.getDegrees());
      targetAngle.setDouble(pivotState.angle.getDegrees());
      //approximationErrorEntry.setDouble(AutoAimCalculator.error);
      climberModeEntry.setBoolean(climbMode);
      aimAngleEntry.setDouble(AutoAimCalculator.theta.getDegrees());
      noteLoadedEntry.setBoolean(Intake.noteLoaded);
      pivotStateEntry.setString(pivotState.name());
    }
  }

  public enum PivotState{
    PICKUP(ArmConstants.PICKUP_POSITION),
    SPEAKER(ArmConstants.SPEAKER_POSITION),
    AMP(ArmConstants.AMP_POSITION),
    HOLD(new Rotation2d()),
    NOTHING(new Rotation2d()),
    LOB(new Rotation2d()),
    AUTOAIM(new Rotation2d());

    public Rotation2d angle;
    private PivotState(Rotation2d angle){
      this.angle = angle;
    }
  }
}
