// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Util.Gains;

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
  private final Gains pivotGains = new Gains(0, 0, 0, 0, 12);
  /** Creates a new Arm. */
  public Arm() {
    TalonFXConfiguration pivotConfiguration = new TalonFXConfiguration();

    pivotConfiguration.MotorOutput.Inverted = ArmConstants.pivotInverted;
    pivotConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivotConfiguration.Voltage.PeakForwardVoltage = pivotGains.peakOutput;
    pivotConfiguration.Voltage.PeakReverseVoltage = pivotGains.peakOutput;
    pivotConfiguration.Feedback.SensorToMechanismRatio = ArmConstants.pivotGearRatio;

    pivotConfiguration.MotionMagic.MotionMagicCruiseVelocity = 1;
    pivotConfiguration.MotionMagic.MotionMagicAcceleration = 10;
    pivotConfiguration.MotionMagic.MotionMagicJerk = 50;

    pivotConfiguration.Slot0.kP = pivotGains.P;
    pivotConfiguration.Slot0.kD = pivotGains.D;
    pivotConfiguration.Slot0.kS = pivotGains.S;
    pivotConfiguration.Slot0.kD = pivotGains.V;

    pivotMotor.getConfigurator().apply(pivotConfiguration);

    //-----------------------------------------------------------------------------------------


    leftPID = leftShooterMotor.getPIDController();
    leftEncoder = leftShooterMotor.getEncoder();

    rightPID = rightShooterMotor.getPIDController();
    rightEncoder = rightShooterMotor.getEncoder();

    leftPID.setP(shooterGains.P);
    leftPID.setFF(shooterGains.F);
    leftPID.setOutputRange(-shooterGains.peakOutput,shooterGains.peakOutput);

    rightPID = leftPID;

    leftShooterMotor.setInverted(true);
    rightShooterMotor.setInverted(false);

    leftEncoder.setVelocityConversionFactor(0.5);
    rightEncoder.setVelocityConversionFactor(0.5);


  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShooterRPM(double rpm){
    leftPID.setReference(rpm, ControlType.kVelocity);
    rightPID.setReference(rpm, ControlType.kVelocity);
  }

  public void setShooterDutyCycle(double speed){
    leftShooterMotor.set(speed);
    rightShooterMotor.set(speed); 
  }
}
