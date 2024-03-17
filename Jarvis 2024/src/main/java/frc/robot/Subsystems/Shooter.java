// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.Gains;

public class Shooter extends SubsystemBase {
  
  private final CANSparkMax leftShooterMotor = new CANSparkMax(11, MotorType.kBrushless);
  private SparkPIDController leftPID;
  private RelativeEncoder leftEncoder;


  private final CANSparkMax rightShooterMotor = new CANSparkMax(12, MotorType.kBrushless);  
  private SparkPIDController rightPID;
  private RelativeEncoder rightEncoder;

  private static final ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
  private static final GenericEntry rightSpeedEntry = armTab.add("Right RPM", 0).withPosition(1, 2).getEntry();
  private static final GenericEntry leftSpeedEntry = armTab.add("Left RPM", 0).withPosition(2, 2).getEntry();


  private final Gains shooterGains = new Gains(0,1.0/Constants.ArmConstants.SHOOTER_RPM,1);  
  private double targetRPM = Constants.ArmConstants.SHOOTER_RPM;

  public Shooter() {
    
    leftPID = leftShooterMotor.getPIDController();
    leftEncoder = leftShooterMotor.getEncoder();

    rightPID = rightShooterMotor.getPIDController();
    rightEncoder = rightShooterMotor.getEncoder();

    leftPID.setP(shooterGains.P);
    leftPID.setFF(shooterGains.F);
    leftPID.setOutputRange(-shooterGains.peakOutput,shooterGains.peakOutput);

    rightPID.setP(shooterGains.P);
    rightPID.setFF(shooterGains.F);
    rightPID.setOutputRange(-shooterGains.peakOutput,shooterGains.peakOutput);

    leftShooterMotor.setInverted(true);
    rightShooterMotor.setInverted(false);

    leftEncoder.setVelocityConversionFactor(0.5);
    rightEncoder.setVelocityConversionFactor(0.5);

    leftShooterMotor.enableVoltageCompensation(11.5);
    rightShooterMotor.enableVoltageCompensation(11.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftSpeedEntry.setDouble(leftEncoder.getVelocity());
    rightSpeedEntry.setDouble(rightEncoder.getVelocity());
  }


  /**
   * Closed loop control of the shooting wheels' RPM
   * @param rpm target rpm
   */
  public void setShooterRPM(double rpm){
    leftPID.setReference(rpm, ControlType.kVelocity);
    rightPID.setReference(rpm, ControlType.kVelocity);
    targetRPM = rpm;
  }


  /**
   * @return True if both flywheels have reached the target RPM
   */
  public boolean isFlywheelReady() {
    double leftError = Math.abs(leftShooterMotor.getEncoder().getVelocity() - targetRPM);
    double rightError = Math.abs(rightShooterMotor.getEncoder().getVelocity() - targetRPM);

    return leftError < Constants.ArmConstants.MAX_SHOOTER_RPM_ERROR && rightError < Constants.ArmConstants.MAX_SHOOTER_RPM_ERROR;
  }

  public void setShooterDutyCycle(double speed){
    leftShooterMotor.set(speed);
    rightShooterMotor.set(speed); 
  }

  public void setShooterVoltage(double voltage){
    leftShooterMotor.setVoltage(voltage);
    rightShooterMotor.setVoltage(voltage);
  }
}
