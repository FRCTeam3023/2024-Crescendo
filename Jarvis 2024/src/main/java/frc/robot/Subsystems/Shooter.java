// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.Gains;

public class Shooter extends SubsystemBase {
  
  private final CANSparkMax leftShooterMotor = new CANSparkMax(11, MotorType.kBrushless);
  private SparkPIDController leftPID;
  private RelativeEncoder leftEncoder;


  private final CANSparkMax rightShooterMotor = new CANSparkMax(12, MotorType.kBrushless);  
  private SparkPIDController rightPID;
  private RelativeEncoder rightEncoder;

  private static final ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
  private static final GenericEntry speedEntry = armTab.add("RPM", 0).withPosition(2, 1).getEntry();


  private final Gains shooterGains = new Gains(0,0,1);  
  public Shooter() {
    
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


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    speedEntry.setDouble(leftEncoder.getVelocity());
  }


  /**
   * Closed loop control of the shooting wheels' RPM
   * @param rpm target rpm
   */
  public void setShooterRPM(double rpm){
    leftPID.setReference(rpm, ControlType.kVelocity);
    rightPID.setReference(rpm, ControlType.kVelocity);
  }


  public void setShooterDutyCycle(double speed){
    leftShooterMotor.set(speed);
    rightShooterMotor.set(speed); 
  }

}
