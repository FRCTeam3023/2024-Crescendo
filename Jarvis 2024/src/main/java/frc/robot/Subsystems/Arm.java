// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Util.Gains;

public class Arm extends SubsystemBase {

  private final CANSparkMax leftShooterMotor = new CANSparkMax(11, MotorType.kBrushless);
  private final CANSparkMax rightShooterMotor = new CANSparkMax(12, MotorType.kBrushless);  

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


  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
