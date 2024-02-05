// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.List;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Util.Gains;
import frc.robot.Util.PIDDisplay;
import frc.robot.Util.TalonFXsetter;

public class Arm extends SubsystemBase {
  private TalonFX pivotMotor = new TalonFX(0);
  private Gains pivotGains = new Gains(0, 0, 0, 0, 0);

  private static final ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
  GenericEntry armPositionEntry;

  public Arm() {
    TalonFXConfiguration pivotConfiguration = new TalonFXConfiguration();
    pivotConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    pivotConfiguration.Slot0.kP = pivotGains.P;
    pivotConfiguration.Slot0.kD = pivotGains.D;
    pivotConfiguration.Slot0.kS = pivotGains.S;
    pivotConfiguration.Slot0.kV = pivotGains.V;

    pivotConfiguration.MotionMagic.MotionMagicCruiseVelocity = 10;
    pivotConfiguration.MotionMagic.MotionMagicAcceleration = 40;
    pivotConfiguration.MotionMagic.MotionMagicJerk = 400;
    pivotConfiguration.Feedback.SensorToMechanismRatio = ArmConstants.PIVOT_GEARING;
    pivotConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    pivotMotor.getConfigurator().apply(pivotConfiguration);

    PIDDisplay.PIDList.addOption("Arm Pivot", new TalonFXsetter(List.of(pivotMotor.getConfigurator()), pivotConfiguration));

    armPositionEntry = armTab.add("Arm Position", 0).withPosition(0, 0).getEntry();
  }

  @Override
  public void periodic() {
    armPositionEntry.setDouble(pivotMotor.getPosition().getValue());
  }

  public void setPivotPosition(double position) {
    pivotMotor.setControl(new MotionMagicExpoVoltage(position));
  }
}
