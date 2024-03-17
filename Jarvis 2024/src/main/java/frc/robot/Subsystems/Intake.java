// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final TalonSRX intakeMotor = new TalonSRX(13);
  private final DigitalInput noteSensor = new DigitalInput(4);

  private static final ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
  private static final GenericEntry noteSensorEntry = armTab.add("Note", false).withPosition(2, 4).getEntry();

  public static boolean noteSensed;
  //int iterations = 0;

  public Intake() {}

  @Override
  public void periodic() {
    noteSensed = senseNote();
    noteSensorEntry.setBoolean(noteSensed);
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void intakeTillSensed(double speed) {
    if (senseNote()) {
      setIntakeSpeed(0);
    }
    else
      setIntakeSpeed(speed);
  }

  public void outakeTillSensed(double speed) {
    if (!senseNote()) {
      setIntakeSpeed(0);
    }
    else
      setIntakeSpeed(speed);
  }

  public boolean senseNote() {
    return !noteSensor.get();
  }
}
