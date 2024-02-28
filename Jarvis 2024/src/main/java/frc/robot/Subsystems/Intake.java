// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final TalonSRX intakeMotor = new TalonSRX(13);
  private final DigitalInput noteSensor = new DigitalInput(4);

  public static boolean noteLoaded;
  //int iterations = 0;

  public Intake() {}

  @Override
  public void periodic() {
    // iterations++;
    // if (iterations > 30) {
    //   System.out.println("Note sensor data: " + noteSensor.get());
    //   iterations = 0;
    // }
    //noteLoaded = noteSensor.get();
  }

  public void setIntakeSpeed(double speed) {
    System.out.println(speed);
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void intakeTillSensed(double speed) {
    if (senseNote()) {
      noteLoaded = true;
      setIntakeSpeed(0);
    }
    else
      setIntakeSpeed(speed);
      noteLoaded = false;
  }

  public boolean senseNote() {
    return !noteSensor.get();
  }
}
