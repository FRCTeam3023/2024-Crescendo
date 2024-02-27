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

  public Intake() {}

  @Override
  public void periodic() {

  }

  public void setIntakeSpeed(double speed) {
    //TODO noteloaded false when firing
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void intakeTillSensed(double speed) {
    if (noteSensor.get()) {
      noteLoaded = true;
      setIntakeSpeed(0);
    }
    else {
      setIntakeSpeed(speed);
    }
  }
}
