// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.schedulers.SequentialScheduler;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

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
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void intakeTillSensed(double speed) {
    if (!noteSensor.get()) {
      noteLoaded = true;
      run(() -> new SequentialCommandGroup(
        new InstantCommand(() -> setIntakeSpeed(-speed)),
        new WaitCommand(Constants.ArmConstants.NOTE_RETRACTION_TIME), 
        new InstantCommand(() -> setIntakeSpeed(0))
      ));
    }
    else
      setIntakeSpeed(speed);
  }
}