// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Pivot;

public class AimPivot extends Command {
  /** Creates a new AimPivot. */
  Drivetrain drivetrain;
  Pivot pivot;

  public AimPivot(Pivot pivot, Drivetrain drivetrain) {
    this.pivot = pivot;
    this.drivetrain = drivetrain;
    addRequirements(pivot);
  }

  @Override
  public void initialize() {
    Pivot.climbMode = false;
  }

  @Override
  public void execute() {
    pivot.faceSpeaker(drivetrain.getPose());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
