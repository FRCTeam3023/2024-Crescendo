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
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivot = pivot;
    this.drivetrain = drivetrain;
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot.faceTarget(drivetrain.getPose());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
