// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.Pivot.PivotState;

public class AimPivot extends Command {
  /** Creates a new AimPivot. */
  Drivetrain drivetrain;

  public AimPivot() {
  }

  @Override
  public void initialize() {
    Pivot.climbMode = false;
  }

  @Override
  public void execute() {
    Pivot.setPivotState(PivotState.AUTOAIM);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
