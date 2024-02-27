// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Pivot;

public class PivotHold extends Command {
  /** Creates a new PivotHold. */
  Pivot pivot;
  Joystick joystick;
  boolean faceTargetPoint = false;
 
  public PivotHold(Pivot pivot, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivot = pivot;
    this.joystick = joystick;
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftInput = joystick.getRawAxis(2);
    double rightInput = joystick.getRawAxis(3);
    if(leftInput > 0.05){
      pivot.setClimberOutput(leftInput/5);
      Pivot.holdPosition = pivot.getLocalAngle();
      pivot.setPivotDutyCycle(0);
      System.out.println(leftInput);
      Pivot.climbMode = true;
    }else if(rightInput > 0.05){
      pivot.setClimberOutput(-rightInput / 5);
      Pivot.holdPosition = pivot.getLocalAngle();
      pivot.setPivotDutyCycle(0);
      System.out.println(rightInput);
      Pivot.climbMode = true;
    }else{
      if(Pivot.climbMode){
        pivot.setClimberOutput(0);
        pivot.setPivotDutyCycle(0);
      }else{
        pivot.setPivotAngle(Pivot.holdPosition, false);
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot.setPivotDutyCycle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
