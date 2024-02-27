// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class IntakeShooterControl extends Command {
  /** Creates a new IntakeShooterControl. */
  Intake intake;
  Shooter shooter;
  Joystick joystick;

  public IntakeShooterControl(Intake intake, Shooter shooter, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.shooter = shooter;
    this.joystick = joystick;
    addRequirements(intake, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(joystick.getRawButton(5)){
      intake.setIntakeSpeed(1);
    }else{
      intake.setIntakeSpeed(0);  
    }

    if(joystick.getRawButton(9)){
      intake.setIntakeSpeed(-.25);
      shooter.setShooterDutyCycle(-.25);
    }else{
      shooter.setShooterDutyCycle(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakeSpeed(0);
    shooter.setShooterDutyCycle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
