// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm;

public class ArmControl extends Command {
  /** Creates a new ArmControl. */
  Arm arm;
  Joystick armJoystick;
  public ArmControl(Arm arm, Joystick armJoystick) {
    this.arm = arm;
    this.armJoystick = armJoystick;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(armJoystick.getRawButton(6)){
      arm.setShooterDutyCycle(1);
    }else{
      arm.setShooterDutyCycle(0);
    }


    if(armJoystick.getRawButton(5)){
      arm.setIntakeSpeed(1);
    }else{
      arm.setIntakeSpeed(0);  
    }

    if(armJoystick.getPOV() == 180){
      arm.setIntakeSpeed(-.25);
      arm.setShooterDutyCycle(-.25);
    }

    if(armJoystick.getRawButton(1)){
      arm.setPivotDutyCycle(-.1);
    }else if(armJoystick.getRawButton(4)){
      arm.setPivotDutyCycle(.1);
    }else{
      arm.setPivotDutyCycle(0);
    }
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
