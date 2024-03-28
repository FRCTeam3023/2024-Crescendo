// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Subsystems.Drivetrain;

public class JoystickDrive extends Command {
  Drivetrain drivetrain;
  Joystick controller;
  Joystick rightJoystick;

  public static boolean fieldRelativeDrive = true;

  /** Creates a new JoystickDrive. */
  public JoystickDrive(Drivetrain drivetrain, Joystick controller, Joystick rightJoystick) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.rightJoystick = rightJoystick;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double topSpeed;

    //Toggle between seperate fast and slow drive modes with joystick button
    if(controller.getRawButton(9)){
      topSpeed = Constants.FAST_DRIVE_SPEED;
    }else{
      topSpeed = Constants.MAX_DRIVE_SPEED;
    }


    //--------------------------------------------------------------------------
    //translation input 
    double xInputRight = controller.getRawAxis(4);
    double yInputRight = controller.getRawAxis(5);

    double r = Math.sqrt((xInputRight*xInputRight) + (yInputRight * yInputRight));
    double theta = Math.atan2(yInputRight, xInputRight);


    double processedMagnitude = topSpeed * applyDeadband(r, Constants.DRIVE_TOLERANCE_PERCENT);

    double xSpeed = -Math.sin(theta) * processedMagnitude;
    double ySpeed = -Math.cos(theta) * processedMagnitude;
    //------------------------------------------------------------------------------------
    //rotation input -closed loop and open loop toggle
    double rotationSpeed = 0;

    //flip Direction if alliance is red, field-centric based on blue alliance
    if(Robot.alliance == Alliance.Red){
      xSpeed = -xSpeed;
      ySpeed = -ySpeed;
    }

      double xInputLeft = applyDeadband(controller.getRawAxis(0), Constants.DRIVE_TOLERANCE_PERCENT);
      rotationSpeed = -Math.signum(xInputLeft) * Math.pow(xInputLeft, 2) * Constants.MAX_ANGULAR_SPEED;
      drivetrain.drive(new ChassisSpeeds(xSpeed,ySpeed,rotationSpeed), fieldRelativeDrive);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(), false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double applyDeadband(double joystickValue, double tolerance){
    if(Math.abs(joystickValue) > tolerance){
      return Math.signum(joystickValue) * (Math.abs(joystickValue) - tolerance)/(1-tolerance);
    } 
    return 0;
  }
}
