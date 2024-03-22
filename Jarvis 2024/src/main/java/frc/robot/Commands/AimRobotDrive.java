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
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.Pivot.PivotState;
import frc.robot.Util.AutoAimCalculator;

public class AimRobotDrive extends Command {
  /** Creates a new AimRobot. */

  Drivetrain drivetrain;
  Joystick controller;
  
  public AimRobotDrive(Drivetrain drivetrain, Joystick controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.controller = controller;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pivot.climbMode = false;
    drivetrain.resetTurnController();
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
    Alliance currentColor = null;
    if (DriverStation.getAlliance().isPresent())
      currentColor = DriverStation.getAlliance().get();

    //flip Direction if alliance is red, field-centric based on blue alliance
    if(currentColor == Alliance.Red){
      xSpeed = -xSpeed;
      ySpeed = -ySpeed;
    }

    Pivot.setPivotState(PivotState.AUTOAIM);
    drivetrain.driveFacingTarget(new ChassisSpeeds(xSpeed, ySpeed, 0), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Pivot.setPivotState(PivotState.HOLD);
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
