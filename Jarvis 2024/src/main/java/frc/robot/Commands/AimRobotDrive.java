// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Pivot.PivotState;
import frc.robot.Util.AutoAimCalculator;
import frc.robot.Util.AutoAimCalculatorV2;

public class AimRobotDrive extends Command {
  /** Creates a new AimRobot. */

  Drivetrain drivetrain;
  Joystick controller;
  Joystick controller2;
  
  public AimRobotDrive(Drivetrain drivetrain, Joystick controller, Joystick controller2) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.controller2 = controller2;
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
    
    //flip Direction if alliance is red, field-centric based on blue alliance
    if(Robot.alliance == Alliance.Red){
      xSpeed = -xSpeed;
      ySpeed = -ySpeed;
    }

    if(controller2.getRawButton(7)){
      Pose2d targetPose = Robot.alliance == Alliance.Red ? Constants.redLandingZone : Constants.blueLandingZone;
      double deltaX = targetPose.getX() - drivetrain.getPose().getX();
      double deltaY = targetPose.getY() - drivetrain.getPose().getY();
      double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
      double angle =  Math.atan2(4 * ArmConstants.MAX_LOB_HEIGHT, distance) - Constants.ArmConstants.LAUNCHER_ANGLE_WITH_PIVOT.getRadians();
      double launchVelocity = Math.sqrt(9.8 * (2 * ArmConstants.MAX_LOB_HEIGHT + distance * distance / (8 * ArmConstants.MAX_LOB_HEIGHT)));

      Shooter.lobRPM = launchVelocity / ArmConstants.NOTE_LAUNCH_SPEED * ArmConstants.SHOOTER_RPM;
      PivotState.LOB.angle = Rotation2d.fromRadians(angle);
      Pivot.setPivotState(PivotState.LOB);

      drivetrain.driveFacingHeading(new ChassisSpeeds(xSpeed, ySpeed, 0), true, 
      Rotation2d.fromRadians(Math.atan2(deltaY, deltaX)).plus(Rotation2d.fromDegrees(180)));
    }else{
      Pivot.setPivotState(PivotState.AUTOAIM);
      // drivetrain.driveFacingTarget(new ChassisSpeeds(xSpeed, ySpeed, 0), true);
      drivetrain.driveFacingHeading(new ChassisSpeeds(xSpeed,ySpeed, 0), true, AutoAimCalculator.alpha);
    }
   
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
