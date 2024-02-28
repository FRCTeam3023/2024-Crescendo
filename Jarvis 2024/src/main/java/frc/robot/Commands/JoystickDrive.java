// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.List;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Util.PIDDisplay;
import frc.robot.Util.ProfiledWPILibSetter;

public class JoystickDrive extends Command {
  Drivetrain drivetrain;
  Joystick controller;
  Joystick rightJoystick;
  ProfiledPIDController turnController = new ProfiledPIDController(4, 0, 0, new Constraints(2, 4));

  /** Creates a new JoystickDrive. */
  public JoystickDrive(Drivetrain drivetrain, Joystick controller, Joystick rightJoystick) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.rightJoystick = rightJoystick;
    addRequirements(drivetrain);
    turnController.enableContinuousInput(-Math.PI, Math.PI);
    PIDDisplay.PIDList.addOption("Aim Turn PID", new ProfiledWPILibSetter(List.of(turnController)));
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

    if(rightJoystick.getRawButton(3)){
      Pose2d currentPose = drivetrain.getPose();
      Pose3d targetPose = new Pose3d();
      if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue){
        targetPose = Constants.blueSpeakerPose;
      }else{
        targetPose = Constants.redSpeakerPose;
      }
      Translation2d relativeTargetTranslation = currentPose.getTranslation().minus(targetPose.toPose2d().getTranslation());

      // //Account for velocity
      // double horizontalSpeed = Constants.ArmConstants.NOTE_LAUNCH_SPEED * Math.cos(Pivot.holdPosition.getRadians() - Constants.ArmConstants.launcherAngleWithPivot.getRadians());
      // double airTime = (Math.pow(relativeTargetTranslation.getY(),2) + Math.pow(relativeTargetTranslation.getX(),2)) / horizontalSpeed;
      // relativeTargetTranslation = new Translation2d(relativeTargetTranslation.getX() - drivetrain.getChassisSpeeds().vxMetersPerSecond * airTime,
      //   relativeTargetTranslation.getY() - drivetrain.getChassisSpeeds().vyMetersPerSecond * airTime); 

      Rotation2d targetRotation = Rotation2d.fromRadians(Math.atan2(relativeTargetTranslation.getY(), relativeTargetTranslation.getX()));
      rotationSpeed = turnController.calculate(currentPose.getRotation().getRadians(), targetRotation.getRadians());
    }else{
      double xInputLeft = applyDeadband(controller.getRawAxis(0), Constants.DRIVE_TOLERANCE_PERCENT);
      rotationSpeed = -Math.signum(xInputLeft) * Math.pow(xInputLeft, 2) * Constants.MAX_ANGULAR_SPEED;
    }


    drivetrain.drive(new ChassisSpeeds(xSpeed,ySpeed,rotationSpeed), true);

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
