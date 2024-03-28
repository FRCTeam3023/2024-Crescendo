// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.List;

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.Pivot.PivotState;
import frc.robot.Util.PIDDisplay;
import frc.robot.Util.ProfiledWPILibSetter;

public class AmpOrient extends Command {
  /** Creates a new AmpOrient. */
  Drivetrain drivetrain;
  Pose2d targetPose;
  public static boolean atTarget = false;
  ProfiledPIDController xController = new ProfiledPIDController(4, 0, 0, new Constraints(1.5, 2));
  ProfiledPIDController yController = new ProfiledPIDController(4, 0, 0, new Constraints(1.5, 2));
  ProfiledPIDController thetaController = new ProfiledPIDController(6, 0, 0, new Constraints(4, 8));

  ShuffleboardTab telemTab = Shuffleboard.getTab("Telemetry");
  GenericEntry xOutputEntry = telemTab.add("Amp Orient X output",0).getEntry();
  GenericEntry yOutputEntry = telemTab.add("Amp Orient Y output",0).getEntry();

  public AmpOrient(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    PIDDisplay.PIDList.addOption("Amp Turn PID", new ProfiledWPILibSetter(List.of(thetaController)));
    PIDDisplay.PIDList.addOption("Amp Translation PID", new ProfiledWPILibSetter(List.of(xController,yController)));

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    atTarget = false;
    Pose2d currentPose = drivetrain.getPose();
    targetPose = Robot.alliance == Alliance.Blue ? Constants.blueAmpPose : Constants.redAmpPose;
    xController.reset(currentPose.getX());
    yController.reset(currentPose.getY());
    thetaController.reset(currentPose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();
    double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
    double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
    double thetaSpeed = thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    drivetrain.drive(new ChassisSpeeds(xSpeed,ySpeed,thetaSpeed), true);

    xOutputEntry.setDouble(xSpeed);
    yOutputEntry.setDouble(ySpeed);

    double xError = Math.abs(currentPose.getX() - targetPose.getX());
    double yError = Math.abs(currentPose.getY() - targetPose.getY());
    double totalError = Math.sqrt(xError * xError + yError * yError);
    if(totalError < 1){
      atTarget = true;
    } 
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

}
