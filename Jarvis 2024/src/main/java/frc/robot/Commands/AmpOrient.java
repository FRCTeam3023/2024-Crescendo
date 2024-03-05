// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.List;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Util.PIDDisplay;
import frc.robot.Util.ProfiledWPILibSetter;

public class AmpOrient extends Command {
  /** Creates a new AmpOrient. */
  Drivetrain drivetrain;
  Pose2d targetPose;
  ProfiledPIDController xController = new ProfiledPIDController(4, 0, 0, new Constraints(2, 4));
  ProfiledPIDController yController = xController;
  ProfiledPIDController thetaController = new ProfiledPIDController(4, 0, 0, new Constraints(2, 4));

  public AmpOrient(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    PIDDisplay.PIDList.addOption("Amp Turn PID", new ProfiledWPILibSetter(List.of(thetaController)));
    PIDDisplay.PIDList.addOption("Amp Translation PID", new ProfiledWPILibSetter(List.of(xController,yController)));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPose = DriverStation.getAlliance().get() == Alliance.Blue ? Constants.blueAmpPose : Constants.redAmpPose;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();
    double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
    double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
    double thetaSpeed = thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    drivetrain.drive(new ChassisSpeeds(xSpeed,ySpeed,thetaSpeed), true);
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
