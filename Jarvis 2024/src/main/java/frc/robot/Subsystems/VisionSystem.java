// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerJNI.TelemetryKind;

// import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonConstants;

public class VisionSystem extends SubsystemBase {
  /** Creates a new PhotonCamera. */

  private static final PhotonCamera photonCamera = new PhotonCamera("VisionCamera");
  //private static final PhotonCamera intakeCamera = new PhotonCamera("IntakeView");

  private static final ShuffleboardTab PhotonTab = Shuffleboard.getTab("PhotonVision");
  private final Field2d field = new Field2d();
  
  private static final ShuffleboardTab telemTab = Shuffleboard.getTab("Telemetry");
  private static final GenericEntry visionPoseEntry = PhotonTab.add("Vision Pose", new Pose2d().toString()).withPosition(0, 0).getEntry();
  private static final GenericEntry enabledEntry = telemTab.add("Vision System", false).withPosition(1, 2).getEntry();
  private double previousPipelineTimestamp = 0;
  public static boolean disabled = false;
  
  private static boolean intakeCameraInitialized = false;

  public VisionSystem() {
    if (!intakeCameraInitialized)
      try {telemTab.addCamera("Intake View", "IntakeView", "mjpg:http://10.30.23.11:1184/stream.mjpg");}
      catch(Exception e) {System.out.println("Failed to initialize intake camera stream: " + e.getMessage());}
    intakeCameraInitialized = true;
    // telemTab.add(CameraServer.getVideo("IntakeView").getSource());
    PhotonTab.add(field);

  }

  @Override
  public void periodic() {
    enabledEntry.setBoolean(!disabled);
    if (disabled) return;

    var pipelineResult = photonCamera.getLatestResult();
    var resultTimestamp = pipelineResult.getTimestampSeconds();

    if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
      previousPipelineTimestamp = resultTimestamp;
      var target = pipelineResult.getBestTarget();

      if (target.getPoseAmbiguity() <= .1) {
        Transform3d camToTarget = target.getBestCameraToTarget();
        Transform3d targetToCamera = camToTarget.inverse();

        Pose3d targetPose = getSelectedTargetPose(target.getFiducialId());
        Pose3d camPose = targetPose.transformBy(targetToCamera);

        Pose2d visionMeasurement = camPose.transformBy(PhotonConstants.CAMERA_TO_ROBOT).toPose2d();

        Drivetrain.addVisionMeasurement(visionMeasurement, resultTimestamp);
        visionPoseEntry.setString(visionMeasurement.toString());
        field.setRobotPose(visionMeasurement);
      }
    }

  }


  public Pose3d getSelectedTargetPose(int ID){
    Pose3d targetPose;

    switch (ID) {
      case 1:
        targetPose = PhotonConstants.TARGET_1_POSE;
        break;
      case 2:
        targetPose = PhotonConstants.TARGET_2_POSE;
        break;
      case 3:
        targetPose = PhotonConstants.TARGET_3_POSE;
        break;
      case 4:
        targetPose = PhotonConstants.TARGET_4_POSE;
        break;
      case 5:
        targetPose = PhotonConstants.TARGET_5_POSE;
        break;
      case 6:
        targetPose = PhotonConstants.TARGET_6_POSE;
        break;
      case 7:
        targetPose = PhotonConstants.TARGET_7_POSE;
        break;
      case 8:
        targetPose = PhotonConstants.TARGET_8_POSE;
        break;
      case 9:
        targetPose = PhotonConstants.TARGET_9_POSE;
        break;
      case 10:
        targetPose = PhotonConstants.TARGET_10_POSE;
        break;
      case 11:
        targetPose = PhotonConstants.TARGET_11_POSE;
        break;
      case 12:
        targetPose = PhotonConstants.TARGET_12_POSE;
        break;
      case 13:
        targetPose = PhotonConstants.TARGET_13_POSE;
        break;
      case 14:
        targetPose = PhotonConstants.TARGET_14_POSE;
        break;
      case 15:
        targetPose = PhotonConstants.TARGET_15_POSE;
        break;
      case 16:
        targetPose= PhotonConstants.TARGET_16_POSE;
        break;
  
    
      default:
        targetPose = new Pose3d();
        break;
    }

    return targetPose;
  }
}
