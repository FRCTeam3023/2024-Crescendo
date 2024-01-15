// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

// import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonConstants;

public class VisionSystem extends SubsystemBase {
  /** Creates a new PhotonCamera. */

  // private static final PhotonCamera photonCamera = new PhotonCamera("visionCamera");
  private double previousPipelineTimestamp = 0;

  public VisionSystem() {}

  @Override
  public void periodic() {
    // var pipelineResult = photonCamera.getLatestResult();
    // var resultTimestamp = pipelineResult.getTimestampSeconds();

    // if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
    //   previousPipelineTimestamp = resultTimestamp;
    //   var target = pipelineResult.getBestTarget();

    //   if (target.getPoseAmbiguity() <= .05) {
    //     Transform3d camToTarget = target.getBestCameraToTarget();
    //     Transform3d targetToCamera = camToTarget.inverse();

    //     Pose3d targetPose = getSelectedTargetPose(target.getFiducialId());
    //     Pose3d camPose = targetPose.transformBy(targetToCamera);

    //     Pose2d visionMeasurement = camPose.transformBy(PhotonConstants.CAMERA_TO_ROBOT).toPose2d();

        

    //     Drivetrain.addVisionMeasurement(visionMeasurement, resultTimestamp);


    //   }
    // }
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
    
      default:
        targetPose = new Pose3d();
        break;
    }

    return targetPose;
  }
}
