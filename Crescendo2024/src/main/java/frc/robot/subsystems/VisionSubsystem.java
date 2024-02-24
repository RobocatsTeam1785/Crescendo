// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.*;
import frc.lib.Constants.VisionConstants;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  private PhotonCamera frontCamera = new PhotonCamera("Arducam_OV9281_USB_Camera (1)");
  private CvSink frontCam;
  private CvSource cvSource;
  private MjpegServer server1;
  private PhotonPipelineResult frontCameraPipeline;
  private Transform3d robotToCam;
  private PhotonPoseEstimator photonPoseEstimator;
  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    PortForwarder.add(5800, "10.17.85.11", 5800);
    frontCamera.setPipelineIndex(0);
    robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
        frontCamera, robotToCam);
    frontCameraPipeline = frontCamera.getLatestResult();
  }

  public PhotonPipelineResult getProcessed() {
    return frontCameraPipeline;
  }

  public PhotonTrackedTarget getTag() {
    return frontCameraPipeline.getBestTarget();
  }

  public double getAprilTagRot() {
    if (hasTarget()) {
      return frontCameraPipeline.getBestTarget().getYaw();
    }
    return -1;

  }

  public double getAprilTagPitch() {
    if (hasTarget()) {
      return frontCameraPipeline.getBestTarget().getPitch();
    }
    return -1;

  }

  public boolean hasTarget() {
    return frontCameraPipeline.hasTargets();
  }

  @Override
  public void periodic() {
    frontCameraPipeline = frontCamera.getLatestResult();
    photonPoseEstimator.update();
    double latencySeconds = frontCameraPipeline.getLatencyMillis() / 1000.0;
    SmartDashboard.putNumber("latencySeconds", latencySeconds);

    // This method will be called once per scheduler run
  }
}
