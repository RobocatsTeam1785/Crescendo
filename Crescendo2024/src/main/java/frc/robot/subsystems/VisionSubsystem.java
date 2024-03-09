// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.*;
import frc.lib.Constants.VisionConstants;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Utils.Util1785;

public class VisionSubsystem extends SubsystemBase {

  private PhotonCamera frontCamera = new PhotonCamera("Arducam_OV9281_USB_Camera (1)");
  private double camHeight = Units.inchesToMeters(VisionConstants.FRONT_CAM_HEIGHT);
  private double camOffset = Units.inchesToMeters(VisionConstants.FRONT_CAM_OFFSET);
  private double camAngle = Units.degreesToRadians(30);
  private double tag8Height = VisionConstants.CENTER_SPEAKER_TAG_HEIGHT;
  private PhotonPipelineResult frontCameraPipeline;
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    PhotonCamera.setVersionCheckEnabled(false);
    PortForwarder.add(5800, "10.17.85.11", 5800);
    frontCamera.setPipelineIndex(0);
    frontCameraPipeline = frontCamera.getLatestResult();
  }

  public PhotonPipelineResult getProcessed() {
    return frontCameraPipeline;
  }

  public PhotonTrackedTarget getTag() {
    return frontCameraPipeline.getBestTarget();
  }

  public PhotonTrackedTarget getSpeakerTag() {
    if (hasTarget()) {
      for(PhotonTrackedTarget p : frontCameraPipeline.targets){
        if(p.getFiducialId()==VisionConstants.RED_SPEAKER_ID || p.getFiducialId()==VisionConstants.BLUE_SPEAKER_ID){
          return p;
        }
      }
    }
    return null;
  }

  public double getAprilTagDistance(){
    double distance;
    if (hasTarget()){
      if(getSpeakerTag()!=null){//0.7
      distance = PhotonUtils.calculateDistanceToTargetMeters(camHeight, tag8Height, camAngle, Units.degreesToRadians(getSpeakerTag().getPitch()));
    }
      else{
        distance=-1;
      }
      return distance;
    }
    return -1;
  }

  public double getYaw(){
    if(getSpeakerTag()!=null){//0.7
      return getSpeakerTag().getYaw();
    }
    else{
      return 0;
    }
  }

  public boolean hasSpeakerTarget(){
    if(hasTarget()){
      for(PhotonTrackedTarget p : frontCameraPipeline.targets){
        if(p.getFiducialId()==VisionConstants.RED_SPEAKER_ID || p.getFiducialId()==VisionConstants.BLUE_SPEAKER_ID){
          return true;
        }
      }
      return false;
    }
    else{
      return false;
    }
  }

  public boolean hasTarget() {
    return frontCameraPipeline.hasTargets();
  }

  @Override
  public void periodic() {
    frontCameraPipeline = frontCamera.getLatestResult();
    double latencySeconds = frontCameraPipeline.getLatencyMillis() / 1000.0;
    SmartDashboard.putNumber("latencySeconds", latencySeconds);
    SmartDashboard.putNumber("Distance", getAprilTagDistance());
    SmartDashboard.putNumber("Robot distance", Util1785.getDistanceRobotRelative(getYaw(), getAprilTagDistance(),camOffset));
    SmartDashboard.putNumber("Robot angle", Util1785.getRobotRelativeAngle(getYaw(), getAprilTagDistance(),camOffset));
    }
}
