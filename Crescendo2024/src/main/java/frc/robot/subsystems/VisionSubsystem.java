// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.*;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  private PhotonCamera frontCamera = new PhotonCamera("photonvision");
  private PhotonPipelineResult frontCameraPipeline;
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    frontCameraPipeline = frontCamera.getLatestResult();
  }

  public boolean getTarget(){
    return frontCameraPipeline.hasTargets();
  }
  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
