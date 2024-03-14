// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.*;

public class CameraSubsystem extends SubsystemBase {
  public static UsbCamera camera1;
  public static CvSink cvSink;
  public static CvSource cvSource;
  public static MjpegServer server1;

  /**
   * Creates a new CameraSubsystem that initializes the cameras and begins to
   * store the camera servers.
   */
  public CameraSubsystem() {
  }

  public static void cameraServerInit() {
    camera1 = CameraServer.startAutomaticCapture();
    cvSink = CameraServer.getVideo();
    cvSource = CameraServer.putVideo("Camera1", 640, 480);
    server1 = CameraServer.addServer("Server1");
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}