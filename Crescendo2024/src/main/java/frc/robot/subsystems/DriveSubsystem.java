// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase {
  private CANcoder code = new CANcoder(1);
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("getName()", code.getAbsolutePosition().getValueAsDouble());
    // This method will be called once per scheduler run
  }
}
