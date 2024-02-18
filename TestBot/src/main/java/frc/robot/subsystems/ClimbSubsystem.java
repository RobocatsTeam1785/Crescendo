// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimbSubsystem extends SubsystemBase {
  private final CANSparkMax climberLeft = new CANSparkMax(11, MotorType.kBrushless);
  private final CANSparkMax climberRight = new CANSparkMax(12, MotorType.kBrushless);
  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    climberLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
    climberRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public void handleClimbers(double leftPow, double rightPow){
    climberLeft.set(leftPow);
    climberRight.set(rightPow);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
