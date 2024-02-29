// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AmpSubsystem extends SubsystemBase {
  private CANSparkMax motor;
  /** Creates a new AmpSubsystem. */
  public AmpSubsystem() {
    motor = new CANSparkMax(17, MotorType.kBrushless);
    motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public void setVoltage(double voltage){
    motor.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
