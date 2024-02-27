// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class ShooterFeeder2 extends SubsystemBase {
  private CANSparkMax motor = new CANSparkMax(14, MotorType.kBrushless);
  /** Creates a new ShooterFeeder2. */
  public ShooterFeeder2() {
    motor.getEncoder().setPositionConversionFactor(1.0/4.0);
    motor.getEncoder().setVelocityConversionFactor(1.0/4.0/60.0);
  }

  public void handleIntake(double topPow){
    topPow = MathUtil.applyDeadband(topPow,0.1);
    motor.set(topPow);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("top feeder", motor.getEncoder().getVelocity());
  }
}
