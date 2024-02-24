// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class IntakeSubsystem2 extends SubsystemBase {
  private CANSparkMax intakeTop = new CANSparkMax(9, MotorType.kBrushless);
  private CANSparkMax intakeBottom = new CANSparkMax(10, MotorType.kBrushless);

    /** Creates a new IntakeSubsystem2. */
  public IntakeSubsystem2() {
    intakeTop.setInverted(true);
    intakeBottom.setInverted(true);
  }

  public void handleIntake(double topPow, double bottomPow){
    topPow = MathUtil.applyDeadband(topPow,0.1);
    bottomPow = MathUtil.applyDeadband(bottomPow,0.1);
    intakeTop.set(topPow);
    intakeBottom.set(bottomPow);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("top speed", intakeTop.getEncoder().getVelocity());
    SmartDashboard.putNumber("bottom speed", intakeBottom.getEncoder().getVelocity());
  }
}
