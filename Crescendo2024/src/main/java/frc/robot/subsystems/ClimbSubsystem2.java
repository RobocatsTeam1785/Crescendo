// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.lib.Constants.ClimberConstants;

public class ClimbSubsystem2 extends SubsystemBase {
  private CANSparkMax leftClimber = new CANSparkMax(11, MotorType.kBrushless);
  private CANSparkMax rightClimber = new CANSparkMax(12, MotorType.kBrushless);
  /** Creates a new ClimbSubsystem2. */
  public ClimbSubsystem2() {
    leftClimber.setInverted(false);
    rightClimber.setInverted(true);
  }

  public void handleClimbers(double powLeft, double powRight){
    leftClimber.set(powLeft);
    rightClimber.set(powRight);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
