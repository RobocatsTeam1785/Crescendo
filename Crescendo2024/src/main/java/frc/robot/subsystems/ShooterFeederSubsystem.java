// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.lib.Constants.ShooterFeederConstants;

public class ShooterFeederSubsystem extends SubsystemBase {
  private CANSparkMax motor;
  private RelativeEncoder encoder;

  private PIDController pidController;
  private SimpleMotorFeedforward feedforward;

  private AnalogInput photoSensor;
  /** Creates a new ShooterFeederSubsystem. */
  public ShooterFeederSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
