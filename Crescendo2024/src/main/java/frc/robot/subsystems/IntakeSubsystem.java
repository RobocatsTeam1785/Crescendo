// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax topMotor = new CANSparkMax(9, MotorType.kBrushless);
  private final CANSparkMax bottomMotor = new CANSparkMax(10, MotorType.kBrushless);

  //private final DigitalInput pesensor = new DigitalInput(1);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    topMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    bottomMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public void handleMotors(double topPow, double bottomPow){
    topMotor.set(-topPow);
    bottomMotor.set(-bottomPow);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run\
    //SmartDashboard.putBoolean("Photosensor reading intake", pesensor.get());
  }
}
