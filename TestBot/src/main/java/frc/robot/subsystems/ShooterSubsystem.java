// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;


public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax shooterMotorTop = new CANSparkMax(15, MotorType.kBrushless);
  private final CANSparkMax shooterMotorBottom = new CANSparkMax(16, MotorType.kBrushless);
  private final CANSparkMax shooterRotMotor = new CANSparkMax(13, MotorType.kBrushless);
  private final CANSparkMax shooterFeederMotor = new CANSparkMax(14, MotorType.kBrushless);

  private final DutyCycleEncoder hexEncoder = new DutyCycleEncoder(5);

  private final DigitalInput pesensor = new DigitalInput(2);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    shooterMotorTop.setIdleMode(CANSparkMax.IdleMode.kCoast);
    shooterMotorBottom.setIdleMode(CANSparkMax.IdleMode.kCoast);
    shooterRotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    shooterFeederMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }
  
  public void handleMotors(double shooterPowTop, double shooterPowBottom, double feederPow, double rotPow){
    shooterMotorTop.set(shooterPowTop);
    shooterMotorBottom.set(shooterPowBottom);
    shooterRotMotor.set(feederPow);
    shooterFeederMotor.set(rotPow);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Photosensor reading shooter", pesensor.get());
    SmartDashboard.putNumber("Hex encoder reading", hexEncoder.getAbsolutePosition());
  }
}
