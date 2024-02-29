// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.lib.Constants.ShooterConstants;
import com.revrobotics.RelativeEncoder;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax topShooterMotor;
  private CANSparkMax bottomShooterMotor;

  private RelativeEncoder topEncoder;
  private RelativeEncoder bottomEncoder;
  
  private PIDController topPIDController;
  private PIDController bottomPIDController;

  private SimpleMotorFeedforward topFeedforward;
  private SimpleMotorFeedforward bottomFeedforward;
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    topShooterMotor = new CANSparkMax(ShooterConstants.TOP_MOTOR_ID, MotorType.kBrushless);
    bottomShooterMotor = new CANSparkMax(ShooterConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
    topShooterMotor.setSmartCurrentLimit(40);
    bottomShooterMotor.setSmartCurrentLimit(40);
    topShooterMotor.setInverted(true);
    bottomShooterMotor.setInverted(true);
    topShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    bottomShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    topEncoder = topShooterMotor.getEncoder();
    bottomEncoder = bottomShooterMotor.getEncoder();
    topEncoder.setPositionConversionFactor(ShooterConstants.ENCODER_CONVERSION_FACTOR);
    bottomEncoder.setPositionConversionFactor(ShooterConstants.ENCODER_CONVERSION_FACTOR);
    topEncoder.setVelocityConversionFactor(ShooterConstants.ENCODER_CONVERSION_FACTOR/60);
    bottomEncoder.setVelocityConversionFactor(ShooterConstants.ENCODER_CONVERSION_FACTOR/60);

    topPIDController = new PIDController(
      ShooterConstants.TOP_KP,
      ShooterConstants.TOP_KI,
      ShooterConstants.TOP_KD
    );

    bottomPIDController = new PIDController(
      ShooterConstants.BOTTOM_KP,
      ShooterConstants.BOTTOM_KI,
      ShooterConstants.BOTTOM_KD
    );

    topFeedforward = new SimpleMotorFeedforward(
      ShooterConstants.TOP_KS,
      ShooterConstants.TOP_KV,
      ShooterConstants.TOP_KA
    );

    bottomFeedforward = new SimpleMotorFeedforward(
      ShooterConstants.BOTTOM_KS,
      ShooterConstants.BOTTOM_KV,
      ShooterConstants.BOTTOM_KA
    );    
  }

  public void setVelocity(double velocity){
    double topPID = topPIDController.calculate(topEncoder.getVelocity(), velocity);
    double topFF = topFeedforward.calculate(velocity);

    double bottomPID = bottomPIDController.calculate(bottomEncoder.getVelocity(), velocity);
    double bottomFF = bottomFeedforward.calculate(velocity);


    topShooterMotor.setVoltage(topPID + topFF);
    bottomShooterMotor.setVoltage(bottomPID + bottomFF);
    SmartDashboard.putNumber("Top Shooter RPM", topEncoder.getVelocity()*60);
    SmartDashboard.putNumber("Bottom Shooter Velocity", bottomEncoder.getVelocity()*60);
  }


  public double getTopVelocity(){
    return topEncoder.getVelocity();
  }

  public double getBottomVelocity(){
    return bottomEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
