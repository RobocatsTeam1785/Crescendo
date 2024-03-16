// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import frc.lib.Constants.ClimberConstants;
import com.revrobotics.SparkLimitSwitch.Type;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberSubsystem extends SubsystemBase {
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;

  private SparkLimitSwitch leftLimitSwitch;
  private SparkLimitSwitch rightLimitSwitch;

  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    leftMotor = new CANSparkMax(ClimberConstants.MOTOR_ID_LEFT, MotorType.kBrushless);
    rightMotor = new CANSparkMax(ClimberConstants.MOTOR_ID_RIGHT, MotorType.kBrushless);
    leftMotor.setInverted(false);
    rightMotor.setInverted(true);
    leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    leftLimitSwitch = leftMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    rightLimitSwitch = rightMotor.getForwardLimitSwitch(Type.kNormallyOpen);

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();
    leftEncoder.setPositionConversionFactor(ClimberConstants.ENCODER_CONVERSION_FACTOR);
    leftEncoder.setVelocityConversionFactor(ClimberConstants.ENCODER_CONVERSION_FACTOR / 60);
    rightEncoder.setPositionConversionFactor(ClimberConstants.ENCODER_CONVERSION_FACTOR);
    rightEncoder.setVelocityConversionFactor(ClimberConstants.ENCODER_CONVERSION_FACTOR / 60);
  }

  public void handleClimbers(double leftPow, double rightPow){
    leftPow = MathUtil.applyDeadband(leftPow,0.1);
    rightPow = MathUtil.applyDeadband(rightPow,0.1);

    if(leftLimitSwitch.isPressed() && leftPow<0){
      //leftPow = 0;
    }
    else{
      leftPow = -leftPow;
    }
    if(rightLimitSwitch.isPressed() && rightPow<0){
      //rightPow = 0;
    }
    leftMotor.setVoltage(leftPow * ClimberConstants.MAX_VOLTAGE);
    rightMotor.setVoltage(rightPow * ClimberConstants.MAX_VOLTAGE);

    SmartDashboard.putNumber("Left Climber Voltage", leftPow * ClimberConstants.MAX_VOLTAGE);
    SmartDashboard.putNumber("Right Climber Voltage", rightPow * ClimberConstants.MAX_VOLTAGE);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Climber Position", leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Climber Position", rightEncoder.getPosition());
    SmartDashboard.putBoolean("Left Climber Limit Switch Pressed", leftLimitSwitch.isPressed());
    SmartDashboard.putBoolean("Right Climber Limit Switch", rightLimitSwitch.isPressed());
  }
}
