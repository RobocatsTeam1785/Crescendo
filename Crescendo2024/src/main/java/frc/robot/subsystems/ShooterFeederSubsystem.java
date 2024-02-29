// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.lib.Constants.ShooterFeederConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterFeederSubsystem extends SubsystemBase {
  private CANSparkMax motor;
  private RelativeEncoder encoder;

  private PIDController pidController;
  private SimpleMotorFeedforward feedforward;

  private DigitalInput photoSensor;
  /** Creates a new ShooterFeederSubsystem. */
  public ShooterFeederSubsystem() {
    motor = new CANSparkMax(ShooterFeederConstants.MOTOR_ID, MotorType.kBrushless);
    motor.setInverted(false);
    motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    motor.setSmartCurrentLimit(40);
    
    encoder = motor.getEncoder();
    encoder.setPositionConversionFactor(ShooterFeederConstants.ENCODER_CONVERSION_FACTOR);
    encoder.setVelocityConversionFactor(ShooterFeederConstants.ENCODER_CONVERSION_FACTOR/60);

    pidController = new PIDController(
      ShooterFeederConstants.KP,
      ShooterFeederConstants.KI,
      ShooterFeederConstants.KD
    );

    feedforward = new SimpleMotorFeedforward(
      ShooterFeederConstants.KS,
      ShooterFeederConstants.KV,
      ShooterFeederConstants.KA
    );

    photoSensor = new DigitalInput(ShooterFeederConstants.PHOTO_ELECTRIC_SENSOR_ID);
  }

  public void setVelocity(double velocity){
    velocity = velocity * ShooterFeederConstants.MAX_RPM;
    /*if(photoSensor.get()){
      velocity = 0;
    }*/
    SmartDashboard.putNumber("Shooter feeder target velocity", velocity);
    double pid = pidController.calculate(encoder.getVelocity(), velocity);
    double ff = feedforward.calculate(velocity);

    motor.setVoltage(pid + ff);
  }

  public double getVelocity(){
    return encoder.getVelocity();
  }

  public boolean getPhotoSensor(){
    return photoSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter feeder velocity", encoder.getVelocity());
    SmartDashboard.putBoolean("Shooter feeder photo sensor", photoSensor.get());
  }
}
