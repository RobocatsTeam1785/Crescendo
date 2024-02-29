// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.Constants.IntakeConstants;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax topMotor;
  private CANSparkMax bottomMotor;

  private RelativeEncoder topEncoder;
  private RelativeEncoder bottomEncoder;

  private DigitalInput photoSensor;

  private PIDController topPIDController;
  private PIDController bottomPIDController;

  private SimpleMotorFeedforward topFeedforward;
  private SimpleMotorFeedforward bottomFeedforward;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    topMotor = new CANSparkMax(IntakeConstants.TOP_MOTOR_ID, MotorType.kBrushless);
    bottomMotor = new CANSparkMax(IntakeConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
    topMotor.setInverted(false);
    bottomMotor.setInverted(false);

    topMotor.setSmartCurrentLimit(40);
    bottomMotor.setSmartCurrentLimit(40);

    topMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    bottomMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    topEncoder = topMotor.getEncoder();
    bottomEncoder = bottomMotor.getEncoder();
    topEncoder.setPositionConversionFactor(IntakeConstants.ENCODER_CONVERSION_FACTOR);
    bottomEncoder.setPositionConversionFactor(IntakeConstants.ENCODER_CONVERSION_FACTOR);
    topEncoder.setVelocityConversionFactor(IntakeConstants.ENCODER_CONVERSION_FACTOR/60);
    bottomEncoder.setVelocityConversionFactor(IntakeConstants.ENCODER_CONVERSION_FACTOR/60);

    //photoSensor = new DigitalInput(IntakeConstants.PHOTO_ELECTRIC_SENSOR_ID);

    topPIDController = new PIDController(
      IntakeConstants.TOP_KP,
      IntakeConstants.TOP_KI,
      IntakeConstants.TOP_KD
    );
    bottomPIDController = new PIDController(
      IntakeConstants.BOTTOM_KP,
      IntakeConstants.BOTTOM_KI,
      IntakeConstants.BOTTOM_KD
    );

    topFeedforward = new SimpleMotorFeedforward(
      IntakeConstants.TOP_KS,
      IntakeConstants.TOP_KV,
      IntakeConstants.TOP_KA
    );
    bottomFeedforward = new SimpleMotorFeedforward(
      IntakeConstants.BOTTOM_KS,
      IntakeConstants.BOTTOM_KV,
      IntakeConstants.BOTTOM_KA
    );
  }

  public void handleIntake(double velocity){
    velocity = velocity * IntakeConstants.MAX_SPEED_TOP;
    SmartDashboard.putNumber("Intake Velocity Setpoint", velocity);
    double topPID = topPIDController.calculate(topEncoder.getVelocity(), velocity);
    double topFF = topFeedforward.calculate(velocity);

    double bottomPID = bottomPIDController.calculate(bottomEncoder.getVelocity(), velocity);
    double bottomFF = bottomFeedforward.calculate(velocity);

    topMotor.setVoltage(topPID+topFF);
    bottomMotor.setVoltage(bottomPID+bottomFF);
  }

  public double getVelocityTop(){
    return topEncoder.getVelocity();
  }

  public double getVelocityBottom(){
    return bottomEncoder.getVelocity();
  }

  public boolean getPhotoSensor(){
    return photoSensor.get();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Top Velocity", topEncoder.getVelocity());
    SmartDashboard.putNumber("Intake Bottom Velocity", bottomEncoder.getVelocity());
  }
}
