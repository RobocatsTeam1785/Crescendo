// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.lib.Constants.ShooterRotConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;


public class ShooterRotSubsystem extends ProfiledPIDSubsystem {
  private CANSparkMax motor;

  private DutyCycleEncoder hexEncoder;

  private ArmFeedforward armFeedforward;
  private InterpolatingDoubleTreeMap interpolatingDoubleTreeMap = new InterpolatingDoubleTreeMap();

  /** Creates a new ShooterRotSubsystem. */
  public ShooterRotSubsystem() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            ShooterRotConstants.KP_VALUE,
            ShooterRotConstants.KI_VALUE,
            ShooterRotConstants.KD_VALUE,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(
              ShooterRotConstants.MAX_ROTATIONAL_SPEED,
              ShooterRotConstants.MAX_ACCELERATION
            )
        )
    );
    getController().enableContinuousInput(-Math.PI, Math.PI);

    motor = new CANSparkMax(ShooterRotConstants.MOTOR_ID, MotorType.kBrushless);
    motor.setInverted(false);
    motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    hexEncoder = new DutyCycleEncoder(ShooterRotConstants.SHAFT_ENCODER_ID);

    armFeedforward = new ArmFeedforward(
      ShooterRotConstants.KS_VALUE,
      ShooterRotConstants.KG_VALUE,
      ShooterRotConstants.KV_VALUE,
      ShooterRotConstants.KA_VALUE
    );
    setGoal((45-90)*Math.PI/180);
    interpolatingDoubleTreeMap.put(1.0,(55-2-90)*Math.PI/180);
    interpolatingDoubleTreeMap.put(2.0,(43.0-2-90)*Math.PI/180);
    interpolatingDoubleTreeMap.put(3.0,(36.0-4.5-90)*Math.PI/180);
    interpolatingDoubleTreeMap.put(4.0,(32-5.5-90)*Math.PI/180);
    interpolatingDoubleTreeMap.put(5.0,(29.75-6-90)*Math.PI/180);
    interpolatingDoubleTreeMap.put(6.0,(29.0-2-90)*Math.PI/180);


  }

  public double getEstimatedAngle(double distance){
    return interpolatingDoubleTreeMap.get(distance);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    double feedforward = armFeedforward.calculate(setpoint.position, setpoint.velocity);
    motor.setVoltage(output + feedforward);
    SmartDashboard.putNumber("Shooter rot voltage", output + feedforward);
    SmartDashboard.putNumber("REV BORE ENCODEOREOROERO", hexEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("ShooterAngle", getMeasurement()*180/Math.PI);
    SmartDashboard.putNumber("true", getTrueAngle());
    SmartDashboard.putNumber("PID Output", output);
    SmartDashboard.putNumber("Feedforward output", feedforward);

    SmartDashboard.putNumber("Shooter Angle Goal", setpoint.position*180/Math.PI+90);
    SmartDashboard.putNumber("Shooter Angle Current", getMeasurement()*180/Math.PI+90);

  }

  public double getAngle(){//0-1
    double angle = hexEncoder.getAbsolutePosition();
    angle -= ShooterRotConstants.ROT_OFFSET;
    angle = angle - 0.5;
    angle = angle * 2 * Math.PI;
    angle += Math.PI;
    return angle;
  }

  public double getTrueAngle(){
    return hexEncoder.getAbsolutePosition();
  }


  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getAngle()-Math.PI/2;
  }
}
