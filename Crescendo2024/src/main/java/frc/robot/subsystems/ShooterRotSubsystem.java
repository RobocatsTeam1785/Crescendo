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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ShooterRotSubsystem extends ProfiledPIDSubsystem {
  private CANSparkMax motor;

  private DutyCycleEncoder hexEncoder;

  private ArmFeedforward armFeedforward;
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

  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    double feedforward = armFeedforward.calculate(setpoint.position, setpoint.velocity);
    motor.setVoltage(output + feedforward);
    SmartDashboard.putNumber("Shooter rot voltage", output + feedforward);
    double e = motor.getAppliedOutput();
  }

  public double getAngle(){//0-1
    double angle = hexEncoder.getAbsolutePosition();
    angle = angle - 0.5;
    angle = angle * 2 * Math.PI;
    return angle;
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getAngle();
  }
}
