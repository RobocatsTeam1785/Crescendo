// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.lib.Constants.ShooterRotConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.math.controller.ArmFeedforward;

import com.fasterxml.jackson.databind.util.PrimitiveArrayBuilder;
// import edu.wpi.first.wpilibj.PWMSparkMax;  //DO I NEED?
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
// import edu.wpi.first.math.util.*; 

public class ShooterRotSubsystem extends ProfiledPIDSubsystem {
  private CANSparkMax angleMotor = new CANSparkMax(ShooterRotConstants.MOTOR_ID_1, MotorType.kBrushless);
  private CANSparkMax angleMotor2 = new CANSparkMax(ShooterRotConstants.MOTOR_ID_2, MotorType.kBrushless);//MAYBE IF THERES ANOTHER MOTOR??
  private final RelativeEncoder m_encoder = angleMotor.getEncoder(); //MIGHT NOT NEED THIS BECAUSE WE USE THE PERCISE ONESE
  private double pos = -Math.PI / 2;// MIGHT NOT NEED THIS IF THIS JUST REPRESENTS INIT ANGLE, USE THE CONSTANT
  private final ArmFeedforward m_feedForward = new ArmFeedforward(ShooterRotConstants.KS_VALUE, ShooterRotConstants.KG_VALUE,ShooterRotConstants.KV_VALUE,ShooterRotConstants.KA_VALUE);
  /** Creates a new ShooterRotSubsystem. */
  public ShooterRotSubsystem() {

    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            ShooterRotConstants.KP_VALUE,
            ShooterRotConstants.KI_VALUE,
            ShooterRotConstants.KD_VALUE,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(ShooterRotConstants.MAX_ROTATIONAL_SPEED,ShooterRotConstants.MAX_ROTATIONAL_SPEED))
            ,ShooterRotConstants.INITIAL_POSITION); 
    //SET CONVERSION FACTORS, IDLE MODES, AND INVERTS
    m_encoder.setPosition(0);
    setGoal(ShooterRotConstants.goal);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    double feedForwardOutput = m_feedForward.calculate(setpoint.position, setpoint.velocity);

    angleMotor.setVoltage(output + feedForwardOutput);
    angleMotor2.setVoltage(output + feedForwardOutput);

    SmartDashboard.putNumber("Voltage", output + feedForwardOutput);
    SmartDashboard.putNumber("PID Output", output);
    SmartDashboard.putNumber("FeedForward", feedForwardOutput);
    SmartDashboard.putNumber("Setpoint velocity", getController().getSetpoint().velocity);
    SmartDashboard.putNumber("Setpoint Position ", setpoint.position);

  }

  public double getAngle() //WAIT DO WE NEED THIS/????
  {
    return  0.0; // WORK ON THIS
    
  }
  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    SmartDashboard.putNumber("Encoder pos", m_encoder.getPosition());
    return m_encoder.getPosition();
  }
}
S