// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax FLDrive = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax FLTurn = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax FRDrive = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax FRTurn = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax BLDrive = new CANSparkMax(5, MotorType.kBrushless);
  private final CANSparkMax BLTurn = new CANSparkMax(6, MotorType.kBrushless);
  private final CANSparkMax BRDrive = new CANSparkMax(7, MotorType.kBrushless);
  private final CANSparkMax BRTurn = new CANSparkMax(8, MotorType.kBrushless);

  private final CANcoder canCoder1 = new CANcoder(1);
  private final CANcoder canCoder2 = new CANcoder(2);
  private final CANcoder canCoder3 = new CANcoder(3);
  private final CANcoder canCoder4 = new CANcoder(4);

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  private boolean isBrake = true;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    gyro.reset();
    FLDrive.setIdleMode(CANSparkMax.IdleMode.kBrake);
    FLTurn.setIdleMode(CANSparkMax.IdleMode.kBrake);
    FRDrive.setIdleMode(CANSparkMax.IdleMode.kBrake);
    FRTurn.setIdleMode(CANSparkMax.IdleMode.kBrake);
    BLDrive.setIdleMode(CANSparkMax.IdleMode.kBrake);
    BLTurn.setIdleMode(CANSparkMax.IdleMode.kBrake);
    BRDrive.setIdleMode(CANSparkMax.IdleMode.kBrake);
    BRTurn.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public void handleDrive(double drivePow, double turnPow){
    FLDrive.set(drivePow);
    FLTurn.set(turnPow);
    FRDrive.set(drivePow);
    FRTurn.set(turnPow);
    BLDrive.set(drivePow);
    BLTurn.set(turnPow);
    BRDrive.set(drivePow);
    BRTurn.set(turnPow);
  }

  public void handleDrive(double drivePow1, double drivePow2, double drivePow3, double drivePow4, double drivePow5, double drivePow6, double drivePow7, double drivePow8){
    FLDrive.set(drivePow1);
    FLTurn.set(drivePow2);
    FRDrive.set(drivePow3);
    FRTurn.set(drivePow4);
    BLDrive.set(drivePow5);
    BLTurn.set(drivePow6);
    BRDrive.set(drivePow7);
    BRTurn.set(drivePow8);
  }

  public void toggleMode(){
    if(isBrake){
      isBrake = false;
      FLDrive.setIdleMode(CANSparkMax.IdleMode.kCoast);
      FLTurn.setIdleMode(CANSparkMax.IdleMode.kCoast);
      FRDrive.setIdleMode(CANSparkMax.IdleMode.kCoast);
      FRTurn.setIdleMode(CANSparkMax.IdleMode.kCoast);
      BLDrive.setIdleMode(CANSparkMax.IdleMode.kCoast);
      BLTurn.setIdleMode(CANSparkMax.IdleMode.kCoast);
      BRDrive.setIdleMode(CANSparkMax.IdleMode.kCoast);
      BRTurn.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }
    else{
      isBrake = true;
      FLDrive.setIdleMode(CANSparkMax.IdleMode.kBrake);
      FLTurn.setIdleMode(CANSparkMax.IdleMode.kBrake);
      FRDrive.setIdleMode(CANSparkMax.IdleMode.kBrake);
      FRTurn.setIdleMode(CANSparkMax.IdleMode.kBrake);
      BLDrive.setIdleMode(CANSparkMax.IdleMode.kBrake);
      BLTurn.setIdleMode(CANSparkMax.IdleMode.kBrake);
      BRDrive.setIdleMode(CANSparkMax.IdleMode.kBrake);
      BRTurn.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Gyro angle:", gyro.getAngle());
    SmartDashboard.putNumber("Gyro pitch", gyro.getPitch());
    SmartDashboard.putNumber("Gyro roll", gyro.getRoll());
    SmartDashboard.putNumber("CANCoder1 value", canCoder1.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("CANCoder2 value", canCoder2.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("CANCoder3 value", canCoder3.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("CANCoder4 value", canCoder4.getAbsolutePosition().getValueAsDouble());
  }
}
