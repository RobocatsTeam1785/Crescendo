// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Constants.*;
import frc.robot.subsystems.*;

public class ReverseIntake extends Command {
  private IntakeSubsystem intakeSubsystem;
  private ShooterFeederSubsystem shooterFeederSubsystem;
  private ShooterRotSubsystem shooterRotSubsystem;
  /** Creates a new IntakeCommand. */
  public ReverseIntake(IntakeSubsystem intake, ShooterFeederSubsystem shooterFeeder, ShooterRotSubsystem shooterRot) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooterFeeder, shooterRot);
    intakeSubsystem = intake;
    shooterFeederSubsystem = shooterFeeder;
    shooterRotSubsystem = shooterRot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      shooterRotSubsystem.setGoal(ShooterRotConstants.INTAKE_ANGLE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.handleIntake(0.75);
    if(Math.abs(shooterRotSubsystem.getMeasurement()-ShooterRotConstants.INTAKE_ANGLE)*180/Math.PI<1.5){
      shooterFeederSubsystem.setVelocity(-0.5);
    }
    else{
      shooterFeederSubsystem.setVelocity(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}
