// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.lib.Constants.*;

public class IntakeCommand extends Command {
  private IntakeSubsystem intakeSubsystem;
  private ShooterFeederSubsystem shooterFeederSubsystem;
  private ShooterRotSubsystem shooterRotSubsystem;
  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem intake, ShooterFeederSubsystem shooterFeeder, ShooterRotSubsystem shooterRot) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooterFeeder, shooterRot);
    intakeSubsystem = intake;
    shooterFeederSubsystem = shooterFeeder;
    shooterRotSubsystem = shooterRot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(shooterFeederSubsystem.getPhotoSensor()){
      this.cancel();
    }
    else{
      shooterRotSubsystem.setGoal((30-90)*Math.PI/180);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooterFeederSubsystem.getPhotoSensor()){
      this.cancel();
    }
    else{
      intakeSubsystem.handleIntake(-0.75);
      shooterFeederSubsystem.setVelocity(0.25);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterRotSubsystem.setGoal(-Math.PI/2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
