// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterFeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class EjectNoteForward extends Command {
  /** Creates a new EjectNoteForward. */
  private ShooterSubsystem shooterSubsystem;
  private ShooterFeederSubsystem shooterFeederSubsystem;
  /** Creates a new EjectNoteBackwards. */
  public EjectNoteForward(ShooterSubsystem shooter, ShooterFeederSubsystem shooterFeeder) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter,shooterFeeder);
    shooterSubsystem = shooter;
    shooterFeederSubsystem = shooterFeeder;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterFeederSubsystem.setVelocity(0.5);
    shooterSubsystem.setVelocity((1500/60));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
