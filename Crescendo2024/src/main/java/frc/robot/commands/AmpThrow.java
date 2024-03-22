// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpSubsystem;

public class AmpThrow extends Command {
  private AmpSubsystem ampSubsystem;
  /** Creates a new AmpThrow. */
  public AmpThrow(AmpSubsystem amp) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(amp);
    ampSubsystem = amp;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ampSubsystem.setVoltage(-5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ampSubsystem.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
