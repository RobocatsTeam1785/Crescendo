// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class RetractAmpSubsystem extends Command {
  private AmpSubsystem ampSubsystem;
  private Timer timer = new Timer();
  /** Creates a new ExtendAmpSubsystem. */
  public RetractAmpSubsystem(AmpSubsystem amp) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(amp);
    ampSubsystem = amp;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.stop();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ampSubsystem.setVoltage(3);
    if(timer.hasElapsed(1)){
      this.cancel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ampSubsystem.setVoltage(0);
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
