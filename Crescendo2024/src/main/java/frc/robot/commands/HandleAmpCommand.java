// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterRotSubsystem;
import frc.lib.Constants.*;
import frc.robot.subsystems.ShooterSubsystem;

public class HandleAmpCommand extends Command {
  private ShooterRotSubsystem shooterRotSubsystem;
  private ExtendAmpSubsystem extendAmp;
  private RetractAmpSubsystem retractAmp;
  private LEDSubsystem ledSubsystem;


  /** Creates a new HandleAmpCommand. */
  public HandleAmpCommand(ShooterRotSubsystem shooterRot, ExtendAmpSubsystem extend, RetractAmpSubsystem retract, LEDSubsystem led) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterRot, led);
    shooterRotSubsystem = shooterRot;
    extendAmp = extend;
    retractAmp = retract;
    ledSubsystem = led;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterRotSubsystem.setGoal(ShooterRotConstants.AMP_ANGLE);
    extendAmp.schedule();
    ledSubsystem.blue();
    ShooterSubsystem.isAmping = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    extendAmp.cancel();
    retractAmp.schedule();
    ledSubsystem.turnOff();
    ShooterSubsystem.isAmping = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
