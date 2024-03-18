// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Constants.VisionConstants;
import frc.lib.Utils.Util1785;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class TurnShooterToSpeaker extends Command {
  private final double MARGIN_OF_ERROR = 2;
  private DriveSubsystem driveSubsystem;
  private Timer timer;
  private boolean done;

  /** Creates a new TurnShooterToSpeaker. */
  public TurnShooterToSpeaker(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    driveSubsystem = drive;
    timer = new Timer();
    done = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(180-Math.abs(driveSubsystem.getGyro().getAngle())<MARGIN_OF_ERROR){
      this.cancel();
      done = true;
    }
    else{
      timer.stop();
      timer.reset();
      timer.start();
      done = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(180-Math.abs(driveSubsystem.getGyro().getAngle())<MARGIN_OF_ERROR || timer.hasElapsed(1.5)){
      this.cancel();
      done = true;
    }
    else{
      driveSubsystem.driveOnlyHeading(180);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    driveSubsystem.drive(
            0,
            0,
            0,
            0,
            0,
            0,
            -1,
            0,
            true,
            0
        );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
