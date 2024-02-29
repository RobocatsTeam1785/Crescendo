// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Constants.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer;

public class ShootCommand extends Command {
  private ShooterSubsystem shooterSubsystem;
  private ShooterFeederSubsystem shooterFeederSubsystem;
  private Timer timer;
  private boolean feeding;
  /** Creates a new ShootCommand. */
  public ShootCommand(ShooterSubsystem shooter, ShooterFeederSubsystem shooterFeeder) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, shooterFeeder);
    shooterSubsystem = shooter;
    shooterFeederSubsystem = shooterFeeder;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!shooterFeederSubsystem.getPhotoSensor()){
      this.cancel();
      feeding=false;
      timer.stop();
      timer.reset();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setVelocity(ShooterConstants.DEFAULT_RPM);
    if(Math.abs(Math.abs(shooterSubsystem.getTopVelocity())-ShooterConstants.DEFAULT_RPM)<ShooterConstants.MARGIN_OF_ERROR &&
        Math.abs(Math.abs(shooterSubsystem.getBottomVelocity())-ShooterConstants.DEFAULT_RPM)<ShooterConstants.MARGIN_OF_ERROR &&
        !feeding){
      feeding=true;
      timer.start();
    }

    if(feeding){
      if(timer.hasElapsed(0.5)){
        this.cancel();
      }
      else{
        shooterFeederSubsystem.setVelocity(0.5);
      }
    }

    else{
      shooterFeederSubsystem.setVelocity(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    feeding=false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}