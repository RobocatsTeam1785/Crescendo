// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Constants.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class ShootProtectedZone extends Command {
  private ShooterSubsystem shooterSubsystem;
  private ShooterFeederSubsystem shooterFeederSubsystem;
  private ShooterRotSubsystem shooterRotSubsystem;
  private Timer timer;
  private Timer timer2;
  private boolean feeding;
  /** Creates a new ShootCommand. */
  public ShootProtectedZone(ShooterSubsystem shooter, ShooterFeederSubsystem shooterFeeder, ShooterRotSubsystem shooterRot) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, shooterFeeder, shooterRot);
    shooterSubsystem = shooter;
    shooterFeederSubsystem = shooterFeeder;
    shooterRotSubsystem = shooterRot;
    timer = new Timer();
    timer2 = new Timer();
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
    else{
      timer2.stop();
      timer2.reset();
      timer2.start();
      shooterRotSubsystem.setGoal(ShooterRotConstants.PROTECTED_ZONE_ANGLE);
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
    if(timer2.hasElapsed(1.0)){
      feeding=true;
      timer2.stop();
      timer2.reset();
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
