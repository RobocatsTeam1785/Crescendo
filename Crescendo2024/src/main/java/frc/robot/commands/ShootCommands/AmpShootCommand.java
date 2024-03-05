// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Constants.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class AmpShootCommand extends Command {
  private ShooterSubsystem shooterSubsystem;
  private ShooterFeederSubsystem shooterFeederSubsystem;
  private Timer timer;
  private boolean feeding;
  private XboxController controller;
  /** Creates a new ShootCommand. */
  public AmpShootCommand(ShooterSubsystem shooter, ShooterFeederSubsystem shooterFeeder, XboxController c) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, shooterFeeder);
    shooterSubsystem = shooter;
    shooterFeederSubsystem = shooterFeeder;
    timer = new Timer();
    controller = c;
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
    shooterSubsystem.setVelocity(ShooterConstants.AMP_RPM);
    if(Math.abs(Math.abs(shooterSubsystem.getTopVelocity())-ShooterConstants.AMP_RPM)<ShooterConstants.MARGIN_OF_ERROR &&
        Math.abs(Math.abs(shooterSubsystem.getBottomVelocity())-ShooterConstants.AMP_RPM)<ShooterConstants.MARGIN_OF_ERROR &&
        !feeding){
      feeding=true;
      timer.start();
    }
    if(controller.getRightTriggerAxis()>0.9){
      feeding = true;
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
