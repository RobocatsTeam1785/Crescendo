// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Constants.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class ShootTrapCommand extends Command {
  private ShooterSubsystem shooterSubsystem;
  private ShooterFeederSubsystem shooterFeederSubsystem;
  private ShooterRotSubsystem shooterRotSubsystem;
  private Timer timer;
  private Timer timer2;
  private boolean feeding;
  private boolean done;
    /** Creates a new ShootCommand. */
  public ShootTrapCommand(ShooterSubsystem shooter, ShooterFeederSubsystem shooterFeeder, ShooterRotSubsystem shooterRot) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, shooterFeeder, shooterRot);
    shooterSubsystem = shooter;
    shooterFeederSubsystem = shooterFeeder;
    shooterRotSubsystem = shooterRot;
    timer = new Timer();
    timer2 = new Timer();
    done = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("SC Start");

    if(!shooterFeederSubsystem.getPhotoSensor()){
      done=true;
      this.cancel();
      feeding=false;
      timer.stop();
      timer.reset();
    }
    else{
      done=false;
      timer2.stop();
      timer2.reset();
      timer2.start();
      shooterRotSubsystem.setGoal(ShooterRotConstants.CLOSE_STAGE_ANGLE);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setVelocity(3000/60);
    if(Math.abs(Math.abs(shooterSubsystem.getTopVelocity())-3000/60)<ShooterConstants.MARGIN_OF_ERROR &&
        Math.abs(Math.abs(shooterSubsystem.getBottomVelocity())-3000/60)<ShooterConstants.MARGIN_OF_ERROR &&
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
        done=true;
        this.cancel();
      }
      else{
        shooterFeederSubsystem.setVelocity(0.75);
      }
    }

    else{
      shooterFeederSubsystem.setVelocity(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("SC End");

    timer.stop();
    timer.reset();
    timer2.stop();
    timer2.reset();
    shooterFeederSubsystem.setVelocity(0);
    //shooterSubsystem.setVelocity(0);
    feeding=false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
