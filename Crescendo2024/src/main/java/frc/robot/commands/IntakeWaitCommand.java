// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.lib.Constants.*;
import edu.wpi.first.wpilibj.Timer;

public class IntakeWaitCommand extends Command {
  private Timer timer;
  private IntakeSubsystem intakeSubsystem;
  private ShooterFeederSubsystem shooterFeederSubsystem;
  private ShooterRotSubsystem shooterRotSubsystem;
  private BlinkLEDGreen blinkLEDGreen;
  private BlinkBlueCommand blinkBlueCommand;
  private LEDSubsystem ledSubsystem;
  private boolean done;
  /** Creates a new IntakeCommand. */
  public IntakeWaitCommand(IntakeSubsystem intake, ShooterFeederSubsystem shooterFeeder, ShooterRotSubsystem shooterRot, LEDSubsystem led) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooterFeeder, shooterRot);
    timer = new Timer();

    intakeSubsystem = intake;
    shooterFeederSubsystem = shooterFeeder;
    shooterRotSubsystem = shooterRot;
    blinkLEDGreen = new BlinkLEDGreen(led);
    blinkBlueCommand = new BlinkBlueCommand(led);
    done=false;
    ledSubsystem = led;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("IW Start");
    if(shooterFeederSubsystem.getPhotoSensor()){
      this.cancel();
      done=true;
    }
    else{
      shooterRotSubsystem.setGoal(ShooterRotConstants.INTAKE_ANGLE);
      done=false;
      ledSubsystem.white();
      timer.stop();
      timer.reset();
      timer.start();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooterFeederSubsystem.getPhotoSensor() || timer.hasElapsed(0.75)){
      blinkLEDGreen.schedule();
      done=true;
      this.cancel();
    }
    else{
      intakeSubsystem.handleIntake(-0.85);
      shooterFeederSubsystem.setVelocity(0.3);
    }
    
    if(intakeSubsystem.getPhotoSensor()){
      ledSubsystem.green();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("IW End");

    timer.stop();
    timer.reset();
    intakeSubsystem.handleIntake(0);
    shooterFeederSubsystem.setVelocity(0);
    if(!blinkLEDGreen.isScheduled() && !done){
      ledSubsystem.changeColorToAlliance();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
