// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.Timer;


public class BlinkLEDGreen extends Command {
  private int counter;
  private Timer timer;
  private LEDSubsystem ledSubsystem;

  private final double BLINK_TIME = 0.5;
  private final double BLINKS = 3;
  /** Creates a new BlinkLEDGreen. */
  public BlinkLEDGreen(LEDSubsystem led) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ledSubsystem);
    ledSubsystem = led;
    counter = 0;
    timer = new Timer();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.stop();
    timer.reset();
    timer.start();
    counter = 0;
    ledSubsystem.green();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.hasElapsed(BLINK_TIME)){
      timer.stop();
      timer.reset();
      timer.start();
      counter+=1;
      if(counter>=BLINKS*2-1){
        this.cancel();
      }
      else if(counter%2==1){
        ledSubsystem.turnOff();
      }
      else{
        ledSubsystem.green();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ledSubsystem.turnOff();
    timer.stop();
    timer.reset();
    counter = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
