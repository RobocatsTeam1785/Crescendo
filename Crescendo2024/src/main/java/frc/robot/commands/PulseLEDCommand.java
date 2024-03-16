// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import java.util.ArrayList;

public class PulseLEDCommand extends Command {
  private final int LOOPS_PER_SECOND = 50; //1000ms / 20ms

  private final int PULSE_LENGTH = 25;

  private final int VOID_LENGTH = 0;

  private final int TOTAL_LED_LENGTH = 150;

  private final int R1 = 0;

  private final int R2 = 255;
  
  private final int G1 = 0;

  private final int G2 = 0;

  private final int B1 = 255;
  
  private final int B2 = 255;

  private boolean isColored = false;

  private int currentLength = 0;

  private boolean isFirst = false;

  private ArrayList<Integer[]> leds = new ArrayList();

  private LEDSubsystem ledSubsystem;
  /** Creates a new PulseLEDCommand. */
  public PulseLEDCommand(LEDSubsystem led) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(led);
    ledSubsystem = led;
    for(int i = 0; i < TOTAL_LED_LENGTH; i++){
      leds.add(new Integer[3]);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    resetLeds();
    isColored = true;
    currentLength = 0;
    isFirst = true;
  }

  private void resetLeds(){
    for (Integer[] l : leds){
      l[0]=0;
      l[1]=0;
      l[2]=0;
    }
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(isColored){
      if(currentLength>=PULSE_LENGTH){
        currentLength = 0;
        isColored = false;
        isFirst = !isFirst;
      }
      else{
        currentLength++;
      }
    }
    else{
      if(currentLength>=VOID_LENGTH){
        currentLength = 0;
        isColored = true;
      }
      else{
        currentLength++;
      }
    }



    if(isColored){
      if(isFirst){
        leds.add(0,new Integer[]{R1, G1, B1});
        leds.remove(leds.size()-1);
      }
      else{
        leds.add(0,new Integer[]{R2, G2, B2});
        leds.remove(leds.size()-1);
      }
    }
    else{
      leds.add(0,new Integer[]{0,0,0});
      leds.remove(leds.size()-1);
    }
    ledSubsystem.displayCustomColors(leds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
