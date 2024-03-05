// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.OptionalInt;

// public void robotInit() {
    //   // PWM port 9
    //   // Must be a PWM header, not MXP or DIO
    //   m_led = new AddressableLED(9);
import edu.wpi.first.hal.AllianceStationID;

//   // Reuse buffer
    //   // Default to a length of 60, start empty output
    //   // Length is expensive to set, so only set it once, then just update data
    //   m_ledBuffer = new AddressableLEDBuffer(60);
    //   m_led.setLength(m_ledBuffer.getLength());
  
    //   // Set the data
    //   m_led.setData(m_ledBuffer);
    //   m_led.start();
    // }

  /** Creates a new LedSubsystem. */


/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/



import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Constants.*;
public class LEDSubsystem extends SubsystemBase {
  /**
   * Creates a new LEDSubsystem.
   */

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  // Store what the last hue of the first pixel is
  private int m_rainbowFirstPixelHue;

  private int bluePulseBrightness = 0;
  private int blueStreakLED = 0;
  private int numLoops = 0;
  
  public LEDSubsystem() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(4); // create constant (if not 9?)

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(100);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Fill the buffer with a rainbow

  }




  public void blue() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 0, 0, 255);
   }
   
   m_led.setData(m_ledBuffer);
  }

  public void holdBlue(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 0, 0, 255);
   }
   
   m_led.setData(m_ledBuffer);
  }

  public void turnOff(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 0, 0, 0);
   }
   
   m_led.setData(m_ledBuffer);
  }

  public void green() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 0, 255, 0);
   }
   
   m_led.setData(m_ledBuffer);
  }







  public void blueStreak(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for blue
      m_ledBuffer.setRGB(i, 0, 0, 255);
   }

   //turns one led off
   m_ledBuffer.setRGB(blueStreakLED, 0, 0, 0);

   //increase brightness
   if (numLoops%3 == 0){
      blueStreakLED += 1;


      //Check bounds
      blueStreakLED %= m_ledBuffer.getLength();
    }

   m_led.setData(m_ledBuffer);


   numLoops += 1;
   //Timer.delay(0.2);
   

  }
}