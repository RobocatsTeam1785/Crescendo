// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Constants.*;
import frc.robot.subsystems.CameraSubsystem;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private RobotContainer robotContainer;
  private Command autoCommand;
  private boolean started;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    started=false;
    robotContainer = new RobotContainer();
    robotContainer.resetGyro();
    robotContainer.zeroSwerveModules();
    robotContainer.getPulseLEDCommand().initialize();
    //CameraSubsystem.cameraServerInit();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    robotContainer.setPeriod(getPeriod());
    if(!started){
      robotContainer.getPulseLEDCommand().execute();
    }
  }

  @Override
  public void autonomousInit() {
    started=true;
    autoCommand = robotContainer.getAutonomousCommand();
    robotContainer.getPulseLEDCommand().cancel();
    robotContainer.zeroSwerveModules();
    if(autoCommand != null){
      autoCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    started=true;
    if(autoCommand!=null){
      autoCommand.cancel();
    }
    robotContainer.getPulseLEDCommand().cancel();
  }

  @Override
  public void teleopPeriodic() {
    robotContainer.e();
    robotContainer.helpGriffin();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
