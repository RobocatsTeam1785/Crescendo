// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.lib.Constants.VisionConstants;
import frc.lib.Utils.Util1785;
import edu.wpi.first.math.util.Units;

public class AutoAlignCommand extends Command {
  private VisionSubsystem visionSubsystem;
  private DriveSubsystem driveSubsystem;
  private boolean done;
  private double period;

  /** Creates a new AutoAlignCommand. */
  public AutoAlignCommand(DriveSubsystem drive, VisionSubsystem vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    driveSubsystem = drive;
    visionSubsystem = vision;
    done = false;
  }

  public void setPeriod(double p){
    period=p;
  }
  /*
   * driveSubsystem.drive(
            driverController.getLeftX(),
            driverController.getLeftY(),
            -driverController.getRightX(),
            driverController.getRightY(),
            driverController.getLeftTriggerAxis(),
            driverController.getRightTriggerAxis(),
            driverController.getPOV(),
            visionSubsystem.hasSpeakerTarget() ? Util1785.getRobotRelativeAngle(visionSubsystem.getYaw(), Util1785.getDistanceRobotRelative(visionSubsystem.getYaw(), visionSubsystem.getAprilTagDistance(), Units.inchesToMeters(VisionConstants.FRONT_CAM_OFFSET)),Units.inchesToMeters(VisionConstants.FRONT_CAM_OFFSET)) : 0,
            //visionSubsystem.getYaw(),
            true,
            period
        ), driveSubsystem));  
   */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("AA Start");

    if(!visionSubsystem.hasSpeakerTarget()){
      done = true;
      this.cancel();
    }
    else{
      done=false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!visionSubsystem.hasSpeakerTarget()){
      done = true;
      this.cancel();
      System.out.println("bad");
    }
    else{
      if(Math.abs(Util1785.getRobotRelativeAngle(visionSubsystem.getYaw(), Util1785.getDistanceRobotRelative(visionSubsystem.getYaw(), visionSubsystem.getAprilTagDistance(), Units.inchesToMeters(VisionConstants.FRONT_CAM_OFFSET)),Units.inchesToMeters(VisionConstants.FRONT_CAM_OFFSET))) < 1.3){
        done = true;
        this.cancel();
        System.out.println("bad 2");
      }
      else{
        System.out.println("good");
        driveSubsystem.drive(
            0,
            0,
            0,
            0,
            0,
            0.91,
            -1,
            visionSubsystem.hasSpeakerTarget() ? Util1785.getRobotRelativeAngle(visionSubsystem.getYaw(), Util1785.getDistanceRobotRelative(visionSubsystem.getYaw(), visionSubsystem.getAprilTagDistance(), Units.inchesToMeters(VisionConstants.FRONT_CAM_OFFSET)),Units.inchesToMeters(VisionConstants.FRONT_CAM_OFFSET)) : 0,
            true,
            period
        );
      }
    }
    System.out.println("E");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("AA End?");
    driveSubsystem.drive(
            0,
            0,
            0,
            0,
            0,
            0,
            -1,
            visionSubsystem.hasSpeakerTarget() ? Util1785.getRobotRelativeAngle(visionSubsystem.getYaw(), Util1785.getDistanceRobotRelative(visionSubsystem.getYaw(), visionSubsystem.getAprilTagDistance(), Units.inchesToMeters(VisionConstants.FRONT_CAM_OFFSET)),Units.inchesToMeters(VisionConstants.FRONT_CAM_OFFSET)) : 0,
            true,
            period
        );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
