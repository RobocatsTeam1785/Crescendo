// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DriverStation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterRotSubsystem;
public class AutoAngleCommand extends InstantCommand {
  private ShooterRotSubsystem shooterRotSubsystem;
  private VisionSubsystem visionSubsystem;
  private DriveSubsystem driveSubsystem;
  private IntakeCommand intakeCommand;
  private final double BLUE_Y_SPEAKER = 0.0;
  private final double X_SPEAKER = 5.5;
  private final double RED_Y_SPEAKER = 16.5;
  public AutoAngleCommand(ShooterRotSubsystem shooterRot, VisionSubsystem vision, DriveSubsystem drive, IntakeCommand intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooterRotSubsystem = shooterRot;
    visionSubsystem = vision;
    driveSubsystem = drive;
    intakeCommand = intake;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!intakeCommand.isScheduled()){
      if(visionSubsystem.getAprilTagDistance()!=-1){
        shooterRotSubsystem.setGoal(MathUtil.clamp(shooterRotSubsystem.getEstimatedAngle(visionSubsystem.getAprilTagDistance()),(0-90)*Math.PI/180, (60-90)*Math.PI/180));
      }
      else{
        double X = X_SPEAKER - driveSubsystem.getPose().getX();
        double Y = driveSubsystem.getPose().getY();
        if(DriverStation.getAlliance().isPresent()){
          if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            Y = RED_Y_SPEAKER - Y;
            double distance = Math.sqrt(Math.pow(X, 2) + Math.pow(Y, 2));
            shooterRotSubsystem.setGoal(MathUtil.clamp(shooterRotSubsystem.getEstimatedAngle(distance),(0-90)*Math.PI/180, (60-90)*Math.PI/180));
          }
          else{
            Y = BLUE_Y_SPEAKER + Y;
            double distance = Math.sqrt(Math.pow(X, 2) + Math.pow(Y, 2));
            shooterRotSubsystem.setGoal(MathUtil.clamp(shooterRotSubsystem.getEstimatedAngle(distance),(0-90)*Math.PI/180, (60-90)*Math.PI/180));
          }
        }
        else{
          shooterRotSubsystem.setGoal((20-90) * Math.PI/180);
        }
      }
    }
  }
}
