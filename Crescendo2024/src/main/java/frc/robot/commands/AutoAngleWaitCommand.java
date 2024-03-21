// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterRotSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoAngleWaitCommand extends Command {

  private final double MARGIN_OF_ERROR = (2-90)*Math.PI/180;
  private Timer timer = new Timer();
  private boolean done;
  private ShooterRotSubsystem shooterRotSubsystem;
  private VisionSubsystem visionSubsystem;
  private DriveSubsystem driveSubsystem;
  private IntakeCommand intakeCommand;
  private final double BLUE_X_SPEAKER = 0.0;
  private final double Y_SPEAKER = 5.5;
  private final double RED_X_SPEAKER = 16.5;
  /** Creates a new AutoAngleWaitCommand. */
  public AutoAngleWaitCommand(ShooterRotSubsystem shooterRot, VisionSubsystem vision, DriveSubsystem drive, IntakeCommand intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooterRotSubsystem = shooterRot;
    visionSubsystem = vision;
    driveSubsystem = drive;
    intakeCommand = intake;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("AAW Start");

    done=false;
    if(!intakeCommand.isScheduled()){
      timer.stop();
      timer.reset();
      timer.start();
      if(visionSubsystem.getAprilTagDistance()!=-1){
        shooterRotSubsystem.setGoal(MathUtil.clamp(shooterRotSubsystem.getEstimatedAngle(visionSubsystem.getAprilTagDistance()),(0-90)*Math.PI/180, (55-90)*Math.PI/180));
      }
      else{
        double X = driveSubsystem.getPose().getX();
        double Y = Y_SPEAKER - driveSubsystem.getPose().getY();
        if(DriverStation.getAlliance().isPresent()){
          if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            X = RED_X_SPEAKER - X;
            double distance = Math.sqrt(Math.pow(X, 2) + Math.pow(Y, 2));
            shooterRotSubsystem.setGoal(MathUtil.clamp(shooterRotSubsystem.getEstimatedAngle(distance),(0-90)*Math.PI/180, (58-90)*Math.PI/180));
          }
          else{
            X = BLUE_X_SPEAKER + X;
            double distance = Math.sqrt(Math.pow(X, 2) + Math.pow(Y, 2));
            shooterRotSubsystem.setGoal(MathUtil.clamp(shooterRotSubsystem.getEstimatedAngle(distance),(0-90)*Math.PI/180, (58-90)*Math.PI/180));
          }
        }
        else{
          shooterRotSubsystem.setGoal((55-90) * Math.PI/180);
        }
      }
    }
    else{
      done=true;
      this.cancel();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(Math.abs(shooterRotSubsystem.getController().getGoal().position) - Math.abs(shooterRotSubsystem.getMeasurement()) ) < 0.5 * Math.PI/180 || timer.hasElapsed(1.5)){
      done=true;
      this.cancel();
    }
    else{
      SmartDashboard.putNumber("Num 1", Math.abs(shooterRotSubsystem.getController().getGoal().position));
      SmartDashboard.putNumber("Num 2", Math.abs(shooterRotSubsystem.getMeasurement()));
      SmartDashboard.putNumber("Diff", Math.abs(Math.abs(shooterRotSubsystem.getController().getGoal().position) - Math.abs(shooterRotSubsystem.getMeasurement()) ));
      SmartDashboard.putNumber("Error", MARGIN_OF_ERROR);
      SmartDashboard.putBoolean("IsGood",Math.abs(Math.abs(shooterRotSubsystem.getController().getGoal().position) - Math.abs(shooterRotSubsystem.getMeasurement()) ) < MARGIN_OF_ERROR );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("AAW End");

    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
