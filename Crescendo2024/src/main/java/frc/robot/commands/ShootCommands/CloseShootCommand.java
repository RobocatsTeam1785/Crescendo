package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Constants.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CloseShootCommand extends Command {
  private ShooterSubsystem shooterSubsystem;
  private ShooterFeederSubsystem shooterFeederSubsystem;
  private LEDSubsystem ledSubsystem;
  private Timer timer;
  private Timer timer2;
  private boolean feeding;
  private double RPM;
  private boolean done;
  /** Creates a new ShootCommand. */
  public CloseShootCommand(ShooterSubsystem shooter, ShooterFeederSubsystem shooterFeeder, LEDSubsystem led) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, shooterFeeder);
    ledSubsystem = led;
    shooterSubsystem = shooter;
    shooterFeederSubsystem = shooterFeeder;
    timer = new Timer();
    timer2 = new Timer();
    RPM = 0;
    done = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!shooterFeederSubsystem.getPhotoSensor()){
      this.cancel();
      feeding=false;
      timer.stop();
      timer.reset();
      done=true;
    }
    else{
      if(!ShooterSubsystem.isAmping){
        ledSubsystem.purple();
      }
      timer2.stop();
      timer2.reset();
      timer2.start();

      done=false;
      RPM = 3000/60;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setVelocity(RPM);
    if(Math.abs(Math.abs(shooterSubsystem.getTopVelocity())-RPM)<ShooterConstants.MARGIN_OF_ERROR &&
        Math.abs(Math.abs(shooterSubsystem.getBottomVelocity())-RPM)<ShooterConstants.MARGIN_OF_ERROR &&
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
        this.cancel();
        done = true;
      }
      else{
        shooterFeederSubsystem.setVelocity(0.5);
      }
    }

    else{
      shooterFeederSubsystem.setVelocity(0);
    }

    SmartDashboard.putNumber("Timer time", timer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    timer2.stop();
    timer2.reset();
    feeding=false;
    shooterSubsystem.setVelocity(0);
    shooterFeederSubsystem.setVelocity(0);
    if(!ShooterSubsystem.isAmping){
      ledSubsystem.red();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}