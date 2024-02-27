package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;
import frc.lib.Constants.*;
import frc.robot.commands.*;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;

public class RobotContainer {
    private DriveSubsystem driveSubsystem;

    private ClimberSubsystem climberSubsystem;

    private IntakeSubsystem intakeSubsystem;
    private IntakeSubsystem2 intakeSubsystem2;

    private ShooterSubsystem shooterSubsystem;
    private ShooterRotSubsystem shooterRotSubsystem;
    private ShooterFeederSubsystem shooterFeederSubsystem;

    private VisionSubsystem visionSubsystem;


    private XboxController driverController;
    private XboxController operatorController;

    private IntakeCommand intakeCommand;
    private ShootCommand shootCommand;
    private ReverseIntake reverseIntake;

    private double period = 0;  
    private double RPM = 0;

    public RobotContainer(){
        driveSubsystem = new DriveSubsystem();

        intakeSubsystem = new IntakeSubsystem();

        shooterFeederSubsystem = new ShooterFeederSubsystem();

        shooterRotSubsystem = new ShooterRotSubsystem();

        //visionSubsystem = new VisionSubsystem();

        climberSubsystem = new ClimberSubsystem();

        shooterSubsystem = new ShooterSubsystem();

        intakeCommand = new IntakeCommand(intakeSubsystem, shooterFeederSubsystem, shooterRotSubsystem);
        shootCommand = new ShootCommand(shooterSubsystem, shooterFeederSubsystem);
        reverseIntake = new ReverseIntake(intakeSubsystem, shooterFeederSubsystem, shooterRotSubsystem);
        /*
        intakeSubsystem = new IntakeSubsystem();

        shooterSubsystem = new ShooterSubsystem();
        shooterRotSubsystem = new ShooterRotSubsystem();
        shooterFeederSubsystem = new ShooterFeederSubsystem();*/

        driverController = new XboxController(0);
        operatorController = new XboxController(1);


        configureButtonBindings();
        driveSubsystem.setDefaultCommand(new InstantCommand(() -> driveSubsystem.drive(
            driverController.getLeftX(),
            driverController.getLeftY(),
            -driverController.getRightX(),
            driverController.getRightY(),
            0,
            0,
            driverController.getPOV(),
            true,
            period
        ), driveSubsystem));   
        intakeSubsystem.setDefaultCommand(new InstantCommand(() -> intakeSubsystem.handleIntake(
            MathUtil.applyDeadband(-operatorController.getLeftTriggerAxis(), 0.1)
        ), intakeSubsystem));     
        shooterFeederSubsystem.setDefaultCommand(new InstantCommand(() -> shooterFeederSubsystem.setVelocity(
            MathUtil.applyDeadband(operatorController.getRightTriggerAxis(), 0.1)
        ), shooterFeederSubsystem));    
        shooterRotSubsystem.setGoal(-Math.PI/2);
        shooterRotSubsystem.enable();
        climberSubsystem.setDefaultCommand(new InstantCommand(() -> climberSubsystem.handleClimbers(
            operatorController.getLeftY(),
            operatorController.getRightY()
        ), climberSubsystem)); 
        shooterSubsystem.setDefaultCommand(new InstantCommand(() -> shooterSubsystem.setVelocity(RPM
        ), shooterSubsystem));
        //shooterFeederSubsystem.setDefaultCommand(new InstantCommand(() -> shooterFeederSubsystem.setVelocity()));
    }

    public void e(){
        SmartDashboard.putNumber("REV", shooterRotSubsystem.getTrueAngle());
    }

    public void configureButtonBindings(){
        new JoystickButton(driverController, Button.kX.value).onTrue(new InstantCommand(() -> resetGyro()));
        new JoystickButton(driverController, Button.kY.value).onTrue(new InstantCommand(() -> setStraight()));
        new JoystickButton(driverController, Button.kA.value).onTrue(new InstantCommand(() -> toggleReverse()));
        new JoystickButton(driverController, Button.kB.value).onTrue(new InstantCommand(() -> set452()));
        new JoystickButton(driverController, Button.kRightBumper.value).onTrue(new InstantCommand(() -> toggleShoot()));    
        new JoystickButton(driverController, Button.kLeftBumper.value).onTrue(new InstantCommand(() -> toggleIntake()));
    }


    public void setPeriod(double p){
        period = p;
    }

    public void setStraight(){
        shooterRotSubsystem.setGoal(-Math.PI/2);
    }

    public void set45(){
        shooterRotSubsystem.setGoal(28*Math.PI/180);
    }
    
    public void set452(){
        shooterRotSubsystem.setGoal((30-90)*Math.PI/180);
    }

    public void set4523(){
        shooterRotSubsystem.setGoal((45-90)*Math.PI/180);
    }

    public void toggleIntake(){
        if(intakeCommand.isScheduled()){
            intakeCommand.cancel();
        }
        else{
            intakeCommand.schedule();
        }
    }

    public void toggleReverse(){
        if(reverseIntake.isScheduled()){
            reverseIntake.cancel();
        }
        else{
            reverseIntake.schedule();
        }
    }

    public void toggleShoot(){
        if(shootCommand.isScheduled()){
            shootCommand.cancel();
        }
        else{
            shootCommand.schedule();
        }
    }

    public void resetGyro(){
        driveSubsystem.getGyro().reset();

      }
    public void toggleRPM(){
        if(RPM<1){
            RPM=3000/60;
        }
        else{
            RPM=0;
        }
    }

    public Command getAutonomousCommand(){
        return new PathPlannerAuto(DriveConstants.AUTO_NAME);
    }
}
