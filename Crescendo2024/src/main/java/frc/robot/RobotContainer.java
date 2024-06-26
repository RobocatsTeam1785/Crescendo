package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;
import frc.lib.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.ShootCommands.AmpShootCommand;
import frc.robot.commands.ShootCommands.CloseShootCommand;
import frc.robot.commands.ShootCommands.ShootCloseStage;
import frc.robot.commands.ShootCommands.ShootCommand;
import frc.robot.commands.ShootCommands.ShootTestTuner;
import frc.robot.commands.ShootCommands.ShootTrapCommand;
import frc.robot.commands.ShootCommands.ShootProtectedZone;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import frc.lib.Utils.*;
import edu.wpi.first.math.util.Units;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
    private DriveSubsystem driveSubsystem;

    private ClimberSubsystem climberSubsystem;

    private IntakeSubsystem intakeSubsystem;

    private ShooterSubsystem shooterSubsystem;

    private ShooterRotSubsystem shooterRotSubsystem;

    private ShooterFeederSubsystem shooterFeederSubsystem;

    private VisionSubsystem visionSubsystem;

    private AmpSubsystem ampSubsystem;

    private LEDSubsystem ledSubsystem;



    private XboxController driverController;
    private XboxController operatorController;




    private IntakeCommand intakeCommand;

    private ReverseIntake reverseIntake;

    private ExtendAmpSubsystem extendAmpSubsystem;

    private RetractAmpSubsystem retractAmpSubsystem;

    private HandleAmpCommand handleAmpCommand;
    
    private EjectNoteBackwards ejectNoteBackwards;

    private EjectNoteForward ejectNoteForwards;

    
    
    private ShootCommand shootCommand;

    private AmpShootCommand ampShootCommand;

    private ShootCloseStage shootCloseStage;
    
    private ShootProtectedZone shootProtectedZone;

    private BlinkLEDGreen blinkGreen;

    private AutoAngleCommand autoAngleCommand;

    private CloseShootCommand closeShootCommand;

    private AutoAlignCommand autoAlignCommand;
    private ShootTestCommand shootTestCommand;

    private ShootTestTuner shootTestTuner;

    private PulseLEDCommand pulseLEDCommand;

    private RevShooterCommand revShooterCommand;

    private AutoAngleWaitCommand autoAngleWaitCommand;

    private TurnShooterToSpeaker turnShooterToSpeaker;
    
    private IntakeWaitCommand intakeWaitCommand;

    private GriffinBadCommand griffinBadCommand;

    private Command backRobotCommand;

    private ShootTrapCommand shootTrapCommand;

    private AmpThrow ampThrow;

    private boolean fieldRelative = true;



    

    private double period = 0;  

    public RobotContainer(){

        driverController = new XboxController(0);
        operatorController = new XboxController(1);

        driveSubsystem = new DriveSubsystem();

        intakeSubsystem = new IntakeSubsystem();

        shooterFeederSubsystem = new ShooterFeederSubsystem();

        shooterRotSubsystem = new ShooterRotSubsystem();

        visionSubsystem = new VisionSubsystem();

        climberSubsystem = new ClimberSubsystem();

        shooterSubsystem = new ShooterSubsystem();

        ampSubsystem = new AmpSubsystem();

        ledSubsystem = new LEDSubsystem();





        intakeCommand = new IntakeCommand(intakeSubsystem, shooterFeederSubsystem, shooterRotSubsystem, ledSubsystem);
        

        reverseIntake = new ReverseIntake(intakeSubsystem, shooterFeederSubsystem, shooterRotSubsystem);

        extendAmpSubsystem = new ExtendAmpSubsystem(ampSubsystem);

        retractAmpSubsystem = new RetractAmpSubsystem(ampSubsystem);

        handleAmpCommand = new HandleAmpCommand(shooterRotSubsystem, extendAmpSubsystem, retractAmpSubsystem, ledSubsystem);

        shootCommand = new ShootCommand(shooterSubsystem, shooterFeederSubsystem, ledSubsystem);

        ampShootCommand = new AmpShootCommand(shooterSubsystem, shooterFeederSubsystem);

        shootCloseStage = new ShootCloseStage(shooterSubsystem, shooterFeederSubsystem, shooterRotSubsystem);

        shootProtectedZone = new ShootProtectedZone(shooterSubsystem, shooterFeederSubsystem, shooterRotSubsystem);

        ejectNoteBackwards = new EjectNoteBackwards(shooterSubsystem, shooterFeederSubsystem);
        
        ejectNoteForwards = new EjectNoteForward(shooterSubsystem, shooterFeederSubsystem);

        blinkGreen = new BlinkLEDGreen(ledSubsystem);

        autoAngleCommand = new AutoAngleCommand(shooterRotSubsystem, visionSubsystem, driveSubsystem, intakeCommand);

        closeShootCommand = new CloseShootCommand(shooterSubsystem, shooterFeederSubsystem, ledSubsystem);

        autoAlignCommand = new AutoAlignCommand(driveSubsystem, visionSubsystem);

        shootTestCommand = new ShootTestCommand(shooterSubsystem);

        shootTestTuner = new ShootTestTuner(shooterSubsystem);

        pulseLEDCommand = new PulseLEDCommand(ledSubsystem);

        revShooterCommand = new RevShooterCommand(shooterSubsystem);

        autoAngleWaitCommand = new AutoAngleWaitCommand(shooterRotSubsystem, visionSubsystem, driveSubsystem, intakeCommand);

        turnShooterToSpeaker = new TurnShooterToSpeaker(driveSubsystem);

        intakeWaitCommand = new IntakeWaitCommand(intakeSubsystem, shooterFeederSubsystem, shooterRotSubsystem, ledSubsystem);

        griffinBadCommand = new GriffinBadCommand(intakeSubsystem);

        backRobotCommand  = new PathPlannerAuto("Back Auto");

        shootTrapCommand = new ShootTrapCommand(shooterSubsystem, shooterFeederSubsystem, shooterRotSubsystem);

        ampThrow = new AmpThrow(ampSubsystem);


        configureButtonBindings();
        driveSubsystem.setDefaultCommand(new InstantCommand(() -> driveSubsystem.drive(
            driverController.getLeftX(),
            driverController.getLeftY(),
            -driverController.getRightX(),
            driverController.getRightY(),
            driverController.getLeftTriggerAxis(),
            driverController.getRightTriggerAxis(),
            driverController.getPOV(),
            visionSubsystem.hasSpeakerTarget() ? Util1785.getRobotRelativeAngle(visionSubsystem.getYaw(), Util1785.getDistanceRobotRelative(visionSubsystem.getYaw(), visionSubsystem.getAprilTagDistance(), Units.inchesToMeters(VisionConstants.FRONT_CAM_OFFSET)),Units.inchesToMeters(VisionConstants.FRONT_CAM_OFFSET)) : 400,
            //visionSubsystem.getYaw(),
            fieldRelative,
            period
        ), driveSubsystem));   

        intakeSubsystem.setDefaultCommand(new InstantCommand(() -> intakeSubsystem.handleIntake(
            MathUtil.applyDeadband(-operatorController.getLeftTriggerAxis(), 0.1)
        ), intakeSubsystem));     

        shooterFeederSubsystem.setDefaultCommand(new InstantCommand(() -> shooterFeederSubsystem.setVelocity(
            MathUtil.applyDeadband(operatorController.getRightTriggerAxis(), 0.1)
        ), shooterFeederSubsystem));    

        climberSubsystem.setDefaultCommand(new InstantCommand(() -> climberSubsystem.handleClimbers(
            operatorController.getLeftY(),
            operatorController.getRightY()
        ), climberSubsystem));

        shooterSubsystem.setDefaultCommand(new InstantCommand(() -> shooterSubsystem.setVelocity(0
        ), shooterSubsystem));

        ampSubsystem.setDefaultCommand(new InstantCommand(() -> ampSubsystem.setVoltage(0
        ),ampSubsystem));

        shooterFeederSubsystem.setDefaultCommand(new InstantCommand(() -> shooterFeederSubsystem.setVelocity(0), shooterFeederSubsystem));

        shooterRotSubsystem.enable();
        shooterRotSubsystem.setDefaultCommand(new InstantCommand(() -> setVarDistanceAngle(), shooterRotSubsystem));

        ledSubsystem.setDefaultCommand(new InstantCommand(() -> handleLEDS(), ledSubsystem));

        NamedCommands.registerCommand("ShootCommand", shootCommand);
        NamedCommands.registerCommand("IntakeCommand", intakeCommand);
        NamedCommands.registerCommand("AutoAngleCommand", autoAngleCommand);
        NamedCommands.registerCommand("CloseShootCommand", closeShootCommand);
        NamedCommands.registerCommand("AutoAlignCommand", autoAlignCommand);
        NamedCommands.registerCommand("ShootCloseStage", shootCloseStage);
        NamedCommands.registerCommand("IntakeWaitCommand", intakeWaitCommand);
        NamedCommands.registerCommand("TurnToSpeaker", turnShooterToSpeaker);
        NamedCommands.registerCommand("AutoAngleWaitCommand", autoAngleWaitCommand);

    }

    public void handleLEDS(){
        if(driverController.getRightTriggerAxis() > 0.6 && !intakeCommand.isScheduled() && !shootCommand.isScheduled() && !intakeWaitCommand.isScheduled() && !intakeWaitCommand.isScheduled()){
            if(!visionSubsystem.hasSpeakerTarget()){
                ledSubsystem.setRGB(127,0,0);
            }
            else if(Util1785.getRobotRelativeAngle(visionSubsystem.getYaw(), Util1785.getDistanceRobotRelative(visionSubsystem.getYaw(), visionSubsystem.getAprilTagDistance(), Units.inchesToMeters(VisionConstants.FRONT_CAM_OFFSET)),Units.inchesToMeters(VisionConstants.FRONT_CAM_OFFSET)) < 2){
                ledSubsystem.setRGB(0,125,255);
            }
            else{
                ledSubsystem.setRGB(125,125,125);
            }
        }
    }

    public void e(){
        SmartDashboard.putNumber("REV", shooterRotSubsystem.getTrueAngle());
    }

    public void scheduleBack(){
        if(backRobotCommand.isScheduled()){
            backRobotCommand.cancel();
        }
        else{
            backRobotCommand.schedule();
        }
    }

    public void toggleFieldRelative(){
        fieldRelative = !fieldRelative;
    }

    public void helpGriffin(){
        if(operatorController.getRightTriggerAxis()>0.6){
            if(!revShooterCommand.isScheduled() && !shootCommand.isScheduled()){
                revShooterCommand.schedule();
            }
        }
        else{
            if(revShooterCommand.isScheduled()){
                revShooterCommand.cancel();
            }
        }
        if(shootCommand.isScheduled()){
            revShooterCommand.cancel();
        }
    }

    public void configureButtonBindings(){
        /*
         * Buttons:
         * 
         * Translational Drive: Driver left joystick
         * Rotational Drive: Driver right joystick
         * Face stage: Driver left trigger
         * Force shot: Driver right trigger
         * Shoot: Driver right bumper
         * Intake: Driver left bumper
         * Zero gyro: Driver X
         * Set straight: Driver B
         * 
         * 
         * Climb left: Operator left joystick
         * Climb right: Operator right joystick
         * Toggle amp: Operator X
         * Eject forward: Operator Y
         * Eject backwards: Operator A
         * Reverse Intake: Operator B
         * Shoot close stage: Operator right bumper
         * Shoot protected zone: Operator right bumper
         * Turn off auto tracking: Operator left trigger
         * 
         * 
         */
        new JoystickButton(driverController, Button.kLeftBumper.value).onTrue(new InstantCommand(() -> toggleIntake()));
        new JoystickButton(driverController, Button.kRightBumper.value).onTrue(new InstantCommand(() -> toggleShoot()));
        new JoystickButton(driverController, Button.kX.value).onTrue(new InstantCommand(() -> resetGyro()));
        new JoystickButton(driverController, Button.kB.value).onTrue(new InstantCommand(() -> setStraight()));
        new JoystickButton(driverController, Button.kA.value).onTrue(new InstantCommand(() -> toggleFieldRelative()));
        new JoystickButton(driverController, Button.kY.value).whileTrue(griffinBadCommand);

        new JoystickButton(operatorController, Button.kX.value).onTrue(new InstantCommand(() -> toggleAmp()));
        new JoystickButton(operatorController, Button.kY.value).whileTrue(ejectNoteForwards);
        new JoystickButton(operatorController, Button.kA.value).whileTrue(ejectNoteBackwards);
        new JoystickButton(operatorController, Button.kB.value).whileTrue(reverseIntake);
        new JoystickButton(operatorController, Button.kBack.value).whileTrue(ampThrow);
        new JoystickButton(operatorController, Button.kRightBumper.value).onTrue(new InstantCommand(() -> toggleShootCloseSpeaker()));
        new JoystickButton(operatorController, Button.kLeftBumper.value).onTrue(new InstantCommand(() -> toggleTrapShoot()));
    }

    public void toggleIntake(){if(intakeCommand.isScheduled()){intakeCommand.cancel();}else{intakeCommand.schedule();}}
    public void toggleShoot(){
        if(shootCommand.isScheduled()){
            shootCommand.cancel();
        }
        else{
            shootCommand.schedule();
        }
    }

    public void toggleTrapShoot(){
        if(shootTrapCommand.isScheduled()){
            shootTrapCommand.cancel();
        }
        else{
            shootTrapCommand.schedule();
        }
    }
    public void toggleAlign(){if(autoAlignCommand.isScheduled()){autoAlignCommand.cancel();}else{autoAlignCommand.schedule();}}
    public void toggleAmp(){if(handleAmpCommand.isScheduled()){handleAmpCommand.cancel();}else{handleAmpCommand.schedule();}}
    public void toggleShootCloseSpeaker(){if(shootCloseStage.isScheduled()){shootCloseStage.cancel();}else{shootCloseStage.schedule();}}
    public void toggleShootProtectedZone(){if(shootProtectedZone.isScheduled()){shootProtectedZone.cancel();}else{shootProtectedZone.schedule();}}


    public void setStraight(){
        if(!intakeCommand.isScheduled() && !handleAmpCommand.isScheduled()){
            shooterRotSubsystem.setGoal((0-90)*Math.PI/180);
        }
    }

    public PulseLEDCommand getPulseLEDCommand(){
        return pulseLEDCommand;
    }

    public void setVarDistanceAngle(){
        if(visionSubsystem.getAprilTagDistance()!=-1 && operatorController.getLeftTriggerAxis() < 0.9){
        shooterRotSubsystem.setGoal(MathUtil.clamp(shooterRotSubsystem.getEstimatedAngle(Util1785.getDistanceRobotRelative(visionSubsystem.getYaw(), visionSubsystem.getAprilTagDistance(), Units.inchesToMeters(VisionConstants.FRONT_CAM_OFFSET))),(0-90)*Math.PI/180, (55-90)*Math.PI/180));}
    }

    public void resetGyro(){
        driveSubsystem.getGyro().reset();
    }

    public void setPeriod(double p){
        period = p;
        autoAlignCommand.setPeriod(period);
    }

    public void setGyroOffset(double offset){
        driveSubsystem.getGyro().setAngleAdjustment(0);
    }

    public void zeroSwerveModules(){
        driveSubsystem.zeroModules();
    }

    public Command getAutonomousCommand(){
        return new PathPlannerAuto(DriveConstants.AUTO_NAME);
    }
}
