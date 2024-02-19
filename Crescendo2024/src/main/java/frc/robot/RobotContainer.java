package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;
import frc.lib.Constants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {
    private DriveSubsystem driveSubsystem;

    private ClimberSubsystem climberSubsystem;

    private IntakeSubsystem intakeSubsystem;
    private VisionSubsystem visionSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private ShooterRotSubsystem shooterRotSubsystem;
    private ShooterFeederSubsystem shooterFeederSubsystem;

    private XboxController driverController;
    private XboxController operatorController;

    public RobotContainer(){
        driveSubsystem = new DriveSubsystem();
        visionSubsystem = new VisionSubsystem();
        /*climberSubsystem = new ClimberSubsystem();

        intakeSubsystem = new IntakeSubsystem();

        shooterSubsystem = new ShooterSubsystem();
        shooterRotSubsystem = new ShooterRotSubsystem();
        shooterFeederSubsystem = new ShooterFeederSubsystem();
*/
        driverController = new XboxController(0);
        operatorController = new XboxController(1);

        configureButtonBindings();
    }
    public void configureButtonBindings(){
        
    }

    public Command getAutonomousCommand(){
        return new PathPlannerAuto(DriveConstants.AUTO_NAME);
    }
}
