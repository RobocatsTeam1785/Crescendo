package frc.robot;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.XboxController.Button;
public class RobotContainer {
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    private final XboxController driverController = new XboxController(0);
    private final XboxController operatorController = new XboxController(1);

    public RobotContainer(){
        climbSubsystem.setDefaultCommand(new InstantCommand(() -> climbSubsystem.handleClimbers(operatorController.getLeftY(), operatorController.getRightY()),climbSubsystem));
        driveSubsystem.setDefaultCommand(new InstantCommand(() -> driveSubsystem.handleDrive(driverController.getLeftY(), driverController.getLeftX()),driveSubsystem));
        shooterSubsystem.setDefaultCommand(new InstantCommand(() -> shooterSubsystem.handleMotors(driverController.getRightTriggerAxis(), driverController.getRightTriggerAxis(),operatorController.getLeftTriggerAxis(),driverController.getRightY()),shooterSubsystem));
        intakeSubsystem.setDefaultCommand(new InstantCommand(() -> intakeSubsystem.handleMotors(driverController.getLeftTriggerAxis(), driverController.getLeftTriggerAxis()),intakeSubsystem));

    }
    public void configureButtonBindings(){
        new JoystickButton(driverController, Button.kX.value).onTrue(new InstantCommand(() -> driveSubsystem.toggleMode()));
    }
}
