package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;
import frc.lib.Constants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController.Button;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;

public class RobotContainer {
    private DriveSubsystem driveSubsystem;

    private ClimberSubsystem climberSubsystem;

    private IntakeSubsystem intakeSubsystem;
    private VisionSubsystem visionSubsystem;
    private IntakeSubsystem2 intakeSubsystem2;

    private ShooterSubsystem shooterSubsystem;
    private ShooterRotSubsystem shooterRotSubsystem;
    private ShooterFeederSubsystem shooterFeederSubsystem;

    private XboxController driverController;
    private XboxController operatorController;

    private double period = 0;

    public RobotContainer() {
        driveSubsystem = new DriveSubsystem();
        visionSubsystem = new VisionSubsystem();

        intakeSubsystem = new IntakeSubsystem();

        shooterFeederSubsystem = new ShooterFeederSubsystem();

        // shooterRotSubsystem = new ShooterRotSubsystem();

        climberSubsystem = new ClimberSubsystem();

        intakeSubsystem = new IntakeSubsystem();

        shooterSubsystem = new ShooterSubsystem();
        shooterRotSubsystem = new ShooterRotSubsystem();

        shooterFeederSubsystem = new ShooterFeederSubsystem();

        driverController = new XboxController(0);
        operatorController = new XboxController(1);

        configureButtonBindings();
        driveSubsystem.setDefaultCommand(new InstantCommand(() -> driveSubsystem.drive(
                driverController.getLeftX(),
                driverController.getLeftY(),
                -driverController.getRightX(),
                driverController.getRightY(),
                driverController.getLeftTriggerAxis(),
                driverController.getRightTriggerAxis(),
                driverController.getPOV(),
                true,
                period), driveSubsystem));
        intakeSubsystem.setDefaultCommand(new InstantCommand(() -> intakeSubsystem.handleIntake(
                MathUtil.applyDeadband(operatorController.getLeftY(), 0.1)), intakeSubsystem));
        // shooterFeederSubsystem.setDefaultCommand(new InstantCommand(() ->
        // shooterFeederSubsystem.setVelocity()));
    }

    public void configureButtonBindings() {
        new JoystickButton(driverController, Button.kX.value).onTrue(new InstantCommand(() -> resetGyro()));
        new JoystickButton(driverController, Button.kY.value).onTrue(new InstantCommand(() -> setStraight()));
        new JoystickButton(driverController, Button.kA.value).onTrue(new InstantCommand(() -> set45()));
        new JoystickButton(driverController, Button.kB.value).onTrue(new InstantCommand(() -> set452()));
        new JoystickButton(driverController, Button.kRightBumper.value).onTrue(new InstantCommand(() -> set4523()));
    }

    public void setPeriod(double p) {
        period = p;
    }

    public void setStraight() {
        shooterRotSubsystem.setGoal(0);
    }

    public void set45() {
        shooterRotSubsystem.setGoal(Math.PI / 4);
    }

    public void set452() {
        shooterRotSubsystem.setGoal(Math.PI / 8);
    }

    public void set4523() {
        shooterRotSubsystem.setGoal(3 * Math.PI / 8);
    }

    public void resetGyro() {
        driveSubsystem.getGyro().reset();

    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto(DriveConstants.AUTO_NAME);
    }
}
