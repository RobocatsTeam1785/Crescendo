package frc.lib.Swerve;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.Constants.DriveConstants;
import com.revrobotics.RelativeEncoder;

public class SwerveModule {
    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;

    private CANcoder CANCoder;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder turnEncoder;

    private Translation2d position;

    private ProfiledPIDController turnPIDController;
    private PIDController drivePIDController;
    private SimpleMotorFeedforward turnFeedforward;
    private SimpleMotorFeedforward driveFeedforward;
    
    public SwerveModule(
        int DRIVEMOTORID,
        int TURNMOTORID,
        int CANCODERID,
        Translation2d pos
    ){
        driveMotor = new CANSparkMax(DRIVEMOTORID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(TURNMOTORID, MotorType.kBrushless);

        driveMotor.setSmartCurrentLimit(45);
        turnMotor.setSmartCurrentLimit(40);

        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        turnMotor.setInverted(true);
        
        driveMotor.setInverted(true);

        CANCoder = new CANcoder(CANCODERID);

        double p = 360-360*CANCoder.getAbsolutePosition().getValueAsDouble();
        p=p%360;
        if(p>180){
            p=p-360;
        }
        else if(p<-180){
            p=p+360;
        }
        p=-p;

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(DriveConstants.DRIVE_CONVERSION_FACTOR);
        driveEncoder.setVelocityConversionFactor(DriveConstants.DRIVE_CONVERSION_FACTOR/60);

        if(DRIVEMOTORID == 1){
            driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        }

        turnEncoder = turnMotor.getEncoder();
        turnEncoder.setPosition(p*Math.PI/180);
        turnEncoder.setPositionConversionFactor(DriveConstants.TURN_CONVERSION_FACTOR);
        turnEncoder.setVelocityConversionFactor(DriveConstants.TURN_CONVERSION_FACTOR/60);

        drivePIDController = new PIDController(
            DriveConstants.TRANSLATIONAL_KP,
            DriveConstants.TRANSLATIONAL_KI,
            DriveConstants.TRANSLATIONAL_KD
        );

        turnPIDController = new ProfiledPIDController(
            DriveConstants.ROTATIONAL_KP,
            DriveConstants.ROTATIONAL_KI,
            DriveConstants.ROTATIONAL_KD,
            new TrapezoidProfile.Constraints(
                9999,
                9999
            )
        );
        turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

        driveFeedforward = new SimpleMotorFeedforward(
            DriveConstants.TRANSLATIONAL_KS,
            DriveConstants.TRANSLATIONAL_KV,
            DriveConstants.TRANSLATIONAL_KA
        );

        turnFeedforward = new SimpleMotorFeedforward(
            DriveConstants.ROTATIONAL_KS,
            DriveConstants.ROTATIONAL_KV,
            DriveConstants.ROTATIONAL_KA
        );

        position = pos;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity(), new Rotation2d(turnEncoder.getPosition()));
      }
    
    public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition(), new Rotation2d(turnEncoder.getPosition()));
    }

    public void zeroModule(){
        double p = 360-360*CANCoder.getAbsolutePosition().getValueAsDouble();
        p=p%360;
        if(p>180){
            p=p-360;
        }
        else if(p<-180){
            p=p+360;
        }
        p=-p;
        turnEncoder.setPosition(p*Math.PI/180);
    }


    public void setDesiredState(SwerveModuleState state){
    //state = new SwerveModuleState(state.speedMetersPerSecond, Rotation2d.fromRadians(-state.angle.getRadians()));
    var encoderRotation = new Rotation2d(turnEncoder.getPosition());
    
    state = SwerveModuleState.optimize(state, encoderRotation);

    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    final double driveOutput =
        drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);

    final double driveFeed = driveFeedforward.calculate(state.speedMetersPerSecond);

    /*double pos = 360-360*CANCoder.getAbsolutePosition().getValueAsDouble();
    pos=pos%360;
    if(pos>180){
        pos=pos-360;
    }
    else if(pos<-180){
        pos=pos+360;
    }*/
    


    final double turnOutput =
        turnPIDController.calculate(turnEncoder.getPosition(), state.angle.getRadians());

    final double turnFeed =
        turnFeedforward.calculate(turnPIDController.getSetpoint().velocity);
    driveMotor.setVoltage(driveOutput + driveFeed);
    
    turnMotor.setVoltage(turnOutput + turnFeed);
    }

    public CANSparkMax getDriveMotor(){
        return driveMotor;
      }
    public CANSparkMax getTurnMotor(){
    return turnMotor;
    }
    public RelativeEncoder getDriveEncoder(){
    return driveEncoder;
    }
    public RelativeEncoder getTurnEncoder(){
    return turnEncoder;
    }
    public CANcoder getCANCoder(){
        return CANCoder;
    }
}
