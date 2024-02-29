// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.lib.Constants.DriveConstants;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose2d;
import javax.swing.text.DefaultStyledDocument.ElementSpec;

import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.Topic;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.I2C;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Swerve.SwerveModule;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.SerialPort;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import edu.wpi.first.wpilibj.DigitalInput;

public class DriveSubsystem extends SubsystemBase {
  private double speedMod = 1;

  private final SwerveModule m_frontLeft = new SwerveModule(
    DriveConstants.FRONT_LEFT_DRIVE_ID,
    DriveConstants.FRONT_LEFT_TURN_ID,
    DriveConstants.FRONT_LEFT_CANCODER_ID,
    DriveConstants.FRONT_LEFT_POS);
  private final SwerveModule m_frontRight = new SwerveModule(
    DriveConstants.FRONT_RIGHT_DRIVE_ID,
    DriveConstants.FRONT_RIGHT_TURN_ID,
    DriveConstants.FRONT_RIGHT_CANCODER_ID,
    DriveConstants.FRONT_RIGHT_POS);
  private final SwerveModule m_backLeft = new SwerveModule(
    DriveConstants.BACK_LEFT_DRIVE_ID,
    DriveConstants.BACK_LEFT_TURN_ID,
    DriveConstants.BACK_LEFT_CANCODER_ID,
    DriveConstants.BACK_LEFT_POS);
  private final SwerveModule m_backRight = new SwerveModule(
    DriveConstants.BACK_RIGHT_DRIVE_ID,
    DriveConstants.BACK_RIGHT_TURN_ID,
    DriveConstants.BACK_RIGHT_CANCODER_ID,
    DriveConstants.BACK_RIGHT_POS);

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable moduleStats = inst.getTable("Swerve");
    StructArrayPublisher<SwerveModuleState> publisher = moduleStats.getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
    StructArrayPublisher<SwerveModuleState> publisher2 = moduleStats.getStructArrayTopic("SetStates", SwerveModuleState.struct).publish();


  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(2.5);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(2.5);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(2.5);
  

  private final PIDController turnPID = new PIDController(
    DriveConstants.HEADING_KP,
    DriveConstants.HEADING_KI,
    DriveConstants.HEADING_KD);

  private final SwerveDrivePoseEstimator odometry;

  //private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);  
  private final AHRS m_gyro = new AHRS(I2C.Port.kOnboard);


  private final double coef = 1.0/(1-0.05);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          DriveConstants.FRONT_LEFT_POS,
          DriveConstants.FRONT_RIGHT_POS,
          DriveConstants.BACK_LEFT_POS,
          DriveConstants.BACK_RIGHT_POS
      );
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    //m_gyro.calibrate();
    m_gyro.reset();
    turnPID.enableContinuousInput(-180, 180);
    odometry = new SwerveDrivePoseEstimator(
      m_kinematics,
      Rotation2d.fromDegrees(-m_gyro.getAngle()),
      getModulePositions(),
      new Pose2d()
    );
    AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(DriveConstants.HEADING_KP, DriveConstants.HEADING_KI, DriveConstants.HEADING_KD), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.3556, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
  }

  public Pose2d getPose(){
    return odometry.getEstimatedPosition();
  }
  
  //Rotation2d.fromDegrees(-m_gyro.getAngle())

  //Rotation2d.fromDegrees(-m_gyro.getAngle())


  public void setPose(Pose2d pose){
    odometry.resetPosition(Rotation2d.fromDegrees(-m_gyro.getAngle()),
       new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          },
          pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return m_kinematics.toChassisSpeeds(
      new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
      }
    );
  }

  public void driveRobotRelative(ChassisSpeeds c){
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(c);
    setSwerveStates(swerveModuleStates);

  }

  public void drive(double leftX, double leftY, double rightX, double rightY, double leftTrigger, double rightTrigger, int FOV, boolean fieldRelative, double periodSeconds){
    double xSpeed =
    -m_xspeedLimiter.calculate(MathUtil.applyDeadband(leftY, 0.075))
        * speedMod * DriveConstants.TRANSLATIONAL_MAX_SPEED;
    double ySpeed =
    -m_yspeedLimiter.calculate(MathUtil.applyDeadband(leftX, 0.075))
        * speedMod *DriveConstants.TRANSLATIONAL_MAX_SPEED;
    double rot =
    m_rotLimiter.calculate(MathUtil.applyDeadband(rightX, 0.075))
        * speedMod *DriveConstants.ROTATIONAL_MAX_SPEED;

    if(rightTrigger>0.9){
      rot=-turnPID.calculate(m_gyro.getYaw(), 0);
    }
      
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
          ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, Rotation2d.fromDegrees(-m_gyro.getAngle()))
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                    periodSeconds
                ));
    
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.TRANSLATIONAL_MAX_SPEED);
    
    setSwerveStates(swerveModuleStates);

  }

  public void increaseSpeedMod(){
    speedMod+=0.1;
    if(speedMod>1){
      speedMod=1;
    }
  }
  public void decreasePowerMod(){
    speedMod-=0.1;
    if(speedMod<0.1){
      speedMod=0.1;
    }
  }
  public void setSwerveStates(SwerveModuleState[] states){
    publisher2.set(states);
    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_backLeft.setDesiredState(states[2]);
    m_backRight.setDesiredState(states[3]);
  }
  public AHRS getGyro(){
    return m_gyro;
  }
  
  public double normalizeAngle(double angle){
    if(Math.abs(angle)<=180){
      return angle;
    }
    else{
      if(angle>180){
        return angle-360;
      }
      else{
        return angle+360;
      }
    }
  }
  public PIDController getHeadingController(){
    return turnPID;
  }
  public SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        };
  }
  public SwerveModuleState[] getModuleStates(){
    return new SwerveModuleState[] {
          m_frontLeft.getState(),
          m_frontRight.getState(),
          m_backLeft.getState(),
          m_backRight.getState()
        };
  }
  public void updateOdometry(){
     odometry.update(
      Rotation2d.fromDegrees(-m_gyro.getAngle()), 
      getModulePositions()
      );
  }
  @Override
  public void periodic() {
    updateOdometry();

    publisher.set(getModuleStates());

    SmartDashboard.putNumber("Gyro Angle", m_gyro.getYaw());
    SmartDashboard.putNumber("Gyro Roll", m_gyro.getRoll());
    SmartDashboard.putNumber("Gyro Pitch", m_gyro.getPitch());
    SmartDashboard.putNumber("Gyro Rot2d", m_gyro.getRotation2d().getDegrees());

  }
}
