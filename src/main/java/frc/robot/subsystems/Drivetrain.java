// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.revrobotics.*;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CHASSIS;
import frc.robot.Constants.DriveMode;

public class Drivetrain extends SubsystemBase {
  private SparkMax frontLeftMotor;
  private SparkMax frontRightMotor;

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private final DifferentialDrive drive;

  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  private final DifferentialDriveOdometry odometry;

  private DriveMode driveMode = DriveMode.MANUAL;

  private EncoderConfig leftEncoderConfig;
  private EncoderConfig rightEncoderConfig;

  ResetMode resetMode;
  PersistMode persistMode;

  SparkBaseConfig motorConfigLeft;
  SparkBaseConfig motorConfigRight;

  /** Creates a new Drive subsystem. */
  public Drivetrain() {
    // Init the motors
    this.frontLeftMotor = new SparkMax(CHASSIS.FRONT_LEFT_ID, MotorType.kBrushless);
    this.frontRightMotor = new SparkMax(CHASSIS.FRONT_RIGHT_ID, MotorType.kBrushless);

    motorConfigLeft.inverted(CHASSIS.INVERTED);
    motorConfigRight.inverted(!CHASSIS.INVERTED);

    motorConfigLeft.smartCurrentLimit(60, 20);
    motorConfigRight.smartCurrentLimit(60, 20);

    motorConfigLeft.openLoopRampRate(CHASSIS.RAMP_RATE);
    motorConfigRight.openLoopRampRate(CHASSIS.RAMP_RATE);

    leftEncoderConfig.positionConversionFactor(CHASSIS.LEFT_POSITION_CONVERSION);
    rightEncoderConfig.positionConversionFactor(CHASSIS.RIGHT_POSITION_CONVERSION);

    leftEncoderConfig.velocityConversionFactor(CHASSIS.LEFT_VELOCITY_CONVERSION);
    rightEncoderConfig.velocityConversionFactor(CHASSIS.RIGHT_VELOCITY_CONVERSION);

    motorConfigLeft.apply(leftEncoderConfig);
    motorConfigRight.apply(rightEncoderConfig);

    this.frontLeftMotor.configure(motorConfigLeft, resetMode, persistMode);
    this.frontRightMotor.configure(motorConfigRight, resetMode, persistMode);

    this.leftEncoder = this.frontLeftMotor.getEncoder();
    this.rightEncoder = this.frontRightMotor.getEncoder();

    this.odometry = new DifferentialDriveOdometry(
      m_gyro.getRotation2d(), getLeftPositionMeters(), getRightPositionMeters());
    

    this.drive = new DifferentialDrive(this.frontLeftMotor, this.frontRightMotor);

    // AutoBuilder.configureLTV(
    //         this::getPose, // Robot pose supplier
    //         this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
    //         this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //         this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
    //         0.02, // PPLTVController is the built in path following controller for differential drive trains
    //         new ReplanningConfig(), // The robot configuration
    //         () -> {
    //           // Boolean supplier that controls when the path will be mirrored for the red alliance
    //           // This will flip the path being followed to the red side of the field.
    //           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //           var alliance = DriverStation.getAlliance();
    //           if (alliance.isPresent()) {
    //             return alliance.get() == DriverStation.Alliance.Red;
    //           }
    //           return false;
    //         },
    //         this // Reference to this subsystem to set requirements
    // );
  }

  // Very important drive mode functions that make everything work
  public DriveMode getDriveMode() {
    return driveMode;
  }

  public void setDriveMode(DriveMode modeToSet) {
    driveMode = modeToSet;
  }

  public void setDeadband(double deadband) {
    this.drive.setDeadband(deadband);
  }

  public void configureMotors() {
    frontLeftMotor.configure(motorConfigLeft, resetMode, persistMode);
    frontRightMotor.configure(motorConfigRight, resetMode, persistMode);
  }

  /**
   * Command to turn the robot to a heading
   */
  public Command alignToHeadingCommand(DoubleSupplier current, DoubleSupplier desired) {
    return runOnce(() -> {
      this.arcadeDrive(0, Math.min((Rotation2d.fromDegrees(desired.getAsDouble()).minus(Rotation2d.fromDegrees(current.getAsDouble())).getDegrees()) * 0.02, 0.1));
    })
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        .withName("headingAlign");
  }

  public double getDesiredHeading() {
    return 90;
  }

  /**
   * sets the chassis brake mode
   */
  public void setBrakeMode(IdleMode idleMode) {
    motorConfigLeft.idleMode(idleMode);
    motorConfigRight.idleMode(idleMode);

    configureMotors();
  }

  public Command releaseBrakeMode() {
    return runOnce(() -> this.setBrakeMode(IdleMode.kCoast));
  }
  public Command enableBrakeMode() {
    return runOnce(() -> this.setBrakeMode(IdleMode.kBrake));
  }

  /**
   * Returns a command that stops the drivetrain its tracks.
   */
  public Command emergencyStop() {
    return startEnd(() -> {
      this.frontLeftMotor.disable();
      this.frontRightMotor.disable();
    }, this::releaseBrakeMode)
        .beforeStarting(this::enableBrakeMode)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  /**
   * Basically like e-stop command for disabled mode only
   */
  public void killSwitch() {
    this.frontLeftMotor.disable();
    this.frontRightMotor.disable();
    this.setBrakeMode(IdleMode.kBrake);
  }

  /**
   * Sets motor output using arcade drive controls
   * 
   * @param forward linear motion [-1 --> 1] (Backwards --> Forwards)
   * @param rot     rotational motion [-1 --> 1] (Left --> Right)
   */
  public void arcadeDrive(double forward, double rot) {
    // Decreasing the drive command for safety
    this.drive.arcadeDrive(forward, rot);
  }

  /**
   * Sets the drivetrain's maximum percent output
   * 
   * @param maxOutput in percent decimal
   */
  public void setMaxOutput(double maxOutput) {
    this.drive.setMaxOutput(maxOutput);
  }

  /**
   * Returns the drivetrain's left encoder position in inches and meters respectively
   * 
   * @return Left Position
   */
  private double getLeftPositionInches() {
    return this.leftEncoder.getPosition();
  }
  private double getLeftPositionMeters() {
    return this.leftEncoder.getPosition() * 0.0254;
  }

  /**
   * Returns the drivetrain's right encoder position in inches and meters respectively
   * 
   * @return Right Position
   */
  private double getRightPositionInches() {
    return this.rightEncoder.getPosition();
  }
  private double getRightPositionMeters() {
    return this.rightEncoder.getPosition() * 0.0254;
  }
  
  /**
   * Returns the drivetrain's average encoder position in inches
   * 
   * @return Average Position
   */
  public double getAveragePositionInches() {
    return (this.getLeftPositionInches() + this.getRightPositionInches()) / 2;
  }

  /**
   * Returns the drivetrain's left encoder velocityin inches and meters per second respectively
   * 
   * @return Left Velocity
   */
  private double getLeftVelocityInches() {
    return this.leftEncoder.getVelocity();
  }
  private double getLeftVelocityMeters() {
    return this.leftEncoder.getVelocity() * 0.0254;
  }

  /**
   * Returns the drivetrain's right encoder velocity in inches and meters per second respectively
   * 
   * @return Right Velocity
   */
  private double getRightVelocityInches() {
    return this.rightEncoder.getVelocity();
  }
  private double getRightVelocityMeters() {
    return this.rightEncoder.getVelocity() * 0.0254;
  }

  /**
   * Returns the drivetrain's average encoder velocty in inches and meters per second respectively
   * 
   * @return Average Velocity
   */
  public double getAverageVelocityInches() {
    return (this.getLeftVelocityInches() + this.getRightVelocityInches()) / 2;
  }
  public double getAverageVelocityMeters() {
    return (this.getLeftVelocityMeters() + this.getRightVelocityMeters()) / 2;
  }

  /**
   * Resest all drivetrain encoder positions
   */
  public void resetEncoders() {
    this.leftEncoder.setPosition(0);
    this.rightEncoder.setPosition(0);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void setVoltageOutput(double leftVolts, double rightVolts) {
    this.frontLeftMotor.setVoltage(leftVolts);
    this.frontRightMotor.setVoltage(rightVolts);
    this.drive.feed();
  }

   /**
   * Resest drivetrain gyro position
   */
  public void resetGyro() {
    this.m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in DEGREES, from -180 to 180
   */
  public double getHeadingDeg() {
    return this.m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in DEGREES per second
   */
  public double getTurnRateDeg() {
    return -this.m_gyro.getRate();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in RADIANS per second
   */
  public double getTurnRateRad() {
    return -this.m_gyro.getRate() * (Math.PI / 180);
  }

  /**
   * Returns a ChassisSpeeds class with robot data (used for pathplanner).
   *
   * @return some data in the form of ChassisSpeeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return new ChassisSpeeds(getAverageVelocityMeters(), 0, getTurnRateRad());
  }

  /* This function drives the robot using arcadeDrive, which would normally use values from -1 to 1.
   * We're instead passing a m/s value, which might be causing some inaccuracies.
   * They might also be PID problems.
   */
  public void setChassisSpeeds(ChassisSpeeds speed, DriveFeedforwards test) {
    double linearSpeed = speed.vxMetersPerSecond;
    double rotSpeed = speed.omegaRadiansPerSecond;
    //dividing by 3.4 because that's more or less the maximum speed of the robot
    arcadeDrive(Math.min(linearSpeed / 3.4, 0.35), Math.min(rotSpeed, 0.35));
  }

  /* Gets the position of the robot via pose2d
  */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /* Gets the position of the robot via pose2d
  */
  public Pose2d getDesiredPose() {
    return new Pose2d(-1, 0, Rotation2d.fromDegrees(0));
  }

  /* Resets the position of the robot via pose2d
  */
  public void resetPose(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(
      m_gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), pose);
  }

  /* Resets the position of the robot to 0, 0
  */
  public void resetFieldPosition() {
    resetEncoders();
    odometry.resetPosition(new Rotation2d(0, 0), 0, 0, new Pose2d(new Translation2d(0, 0), new Rotation2d(0, 0)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.drive.feed();

    odometry.update(
      m_gyro.getRotation2d(), getLeftPositionMeters(), getRightPositionMeters());

    printToDashBoard();
    //System.out.println(getHeadingDeg());
  }

  /* Prints all values to the dashboard.
  */
  public void printToDashBoard() {
    SmartDashboard.putNumber("Pitch", this.m_gyro.getPitch());
    SmartDashboard.putNumber("Left Encoder Position", this.getLeftPositionMeters());
    SmartDashboard.putNumber("Right Encoder Position", this.getRightPositionMeters());
  
    SmartDashboard.putNumber("Left Encoder Velocity", this.getLeftVelocityMeters());
    SmartDashboard.putNumber("Right Encoder Velocity", this.getRightVelocityMeters());

    SmartDashboard.putNumber("Forward Velocity", this.getAverageVelocityMeters());
    SmartDashboard.putNumber("Rotational Velocity", this.getTurnRateRad());

    SmartDashboard.putNumber("Heading", this.getHeadingDeg());

    SmartDashboard.putNumber("X Position", this.getPose().getX());
    SmartDashboard.putNumber("Y Position", this.getPose().getY());
  }
}