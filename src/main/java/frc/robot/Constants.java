// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or5 share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.util.control.SparkMaxConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Ways to drive the robot
  public static enum DriveMode {
    MANUAL,
    HEADINGLOCK,
    AUTODRIVE,
    POINTLOCK;
  }
  public static final class AUTO {
    public static RobotConfig robotConfig = new RobotConfig(
      Constants.AUTO.mass, 
      Constants.AUTO.moi, 
      new ModuleConfig(
        Constants.AUTO.wheelRadius, 
      Constants.AUTO.maxDriveVelocity, 
      Constants.AUTO.wheelCOF, 
      Constants.AUTO.driveMotor, 
      1, 
      Constants.AUTO.numMotors),
      Constants.AUTO.trackwidth);

    public static DCMotor driveMotor;

    public static double wheelRadius = 4;
    public static double maxDriveVelocity = 30;
    public static double wheelCOF = 1;
    public static int numMotors = 2;

    public static double mass = 30;
    public static double trackwidth = 30;
    public static double moi = 30;
  }
  public static final class CHASSIS {
    // THESE CAN IDs are for JERRY
    public static final int FRONT_LEFT_ID = 13;
    public static final int FRONT_RIGHT_ID = 11;
    //public static final int BACK_LEFT_ID = 12;
    //public static final int BACK_RIGHT_ID = 13;

    // Drivetrain restrictions
    public static final double DEFAULT_OUTPUT = 1; // THIS MUST REMAIN 1 FOR AUTOS
    public static final double MAX_INTERVAL = 1 - DEFAULT_OUTPUT;
    public static final double MIN_INTERVAL = DEFAULT_OUTPUT - 0.2;
    public static final double RAMP_RATE = 0.15; // s
    public static final double TOLERANCE = 1; // in
    public static final boolean INVERTED = true;

    // Chassis dimensions needed
    // THESE ARE FOR JERRY
    public static final double WHEEL_DIAMETER = 4;
    public static final double GEAR_RATIO = 7.2;
    public static final double TRACK_WIDTH = 12;

    // Chasis conversion factors TODO: Re-collect conversion data
    public static final double LEFT_POSITION_CONVERSION = 48 / 18.23804473876953; // inches / revs
    public static final double RIGHT_POSITION_CONVERSION = 48 / 18.14280891418457; // inches / revs
    public static final double POSITION_CONVERSION = (WHEEL_DIAMETER * Math.PI) / GEAR_RATIO;

    public static final double LEFT_VELOCITY_CONVERSION = LEFT_POSITION_CONVERSION / 60.0; // inches / s
    public static final double RIGHT_VELOCITY_CONVERSION = RIGHT_POSITION_CONVERSION / 60.0; // inches / s
    public static final double VELOCITY_CONVERSION = POSITION_CONVERSION / 60;

    // input diameter = Δd inches between center wheels ~~v~~ in inches / degree
    public static final double TURNING_CONVERSION = ((TRACK_WIDTH * Math.PI) / 360);

    // Drive PID Constants TODO: Re-tune Drivetrain PID
    public static final SparkMaxConstants LEFT_DRIVE_CONSTANTS = new SparkMaxConstants(
      // Originally (before jerry) the kp was 0.06 - It was changed to reduce wild turning in auto
        0.03, 0, 0.06, 0, 0, -1, 1, 0, 0, 4500, 1000, 0.01);
    public static final SparkMaxConstants RIGHT_DRIVE_CONSTANTS = new SparkMaxConstants(
        0.03, 0, 0.06, 0, 0, -1, 1, 0, 0, 4500, 1000, 0.01);

    // Gyro constants
    public static final SerialPort.Port GYRO_PORT = SerialPort.Port.kMXP;
    public static final boolean GYRO_OUTPUT_INVERTED = false;
    public static final double GYRO_TOLERANCE = 0.8;
  }

  public static final class PERIPHERALS {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    public static final int THIRD_PORT = 3;
    public static final double CONTROLLER_DEADBAND = 0.1;
  }

  public static final class MISC {
    public static final double ENSURE_RANGE(double value, double min, double max) {
      return Math.min(Math.max(value, min), max);
    }

    public static final boolean IN_RANGE(double value, double min, double max) {
      return (value >= min) && (value <= max);
    }

    public static final boolean WITHIN_TOLERANCE(double value, double tolerance) {
      return (value >= -tolerance) && (value <= tolerance);
    }

    public static final boolean WITHIN_TOLERANCE(double input, double setpoint, double tolerance) {
      return (input >= (setpoint - tolerance)) && (input <= (setpoint + tolerance));
    }
  }
}
