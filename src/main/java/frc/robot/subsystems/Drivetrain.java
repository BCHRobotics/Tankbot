package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** 
 * Class for the drivetrain subsystem (A BLUEPRINT to CREATE the drivetrain subystem, the ACTUAL subsystem will be created in RobotContainer.java) 
 * Creates and sets up motors, sensors, etc..
 * Has commands related to the drivetrain (ex. commands for driving, braking, etc.)
 * 
 */
public class Drivetrain extends SubsystemBase {
    private final SparkMax frontLeftMotor;
    private final SparkMax frontRightMotor;
    private final SparkMax backLeftMotor;
    private final SparkMax backRightMotor;
    private final DifferentialDrive differentialDrive;
    // private final Timer timer;
    
    public Drivetrain() {
       this.backLeftMotor = new SparkMax (31, MotorType.kBrushless);
       this.frontLeftMotor = new SparkMax (11, MotorType.kBrushless);
       this.backRightMotor = new SparkMax (10, MotorType.kBrushless);
       this.frontRightMotor = new SparkMax (12, MotorType.kBrushless);

       this.frontRightMotor.setInverted(true);
       

       SparkMaxConfig frontLeftConfig = new SparkMaxConfig();
    frontLeftConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kCoast);

    // Persist parameters to retain configuration in the event of a power cycle
    frontLeftMotor.configure(frontLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig frontRightConfig = new SparkMaxConfig();
    frontRightConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kCoast);

    // Persist parameters to retain configuration in the event of a power cycle
    frontRightMotor.configure(frontRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        SparkMaxConfig backLeftConfig = new SparkMaxConfig();
    backLeftConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kCoast);
        backLeftConfig.follow(this.frontLeftMotor, false);
    
    backLeftMotor.configure(backLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig backRightConfig = new SparkMaxConfig();
    backRightConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kCoast);
        backRightConfig.follow(this.frontRightMotor, false);
    
    backRightMotor.configure(backRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       
    this.differentialDrive = new DifferentialDrive(frontLeftMotor, frontRightMotor);

    }

    private double speedMultiplier = 1;
    public Command arcadeDriveCommand (DoubleSupplier forward, DoubleSupplier turn) {
        return run (() -> this.differentialDrive.arcadeDrive(forward.getAsDouble() * speedMultiplier, turn.getAsDouble() * speedMultiplier));//Multiplies the forward and turn speed by the speed multiplier
    }
    
    public Command brake() {
        return runOnce (() -> { 
            this.frontLeftMotor.stopMotor();
            this.frontRightMotor.stopMotor();
        });
    }
    
    public Command slowmode() {
        return runOnce (() -> speedMultiplier=0.5 ); //Makes the speed multiplier 0.5
    }

    public Command turbomode() {
        return runOnce(() -> speedMultiplier=5); //Makes the speed multiplier 5
    }

    public Command normalmode() {
        return runOnce (() -> speedMultiplier=1);//Makes the speed multiplier 1
    }
    
    public Command driveForward () {
        return run (() -> this.differentialDrive.arcadeDrive( 0.25,  0 ));//sets the forward speed to 0.25 and the turn speed to 0
    }
}