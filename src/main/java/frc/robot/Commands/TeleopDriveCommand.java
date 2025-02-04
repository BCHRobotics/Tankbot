package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CHASSIS;
import frc.robot.Constants.DriveMode;
import frc.robot.Constants.PERIPHERALS;
import frc.robot.subsystems.Drivetrain;

public class TeleopDriveCommand extends Command {
    // This command needs to command the drivetrain, so we have a references here
    private Drivetrain driveSubsystem;

    // These three are inputs for x and rot speed from the driver
    // NOTE - these are NOT DOUBLES, they are DOUBLE SUPPLIERS, which point to a double value
    // this is the easiest way to get the updated speeds, instead of assigning the variables over and over
    DoubleSupplier commandX;
    DoubleSupplier commandRot;

    DoubleSupplier minOutput;
    DoubleSupplier maxOutput;

    public TeleopDriveCommand(DoubleSupplier xSpeed, DoubleSupplier rotSpeed, DoubleSupplier min, DoubleSupplier max, Drivetrain subsystem) {
        // Assign the variables that point to input values
        commandX = xSpeed;
        commandRot = rotSpeed;

        minOutput = min;
        maxOutput = max;
        
        // This command requires the drivetrain so that it cannot run at the same time as other driving commands
        driveSubsystem = subsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        this.setName("arcadeDrive");
       
        driveSubsystem.setDriveMode(DriveMode.MANUAL);
        driveSubsystem.setDeadband(PERIPHERALS.CONTROLLER_DEADBAND);
        driveSubsystem.setRampRate(true);
        driveSubsystem.setBrakeMode(IdleMode.kBrake);
        
        // figure out what the hell withInterruptBehavior(InterruptionBehavior.kCancelSelf) does

        // Tell the driver that manual driving has been enabled
        System.out.println("MANUAL ON");
    }

    @Override
    public void execute() {
        driveSubsystem.setMaxOutput(CHASSIS.DEFAULT_OUTPUT + (maxOutput.getAsDouble() * CHASSIS.MAX_INTERVAL)
          - (minOutput.getAsDouble() * CHASSIS.MIN_INTERVAL));
        
        // Drive the robot, suppling x and y values
        driveSubsystem.arcadeDrive(commandX.getAsDouble(), commandRot.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        // Check to see if the command was canceled by another command or if it ended itself
        if (interrupted) {
            // This will happen most of the time, e.g. when switching to vision
            System.out.println("Manual Off: Interrupted");
        }
        else {
            // This happens when the driving mode switches off manual
            // Doesn't usually happen, for example when a vision command is triggered 
            // it is setup before the drive mode is set
            // So the program interrupts this command (because of the new one)
            // before it realizes the driveMode isn't manual anymore
            System.out.println("Manual Off: DriveMode");
        }
    }

    @Override
    public boolean isFinished() {
        // End if the drive mode is not manual
        return driveSubsystem.getDriveMode() != DriveMode.MANUAL;
    }
}
