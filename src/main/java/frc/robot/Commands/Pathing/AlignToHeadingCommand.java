package frc.robot.Commands.Pathing;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.CHASSIS;
import frc.robot.Constants.DriveMode;
import frc.robot.Constants.PERIPHERALS;
import frc.robot.subsystems.Drivetrain;

public class AlignToHeadingCommand extends Command {
    // This command needs to command the drivetrain, so we have a references here
    private Drivetrain driveSubsystem;

    DoubleSupplier currentHeading;
    DoubleSupplier desiredHeading;

    PIDController rotationController;

    public AlignToHeadingCommand(DoubleSupplier current, DoubleSupplier desired, Drivetrain subsystem) {
        rotationController = new PIDController(0.04, 0, 0);

        // Assign the variables that point to input values
        currentHeading = current;
        desiredHeading = desired;
        
        // This command requires the drivetrain so that it cannot run at the same time as other driving commands
        driveSubsystem = subsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        this.setName("headingLock");

        driveSubsystem.setDriveMode(DriveMode.HEADINGLOCK);
        driveSubsystem.setDeadband(PERIPHERALS.CONTROLLER_DEADBAND);
        driveSubsystem.setRampRate(true);
        driveSubsystem.setBrakeMode(IdleMode.kBrake);

        // Tell the driver that headinglock driving has been enabled
        System.out.println("HEADINGLOCK ON");
    }

    @Override
    public void execute() {
        // Turn the robot to a heading
        driveSubsystem.setMaxOutput(CHASSIS.DEFAULT_OUTPUT);

        // Turn the robot using arcade drive function
        driveSubsystem.arcadeDrive(0, MathUtil.clamp(rotationController.calculate(currentHeading.getAsDouble(), desiredHeading.getAsDouble()), -0.35, 0.35));
    }

    @Override
    public void end(boolean interrupted) {
        // Check to see if the command was canceled by another command or if it ended itself
        if (interrupted) {
            System.out.println("HeadingLock Off: Interrupted");
        }
        else {
            System.out.println("HeadingLock Off: DriveMode");
        }
    }

    @Override
    public boolean isFinished() {
        // End if the drive mode is not headinglock
        return (driveSubsystem.getDriveMode() != DriveMode.HEADINGLOCK) || Math.abs(currentHeading.getAsDouble() - desiredHeading.getAsDouble()) < 10;
    }
}
