package frc.robot.Commands.Pathing;

import java.util.concurrent.Callable;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.CHASSIS;
import frc.robot.Constants.DriveMode;
import frc.robot.Constants.PERIPHERALS;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.AutoUtils;

/*
 * A command that turns the robot to face a point, then finishes
 */
public class AlignToPointCommand extends Command {
    // This command needs to command the drivetrain, so we have a references here
    private Drivetrain driveSubsystem;

    Callable<Pose2d> currentPosition;
    Callable<Pose2d> desiredPosition;
    Pose2d currentPose;

    PIDController rotationController;

    double desiredHeading;

    public AlignToPointCommand(Callable<Pose2d> current, Callable<Pose2d> desired, Drivetrain subsystem) {
        rotationController = new PIDController(0.05, 0, 0);

        // Assign the variables that point to input values
        currentPosition = current;
        desiredPosition = desired;
        
        // This command requires the drivetrain so that it cannot run at the same time as other driving commands
        driveSubsystem = subsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        this.setName("pointLock");

        driveSubsystem.setDriveMode(DriveMode.POINTLOCK);
        driveSubsystem.setDeadband(PERIPHERALS.CONTROLLER_DEADBAND);
        driveSubsystem.setRampRate(true);
        driveSubsystem.setBrakeMode(IdleMode.kBrake);

        // Tell the driver that headinglock driving has been enabled
        System.out.println("POINTLOCK ON");

        currentPose = null;
        try {
            currentPose = currentPosition.call();
        } catch (Exception e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    @Override
    public void execute() {
        driveSubsystem.setMaxOutput(CHASSIS.DEFAULT_OUTPUT);

        Pose2d desiredPose = null;
        try {
            desiredPose = desiredPosition.call();
        } catch (Exception e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        if (currentPose != null && desiredPose != null) {
            Transform2d transform = new Transform2d(
                new Pose2d(currentPose.getX(), currentPose.getY(), new Rotation2d()), 
            new Pose2d(desiredPose.getX(), desiredPose.getY(), new Rotation2d()));

            // Using atan2 to get the angle the robot should be facing
            desiredHeading = Math.atan2(transform.getY(), transform.getX());
            desiredHeading *= 180 / Math.PI;

            // Drive the bot
            driveSubsystem.arcadeDrive(0, MathUtil.clamp(rotationController.calculate(driveSubsystem.getHeadingDeg(), desiredHeading), -0.35, 0.35));
        }
        else {
            System.out.println("AUTO DRIVE ERROR: Pose cannot be found!");
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Check to see if the command was canceled by another command or if it ended itself
        if (interrupted) {
            System.out.println("PointLock Off: Interrupted");
        }
        else {
            System.out.println("PointLock Off: DriveMode");
        }
    }

    @Override
    public boolean isFinished() {
        // End if the drive mode is not headinglock
        return (driveSubsystem.getDriveMode() != DriveMode.POINTLOCK || Math.abs(driveSubsystem.getHeadingDeg() - desiredHeading) < 10);
    }
}