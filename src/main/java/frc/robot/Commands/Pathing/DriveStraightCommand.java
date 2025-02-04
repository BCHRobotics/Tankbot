package frc.robot.Commands.Pathing;

import java.util.concurrent.Callable;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CHASSIS;
import frc.robot.Constants.DriveMode;
import frc.robot.Constants.PERIPHERALS;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.AutoUtils;

public class DriveStraightCommand extends Command {
    // This command needs to command the drivetrain, so we have a references here
    private Drivetrain driveSubsystem;

    Callable<Pose2d> currentPosition;
    Callable<Pose2d> desiredPosition;

    PIDController distanceController;

    Transform2d transform;

    public DriveStraightCommand(Callable<Pose2d> current, Callable<Pose2d> desired, Drivetrain subsystem) {
        distanceController = new PIDController(3, 0, 0);

        // Assign the variables that point to input values
        currentPosition = current;
        desiredPosition = desired;
        
        // This command requires the drivetrain so that it cannot run at the same time as other driving commands
        driveSubsystem = subsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        this.setName("autoDrive");

        driveSubsystem.setDriveMode(DriveMode.AUTODRIVE);
        driveSubsystem.setDeadband(PERIPHERALS.CONTROLLER_DEADBAND);
        driveSubsystem.setRampRate(true);
        driveSubsystem.setBrakeMode(IdleMode.kBrake);

        transform = null;

        // Tell the driver that headinglock driving has been enabled
        System.out.println("AUTO DRIVE ON");
    }

    @Override
    public void execute() {
        // Turn the robot to a heading
        driveSubsystem.setMaxOutput(CHASSIS.DEFAULT_OUTPUT);

        // Defining current and desired position
        Pose2d currentPose = null;
        try {
            currentPose = currentPosition.call();
        } catch (Exception e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        Pose2d desiredPose = null;
        try {
            desiredPose = desiredPosition.call();
        } catch (Exception e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        if (currentPose != null && desiredPose != null) {
            transform = new Transform2d(
                new Pose2d(currentPose.getX(), currentPose.getY(), new Rotation2d()), 
            new Pose2d(desiredPose.getX(), desiredPose.getY(), new Rotation2d()));
            
            Transform2d headingVector = AutoUtils.getHeadingVector(driveSubsystem.getHeadingDeg());
            Transform2d projectedTransform = AutoUtils.projectOntoVector(transform, headingVector);

            double dist = AutoUtils.magnitude(projectedTransform) * (AutoUtils.dotProduct(projectedTransform, headingVector) < 0 ? -1 : 1);

            // Turn the robot using arcade drive function
            driveSubsystem.arcadeDrive(MathUtil.clamp(-distanceController.calculate(dist, 0), -0.35, 0.35), 0);
        }
        else {
            System.out.println("AUTO DRIVE ERROR: Pose cannot be found!");
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Check to see if the command was canceled by another command or if it ended itself
        if (interrupted) {
            System.out.println("Auto Drive Off: Interrupted");
        }
        else {
            System.out.println("Auto Drive Off: DriveMode");
        }
    }

    @Override
    public boolean isFinished() {
        // End if the drive mode is not headinglock
        boolean hasReachedPosition = false;
        if (transform != null) {
            hasReachedPosition = Math.abs(transform.getX()) < 0.35 && Math.abs(transform.getY()) < 0.35;
        }

        return (driveSubsystem.getDriveMode() != DriveMode.AUTODRIVE) || hasReachedPosition;
    }
}
