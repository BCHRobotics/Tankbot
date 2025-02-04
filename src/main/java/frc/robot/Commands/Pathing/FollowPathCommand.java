package frc.robot.Commands.Pathing;

import java.util.List;
import java.util.concurrent.Callable;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/*
 * A command that drives from one point to another on a path
 * 
 * Once finished, another instance of this command is scheduled for the next point
 * (this is like a link in a chain)
 * 
 * THERE IS A COMMAND INSIDE PATHPLANNER WITH THE SAME NAME!!!! CHANGE THE NAME!!
 */
public class FollowPathCommand extends Command {
    // This command needs to command the drivetrain, so we have a references here
    private Drivetrain driveSubsystem;

    Callable<Pose2d> currentPosition;
    List<Pose2d> targetPoints;

    int pointBeingTargeted;

    public FollowPathCommand(Callable<Pose2d> current, List<Pose2d> targets, int pointIndex, Drivetrain subsystem) {
        // Assign the variables that point to input values
        currentPosition = current;
        
        pointBeingTargeted = pointIndex;

        targetPoints = targets;
        
        // There IS a drivetrain requirement even though the rotation and driving commands also have that requirement
        // This is fine, because all commandds are scheduled at once this command will be canceled, that's fine
        driveSubsystem = subsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        this.setName("followPath");

        // Tell the driver that a new point is being targeted
        System.out.println("CHANGING TARGET POINT TO " + pointBeingTargeted);
    }

    @Override
    public void execute() {
        // there is a point after the currently targeted one, so run another followPath command
        if (pointBeingTargeted < targetPoints.size() - 1) {
            new AlignToPointCommand(currentPosition, () -> targetPoints.get(pointBeingTargeted), driveSubsystem)
            .andThen(new DriveStraightCommand(currentPosition, () -> targetPoints.get(pointBeingTargeted), driveSubsystem))
            .andThen(new FollowPathCommand(currentPosition, targetPoints, pointBeingTargeted + 1, driveSubsystem))
            .schedule();
        }
        else { // this is the last point to target, so don't run another followPath command
            new AlignToPointCommand(currentPosition, () -> targetPoints.get(pointBeingTargeted), driveSubsystem)
            .andThen(new DriveStraightCommand(currentPosition, () -> targetPoints.get(pointBeingTargeted), driveSubsystem))
            .schedule();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // quick debug message to show that the command has in fact canceled
        System.out.println("DRIVING COMMANDS SCHEDULED, FOLLOW COMMAND INTERRUPT");
    }

    @Override
    public boolean isFinished() {
        // Command should run once and then end right away
        return (true);
    }
}
