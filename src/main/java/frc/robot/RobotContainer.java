package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;


/** 
 * The RobotContainer is where we set up everything for the robot: 
 * 
 * - Creating Subsystem Objects (ex. drivetrain subsystem, elevator subystem) 
 *         NOTE: Jerry only has drivetrain subsystem 
 * 
 * - Creating controller for robot (xbox controller), and getting it's joystick values
 * 
 * - Button mappings (what buttons do what, ex. left bumper --> brake robot)
 * 
 */

public class RobotContainer {
    private final Drivetrain m_drivetrain = new Drivetrain();
    
    private final SendableChooser <Command> autoChooser;

    private final CommandXboxController driverController = new CommandXboxController(Constants.CONTROLLER.DRIVER_CONTROLLER_PORT);

    public Command getAutonomousCommand() {
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
        }
    }
    
        public RobotContainer() {

        configureButtonBindings();
        configureDefaultCommands();

        //building the autochooser for pathplanner
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

    }
        private void configureButtonBindings() {
            this.driverController.leftBumper()
            .whileTrue(this.m_drivetrain.brake()); // Brakes on when button is held
        }

        private void configureDefaultCommands() {

            Command drivingCommand = m_drivetrain.tankDriveCommand(
                () -> -this.driverController.getLeftY(),  // Left joystick controls left side
                () -> this.driverController.getRightY()  // Right joystick controls right side
            );
        
            // Set the default behavior of the drive subsystem to react to joystick inputs
            m_drivetrain.setDefaultCommand(drivingCommand);
        }
        


    public Drivetrain getDrivetrain() {
        return m_drivetrain;

    }
}
