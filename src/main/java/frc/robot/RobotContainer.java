package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

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
    
    private final CommandXboxController driverController = new CommandXboxController(Constants.CONTROLLER.DRIVER_CONTROLLER_PORT);

    public RobotContainer() {

        configureButtonBindings();
        configureDefaultCommands();

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
        public class DriveSubsystem extends SubsystemBase {
  // Assuming this is a method in your drive subsystem
        public Command followPathCommand(String pathName) {
            try{
                PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

                return new FollowPathCommand(
                        path,
                        this::getPose, // Robot pose supplier
                        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                        this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
                        new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
                        Constants.robotConfig, // The robot configuration
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
            } catch (Exception e) {
                DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                return Commands.none();
            }
        }
        }
    public Drivetrain getDrivetrain() {
        return m_drivetrain;

    }
}
