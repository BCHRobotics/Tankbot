package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


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
    private final Blinkin m_Blinkin = new Blinkin();
    
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

            Command drivingCommand = m_drivetrain.arcadeDriveCommand(
                () -> -this.driverController.getRightX(), 
                () -> this.driverController.getLeftY()).andThen(()->m_BlbackwardLED(this.driverController.getLeftY()));
        
            // Set the default behavior of the drive subsystem to react to joystick inputs
            m_drivetrain.setDefaultCommand(drivingCommand);
        }
        
    public Command getAutonomousCommand() throws FileVersionException, IOException, ParseException {
        // using the string provided by the user to build and run an auto
        return AutoUtils.BuildAutoFromCommands(AutoUtils.separateCommandString(SmartDashboard.getString("Example Path", "")), m_drivetrain);
    }


    public Drivetrain getDrivetrain() {
        return m_drivetrain;

    }{

    this.driverController.x()
        .whileTrue (this.m_Blinkin.blinkinBetter(-0.35))
        .onFalse (this.m_Blinkin.blinkinBetter(0.87));  
}
}