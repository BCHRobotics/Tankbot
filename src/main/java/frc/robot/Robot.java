package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

private RobotContainer robotContainer;

private Command autoCommand;



 /** This method is called once when the robot starts up. */
@Override
public void robotInit() { 
  robotContainer = new RobotContainer();
}

/** This method is called periodically, regardless of the robot mode. */
@Override
public void robotPeriodic() { 
CommandScheduler.getInstance().run();
}

/** This method is called once when teleop mode (driver control) starts. */
@Override
public void teleopInit() { 
   
}

/** This method is called periodically during teleop mode. */
@Override
public void teleopPeriodic() { 
   
}

/** This method is called once when autonomous mode starts. */
@Override
public void autonomousInit() {
  autoCommand = robotContainer.getAutonomousCommand();
  if (autoCommand != null) {
      autoCommand.schedule();
  }
}

/** This function is called periodically during autonomous. */
@Override
  public void autonomousPeriodic() {

}

/** This method is called once each time the robot enters Disabled mode. */
@Override
public void disabledInit() { 
   
}

/** This method is called periodically during disabled mode. */
@Override
public void disabledPeriodic() { 
  robotContainer.getDrivetrain().brake(); 
}

}