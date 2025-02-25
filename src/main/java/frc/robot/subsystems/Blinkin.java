package frc.robot.subsystems;


import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Blinkin extends SubsystemBase{
    Spark blinkin;
    public Blinkin(){
    blinkin = new Spark(0);
    }

    public void SetBlinkin(double colour){
        blinkin.set(colour);
    }


    public Command blinkinBetter(double colour){
        
        return runOnce(()-> SetBlinkin(colour));
    }
    
    // trying to make command for backward movement, cleaning up periodic

    public void backwardLED(DoubleSupplier joystickReading){
        // leftControllerYAxis is a value between -1 -> 1
        
        if (joystickReading.getAsDouble() <= 0){
            blinkin.set(0.61);
        }
        else{
            blinkin.set(0.99);
        }
    }

    }

