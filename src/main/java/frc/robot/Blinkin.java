package frc.robot;


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
    public void backwardLED(double leftControllerYAxis){ // leftControllerYAxis is a value between -1 -> 1
            if (leftControllerYAxis < 0){
            this.SetBlinkin(0.61);
            }
            else if (leftControllerYAxis >= 0){
            this.SetBlinkin(0.93);
            }

    }

}