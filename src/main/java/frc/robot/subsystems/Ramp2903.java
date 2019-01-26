package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Ramp2903 extends Subsystem {

    public Solenoid rampUp = new Solenoid(/*insert robot map value*/);
    public Solenoid rampDown = new Solenoid(/*insert robot map value*/);

    protected void initDefaultCommand() {
        
    }

    public void init() {
        rampUp.set(false);
        rampDown.set(true);
    }

    public void lowerRamp() {
        rampUp.set(false);
        rampDown.set(true);
    }

    public void raiseRamp() {
        rampDown.set(false);
        rampUp.set(true);
    }

}