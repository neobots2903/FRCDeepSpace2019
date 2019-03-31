package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;



public class PickUpArm2903 extends Subsystem implements Runnable{
    double speed = 0.5;
    double elbowError = 2;
    private TalonSRX leftIntakeMotor;
    private TalonSRX rightIntakeMotor;
    private TalonSRX wristMotor;
    private TalonSRX elbowMotor;
    public Solenoid panelRetract;
    public Solenoid panelEject;
    public AnalogInput elbowPotentiometer;
    public DigitalInput topHall;
    public DigitalInput bottomHall;

    ArmState currentArmState = ArmState.Confined;
    Thread currentTask = null;
    public boolean autoPosition = false;
    boolean autoPositionLock = false;

    //THESE ARE NOT SET! REEEEEEE
    final int ELBOW_MAX = 0;
    final int ELBOW_MIN = -2255;
    final int WRIST_MAX = 999999999;
    final int WRIST_MIN = -999999999;

    //Larger elbow = lower position
    //Larger wrist = folded up

    double ELBOW_ZEROPOINT = 0;
    double WRIST_ZEROPOINT = 0;

    final int ELBOW_TOP = -2181;
    final int WRIST_TOP = -3821;

    final int ELBOW_MIDDLE = -1591;
    final int WRIST_MIDDLE = -2891;

    final int ELBOW_BOTTOM = -714;
    final int WRIST_BOTTOM = -2184;

    final int ELBOW_CONFINED = -40;
    final int WRIST_CONFINED = 0;

    @Override
    protected void initDefaultCommand() {
    }
    public PickUpArm2903(){
        Thread thread = new Thread(this);
        elbowPotentiometer = new AnalogInput(RobotMap.elbowPotentiometer);
        leftIntakeMotor = new TalonSRX(RobotMap.TBD);
        rightIntakeMotor = new TalonSRX(RobotMap.TBD);
        wristMotor = new TalonSRX(RobotMap.WristMotor);
        wristMotor.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0, 10);
        elbowMotor = new TalonSRX(RobotMap.DartMotor);
        panelRetract = new Solenoid(RobotMap.panelRetract);
        panelEject = new Solenoid(RobotMap.panelEject);
        topHall = new DigitalInput(RobotMap.upperHall);
        bottomHall = new DigitalInput(RobotMap.bottomHall);
        ELBOW_ZEROPOINT = getElbow();
        WRIST_ZEROPOINT = getWrist();
        thread.start();
    }

    public enum ArmState {
        Confined, Bottom, Middle, Top
    }

    public void waitASec() {
        try {
            Thread.sleep(600);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        // No longer needed.
    }

    public void setArmAsync(ArmState state) {
        try {
        if (currentTask != null)
            currentTask.stop();
            //if (currentTask.isAlive()) return;

        currentTask = new Thread(new Runnable() {
            public void run() {
                //ensureWristCanMove();
                setTargetsGradually(state);
                return;
            }
        });
        currentTask.start();    //Running as thread so sleep functions don't block teleOp
    } catch (Exception ex) { }
    }

    public void waitTillElbowClose(double position) {
        while (Math.abs(getElbow() - position) > 80)
            waitASec();
    }

    public void setTargetsGradually(ArmState state) {
        int ticks = 200;

        double wristTarget =
        (state.equals(ArmState.Confined)) ? WRIST_CONFINED :
        (state.equals(ArmState.Bottom)) ? WRIST_BOTTOM :
        (state.equals(ArmState.Middle)) ? WRIST_MIDDLE :
        (state.equals(ArmState.Top)) ? WRIST_TOP :
        0;

        double elbowTarget =
        (state.equals(ArmState.Confined)) ? ELBOW_CONFINED :
        (state.equals(ArmState.Bottom)) ? ELBOW_BOTTOM :
        (state.equals(ArmState.Middle)) ? ELBOW_MIDDLE :
        (state.equals(ArmState.Top)) ? ELBOW_TOP :
        0;

        double wristCurrent = getWrist();
        double elbowCurrent = getElbow();

        double wristRate = (wristCurrent - wristTarget) / ticks;
        double elbowRate = (elbowCurrent - elbowTarget) / ticks;

        for (int i = 0; i < ticks; i++) {
            SetWristTarget(wristCurrent - wristRate*i);
            SetElbowTarget(elbowCurrent - elbowRate*i);

            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        SetWristTarget(wristTarget);
        SetElbowTarget(elbowTarget);

        currentArmState = state;
    }

    public void TakeIn() {
        leftIntakeMotor.set(ControlMode.PercentOutput ,speed);
        rightIntakeMotor.set(ControlMode.PercentOutput ,speed);
    }
    public void TakeOut() {
        leftIntakeMotor.set(ControlMode.PercentOutput ,-speed);
        rightIntakeMotor.set(ControlMode.PercentOutput ,-speed);
    }
    public void TakeStop() {
        leftIntakeMotor.set(ControlMode.PercentOutput ,0);
        rightIntakeMotor.set(ControlMode.Position ,0);
    }
    public void WristSet(){
        wristMotor.set(ControlMode.PercentOutput, -Robot.wristValue);
        SmartDashboard.putNumber("Wrist Encoder Value:", getWrist());
        SmartDashboard.putNumber("Wrist Speed:", Robot.wristValue);
    }

    public void WristSetHa(double speed) {
        wristMotor.set(ControlMode.PercentOutput, speed);
        SmartDashboard.putNumber("Wrist Speed:", speed);
        SmartDashboard.putNumber("Wrist Encoder Value:", getWrist());
    }

    public void WristStop(){
        wristMotor.set(ControlMode.PercentOutput, 0);
    }

    public double getElbow() {
    return elbowPotentiometer.getValue()-ELBOW_ZEROPOINT;
    }

    public double getWristTarget() {
        return
        (currentArmState.equals(ArmState.Confined)) ? WRIST_CONFINED :
        (currentArmState.equals(ArmState.Bottom)) ? WRIST_BOTTOM :
        (currentArmState.equals(ArmState.Middle)) ? WRIST_MIDDLE :
        (currentArmState.equals(ArmState.Top)) ? WRIST_TOP :
        0;
    }

    public void ensureWristCanMove() {
        // if (getElbow() > ELBOW_FLOORUP) { //assuming wrist can move at bottom rocket position
        //     SetElbowTarget(ELBOW_FLOORUP);   //Arm must go up to allow wrist movement
        //     waitASec(); //make sure arm has time to move out of the way
        //     }
    }

    public boolean wristNeedsUp(double position) {
        return getWrist() < position;
    }

    public double getWrist() {
        return wristMotor.getSelectedSensorPosition()-WRIST_ZEROPOINT;
    }
    
    public void ElbowSet(){
        if ((!topHall.get() && -Robot.dartValue > 0) ||
        (!bottomHall.get() && -Robot.dartValue < 0)) 
            elbowMotor.set(ControlMode.PercentOutput, 0);
        else
            elbowMotor.set(ControlMode.PercentOutput, (-Robot.dartValue)* ((-Robot.dartValue < 0) ? 1.5 : 1));

        SmartDashboard.putNumber("Elbow Encoder Value:", getElbow());
        SmartDashboard.putNumber("Elbow Speed:", -Robot.dartValue);
        SmartDashboard.putBoolean("Top Hall:", !topHall.get());
        SmartDashboard.putBoolean("Bottom Hall:", !bottomHall.get());
    }

    public void ElbowSetHa(double speed) {
        if ((!topHall.get() && speed > 0) ||
        (!bottomHall.get() && speed < 0)) 
            elbowMotor.set(ControlMode.PercentOutput, 0);
        else
        elbowMotor.set(ControlMode.PercentOutput, speed);

        SmartDashboard.putNumber("Elbow Encoder Value:", getElbow());
        SmartDashboard.putNumber("Elbow Speed:", Robot.dartValue);
        SmartDashboard.putBoolean("Top Hall:", !topHall.get());
        SmartDashboard.putBoolean("Bottom Hall:", !bottomHall.get());
    }

    public void SetElbowTarget(double t){
        if(t > ELBOW_MAX)
            Robot.dartController.setSetpoint(ELBOW_MAX);
        else if(t < ELBOW_MIN)
            Robot.dartController.setSetpoint(ELBOW_MIN);
        else 
            Robot.dartController.setSetpoint(t);
    }

    public void SetWristTarget(double t){
        if(t > WRIST_MAX)
            Robot.wristController.setSetpoint(WRIST_MAX);
        else if(t < WRIST_MIN)
            Robot.wristController.setSetpoint(WRIST_MIN);
        else 
            Robot.wristController.setSetpoint(t);
    }

   /* public void ElbowUp(){ 
        elbowMotor.set(ControlMode.PercentOutput ,speed);
        SmartDashboard.putNumber("Potentiometer Value:",elbowPotentiometer.getValue());
    }
    public void ElbowDown(){
        elbowMotor.set(ControlMode.PercentOutput ,-speed);
        SmartDashboard.putNumber("Potentiometer Value:",elbowPotentiometer.getValue());
    } */
    public void ElbowStop(){
        elbowMotor.set(ControlMode.PercentOutput, 0);
    }
    public void punch(){
        eject();
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        retract();
    }

    public void retract() {
        panelRetract.set(false);
        panelEject.set(true);
    }

    public void eject() {
        panelRetract.set(true);
        panelEject.set(false);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    @Override
    public void run() {
        while(true) {
            if(autoPosition) {
                if (!autoPositionLock) {
                    SetWristTarget(getWrist());
                    SetElbowTarget(getElbow());
                    autoPositionLock = true;
                }
                
                ElbowSet();
                WristSet();
            } else {
                autoPositionLock = false;
            }
    }
    }
    
}