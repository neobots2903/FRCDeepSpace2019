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

    //THESE ARE NOT SET! REEEEEEE
    final int ELBOW_MAX = 0;
    final int ELBOW_MIN = -1745;
    final int WRIST_MAX = 999999999;
    final int WRIST_MIN = -999999999;

    //Larger elbow = lower position
    //Larger wrist = folded up

    double ELBOW_ZEROPOINT = 0;
    double WRIST_ZEROPOINT = 0;

    final int ELBOW_AWAY = -1454;  //CHANGE!!
    final int WRIST_AWAY = -417;  //CHANGE!!

    final int ELBOW_TOP = -2113;
    final int WRIST_TOP = -1753; //CHANGE!!

    final int ELBOW_MIDDLE = -1591;
    final int WRIST_MIDDLE = -895; //CHANGE!!

    final int ELBOW_BOTTOM = -714;
    final int WRIST_BOTTOM = -415; //CHANGE!!

    final int ELBOW_FLOOR = 0; //CHANGE!!
    final int WRIST_FLOOR = -2022; //CHANGE!!

    final int ELBOW_CONFINED = 0;
    final int WRIST_CONFINED = 0;

    final int ELBOW_FLOORUP = -148; //CHANGE!!
    final int WRIST_FLOORUP = -1420; //CHANGE!!

    @Override
    protected void initDefaultCommand() {
    }
    public PickUpArm2903(){
        Thread thread = new Thread(this);
        elbowPotentiometer = new AnalogInput(RobotMap.elbowPotentiometer);
        leftIntakeMotor = new TalonSRX(RobotMap.TBD);
        rightIntakeMotor = new TalonSRX(RobotMap.TBD);
        wristMotor = new TalonSRX(RobotMap.WristMotor);
        //wristMotor.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0, 10);
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
        Confined, Floor, Bottom, Middle, Top, Away
    }

    public void waitASec() {
        try {
            Thread.sleep(350);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void setArm(ArmState state) {
        //ensureWristCanMove();
        if (state.equals(ArmState.Confined))
            goToConfined();
        else if (state.equals(ArmState.Floor))
            goToFloor();
        else if (state.equals(ArmState.Bottom))
            goToBottom();
        else if (state.equals(ArmState.Middle))
            goToMiddle();
        else if (state.equals(ArmState.Top))
            goToTop();
        else if (state.equals(ArmState.Away))
            goToAway();
    }

    public void setArmAsync(ArmState state) {
        try {
        if (currentTask != null)
            currentTask.stop();
            //if (currentTask.isAlive()) return;

        currentTask = new Thread(new Runnable() {
            public void run() {
                setArm(state);
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

    public void goToConfined() {
        SetWristTarget(WRIST_CONFINED); //Move wrist all the way up
        waitASec(); //Give wrist time to move out of the way
        SetElbowTarget(ELBOW_CONFINED); //Move elbow all the way down
        currentArmState = ArmState.Confined;
    }

    public void goToFloor() {
        SetWristTarget(WRIST_FLOOR);
        waitASec();
        SetElbowTarget(ELBOW_FLOOR);
        waitTillElbowClose(ELBOW_FLOOR);
        currentArmState = ArmState.Floor;
        goToFloorUp();
    }

    public void goToFloorUp() {
        SetElbowTarget(ELBOW_FLOORUP);
        waitASec();
        SetWristTarget(WRIST_FLOORUP);
        currentArmState = ArmState.Floor;
    }

    public void goToBottom() {
        SetWristTarget(WRIST_BOTTOM);
        waitASec(); 
        SetElbowTarget(ELBOW_BOTTOM);
        currentArmState = ArmState.Bottom;
    }

    public void goToMiddle() {
        SetWristTarget(WRIST_MIDDLE);
        waitASec(); 
        SetElbowTarget(ELBOW_MIDDLE);
        currentArmState = ArmState.Middle;
    }

    public void goToTop() {
        if(wristNeedsUp(WRIST_MIDDLE-5)) {
        SetWristTarget(WRIST_MIDDLE);
        waitASec(); 
        }
        SetElbowTarget(ELBOW_TOP);
        waitASec(); 
        SetWristTarget(WRIST_TOP);
        currentArmState = ArmState.Top;
    }

    public void goToAway() {
        if(wristNeedsUp(WRIST_MIDDLE-5)) {
        SetWristTarget(WRIST_MIDDLE);
        waitASec(); 
        }
        SetElbowTarget(ELBOW_AWAY);
        waitASec(); 
        SetWristTarget(WRIST_AWAY);
        currentArmState = ArmState.Away;
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
    
  /*  public void WristUp(){
        wristMotor.set(ControlMode.PercentOutput ,speed);
        SmartDashboard.putNumber("Wrist Encoder Value:", wristMotor.getSelectedSensorPosition());
    }
    public void WristDown(){
        wristMotor.set(ControlMode.PercentOutput ,-speed);
        SmartDashboard.putNumber("Wrist Encoder Value:", wristMotor.getSelectedSensorPosition());
    } */
    public void WristStop(){
        wristMotor.set(ControlMode.PercentOutput, 0);
    }

    public double getElbow() {
    return elbowPotentiometer.getValue()-ELBOW_ZEROPOINT;
    }

    public double getWristTarget() {
        if (currentArmState.equals(ArmState.Confined))
        return WRIST_CONFINED;
    else if (currentArmState.equals(ArmState.Floor))
        return WRIST_FLOOR;
    else if (currentArmState.equals(ArmState.Bottom))
        return WRIST_BOTTOM;
    else if (currentArmState.equals(ArmState.Middle))
        return WRIST_MIDDLE;
    else if (currentArmState.equals(ArmState.Top))
        return WRIST_TOP;
    else if (currentArmState.equals(ArmState.Away))
        return WRIST_AWAY;
    else return 0;
    }

    public void ensureWristCanMove() {
        if (getElbow() > ELBOW_FLOORUP) { //assuming wrist can move at bottom rocket position
            SetElbowTarget(ELBOW_FLOORUP);   //Arm must go up to allow wrist movement
            waitASec(); //make sure arm has time to move out of the way
            }
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
        if(autoPosition)
            SetWristTarget(getWristTarget());
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
        if(autoPosition)
            SetWristTarget(getWristTarget()+300);
    }

    @Override
    public void run() {
        while(true) {
            if(autoPosition) {
                ElbowSet();
                //WristSet();
            }
    }
    }
    
}