package org.firstinspires.ftc.teamcode.subsystems;


import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OuttakeDifferential {


    //tracks current state of arm
    public State state;
    public WristState wristState;
    private Servo servoL;
    private Servo servoR;
    public OuttakeBox box;
    private final Telemetry telemetry;

    public enum State{
        DOWN,
        UP,
        LEFT,
        RIGHT
    };

    public enum WristState {
        PASSIVE,
        MANUAL
    }

    public OuttakeDifferential(HardwareMap hardwareMap, Telemetry telemetry, State state) {

        this.telemetry = telemetry;
        servoL = hardwareMap.get(Servo.class, "ALservo");
        servoR = hardwareMap.get(Servo.class,"ARservo");

        box = new OuttakeBox(hardwareMap, telemetry);

        servoR.setDirection(Servo.Direction.REVERSE);

        this.state = state;
        wristState = WristState.MANUAL;
    }


    /** States are state.up, state.down, state.left, and state.right
     * @param st the desired state
     */
    public void goTo(State st) throws InterruptedException {
       if (state == State.DOWN && st != State.DOWN) {
           box.stopSpinning();
           servoL.getController().pwmEnable();
           servoL.getController().pwmEnable();
       }

       if(st == State.UP){
           servoL.setPosition(0.6);
           servoR.setPosition(0.57);
           sleep(1000);
           this.state = State.UP;
       }
       else if(st == State.DOWN){
           //if state is left or right set to up first
           if((state == State.LEFT) || (state == State.RIGHT)){
               goTo(State.UP);
           }
           setWrist(WristState.PASSIVE);
           servoL.setPosition(0.06);
           servoR.setPosition(0);
           sleep(750);
           servoL.getController().pwmDisable();
           servoR.getController().pwmDisable();
           this.state = State.DOWN;
       }
       else if(st == State.LEFT){
            if(state == State.DOWN){
                goTo(State.UP);
            }
            //Set servo values to left
       }else if(st == State.RIGHT){
           if(state == State.DOWN){
               goTo(State.UP);
           }
           //Set servo values to right
       }

    }

    public void setWrist(WristState st) throws InterruptedException {
        if (st == WristState.PASSIVE && wristState != WristState.PASSIVE) {
            if (state == State.UP || state == State.DOWN) {
                box.setWristPosition(OuttakeBox.State.P3);
            }
            wristState = WristState.PASSIVE;
        } else if (st == WristState.MANUAL && wristState != WristState.MANUAL) {
            if (state == State.UP) {
                box.setWristPosition(OuttakeBox.State.P1);
            }
            wristState = WristState.MANUAL;
        }
    }

}
