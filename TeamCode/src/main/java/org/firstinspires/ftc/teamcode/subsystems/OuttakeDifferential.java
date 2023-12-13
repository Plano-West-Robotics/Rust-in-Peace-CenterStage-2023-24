package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OuttakeDifferential {


    //tracks current state of arm
    private State state;
    private Servo servoL;
    private Servo servoR;

    private final Telemetry telemetry;
    public enum State{
        down,
        up,
        left,
        right
    };

    public OuttakeDifferential(HardwareMap hardwareMap, Telemetry telemetry, State state) {

        this.telemetry = telemetry;
        servoL= hardwareMap.get(Servo.class, "");
        servoR = hardwareMap.get(Servo.class,"");

        this.state = state;
    }


    /**States are state.up, state.down, state.left, and state.right
     *
     * @param st the desired state
     */
    public void goTo(State st){

       if(st == State.up){
            //set servo values to up
           this.state = State.up;
       }
       else if(st == State.down){

           //if state isnt left and state isnt right then set to down
           if(!((state == State.left) || (state == State.right))){
               //setServo values to down
           }

       }
       else if(st == State.left){
            if(state == State.down){
                goTo(State.up);
            }
            //Set servo values to left
       }else if(st == State.right){
           if(state == State.down){
               goTo(State.up);
           }
           //Set servo values to right
       }

    }





}
