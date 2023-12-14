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
        DOWN,
        UP,
        LEFT,
        RIGHT
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

       if(st == State.UP){
            //set servo values to up
           this.state = State.UP;
       }
       else if(st == State.DOWN){

           //if state isnt left and state isnt right then set to down
           if(!((state == State.LEFT) || (state == State.RIGHT))){
               //setServo values to down
           }

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





}
