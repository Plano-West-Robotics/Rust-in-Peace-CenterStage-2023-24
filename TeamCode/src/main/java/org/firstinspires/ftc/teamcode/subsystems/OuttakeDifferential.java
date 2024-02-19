package org.firstinspires.ftc.teamcode.subsystems;


import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OuttakeDifferential {
    //tracks current state of arm
    public State state;
    public WristState wristState;
    public Servo servoL;
    public Servo servoR;
    public OuttakeBox box;
    private DroneLauncher drone;
    private final Telemetry telemetry;

    public enum State{
        DOWN,
        UP,
        FARLEFT,
        LEFT,
        RIGHT,
        FARRIGHT
    };

    public enum WristState {
        PASSIVE,
        MANUAL
    }

    public OuttakeDifferential(HardwareMap hardwareMap, Telemetry telemetry, State state) {

        this.telemetry = telemetry;
        servoL = hardwareMap.get(Servo.class, "ALservo");
        servoR = hardwareMap.get(Servo.class,"ARservo");

        servoR.setDirection(Servo.Direction.REVERSE);
        box = new OuttakeBox(hardwareMap, telemetry);
        drone = new DroneLauncher(hardwareMap, telemetry);

        this.state = state;
        wristState = WristState.PASSIVE;
    }


    /** States are state.up, state.down, state.left, and state.right
     * @param st the desired state
     */
    public void goTo(State st) {
        new Thread(() -> {
            try {
                if (state == State.DOWN && st != State.DOWN) {
                    box.stopSpinning();
                    drone.droneShoot.setPosition(1);
                    box.setWristPosition(OuttakeBox.State.P3);

                    servoL.getController().pwmEnable();
                    servoR.getController().pwmEnable();
                }

                if (st == State.UP) {
                    servoR.setPosition(0.58);
                    servoL.setPosition(0.52);
                    sleep(500);
                    this.state = State.UP;
                    setWrist(wristState);
                    sleep(500);
                } else if (st == State.DOWN) {
                    //if state is left or right set to up first
                    if ((state == State.LEFT) || (state == State.RIGHT) || (state == State.FARLEFT) || (state == State.FARRIGHT)) {
                        goTo(State.UP, true);
                        this.state = State.DOWN;
                    }
                    box.setWristPosition(OuttakeBox.State.P3);
                    sleep(500);
                    servoL.setPosition(0.02);
                    servoR.setPosition(0.02);
                    sleep(650);
                    servoL.getController().pwmDisable();
                    servoR.getController().pwmDisable();
                } else if (st == State.FARLEFT) {
                    if (state == State.DOWN) {
                        goTo(State.UP, true);
                        sleep(700);
                    }
                    this.state = State.FARLEFT;
                    setWrist(wristState);
                    servoL.setPosition(1);
                    servoR.setPosition(0.35);
                    sleep(800);
                } else if (st == State.LEFT) {
                    if (state == State.DOWN) {
                        goTo(State.UP, true);
                        sleep(700);
                    }
                    this.state = State.LEFT;
                    setWrist(wristState);
                    servoL.setPosition(0.6);
                    servoR.setPosition(0.5);
                    sleep(800);
                } else if (st == State.RIGHT) {
                    if (state == State.DOWN) {
                        goTo(State.UP, true);
                        sleep(700);
                    }
                    this.state = State.RIGHT;
                    setWrist(wristState);
                    servoL.setPosition(0.4);
                    servoR.setPosition(0.7);
                    sleep(800);
                } else if (st == State.FARRIGHT) {
                    if (state == State.DOWN) {
                        goTo(State.UP, true);
                        sleep(700);
                    }
                    this.state = State.FARRIGHT;
                    setWrist(wristState);
                    servoL.setPosition(0.22);
                    servoR.setPosition(0.85);
                    sleep(800);
                }

            } catch (Exception ignored) {}
        }).start();
    }

    public void goTo(State st, boolean auto) {
        try {
            if (state == State.DOWN && st != State.DOWN) {
                box.stopSpinning();
                drone.droneShoot.setPosition(1);
                box.setWristPosition(OuttakeBox.State.P3);

                servoL.getController().pwmEnable();
                servoR.getController().pwmEnable();
            }

            if (st == State.UP) {
                servoL.setPosition(0.52);
                servoR.setPosition(0.58);
                sleep(500);
                this.state = State.UP;
            } else if (st == State.DOWN) {
                //if state is left or right set to up first
                if ((state == State.LEFT) || (state == State.RIGHT)) {
                    goTo(State.UP);
                }
                this.state = State.DOWN;
                servoL.setPosition(0.04);
                servoR.setPosition(0);
                sleep(750);
                servoL.getController().pwmDisable();
                servoR.getController().pwmDisable();
            } else if (st == State.LEFT) {
                if (state == State.DOWN) {
                    goTo(State.UP);
                }
            } else if (st == State.RIGHT) {
                if (state == State.DOWN) {
                    goTo(State.UP);
                }
                servoL.setPosition(0.3);
                servoR.setPosition(1);
                sleep(1000);
                this.state = State.RIGHT;
                setWrist(wristState);
            }
        } catch (Exception ignored) {}
    }

    public void setWrist(WristState st) {
        new Thread(() -> {
            if (st == WristState.PASSIVE) {
                if (state == State.UP || state == State.DOWN) {
                    try {
                        box.setWristPosition(OuttakeBox.State.P3);
                    } catch (Exception ignored) {}
                } else if (state == State.FARRIGHT) {
                    try {
                        box.setWristPosition(OuttakeBox.State.P2);
                    } catch (Exception ignored) {}
                } else if (state == State.FARLEFT) {
                    try {
                        box.setWristPosition(OuttakeBox.State.P4);
                    } catch (Exception ignored) {}
                } else if (state == State.LEFT) {
                    try {
                        box.setWristPosition(OuttakeBox.State.P5);
                    } catch (Exception ignored) {}
                } else if (state == State.RIGHT) {
                    try {
                        box.setWristPosition(OuttakeBox.State.P6);
                    } catch (Exception ignored) {}
                }
                wristState = WristState.PASSIVE;
            } else if (st == WristState.MANUAL) {
                if (state == State.UP) {
                    try {
                        box.setWristPosition(OuttakeBox.State.P1);
                    } catch (Exception ignored) {}
                }  else if (state == State.FARRIGHT) {
                    try {
                        box.setWristPosition(OuttakeBox.State.P4);
                    } catch (Exception ignored) {}
                } else if (state == State.FARLEFT) {
                    try {
                        box.setWristPosition(OuttakeBox.State.P2);
                    } catch (Exception ignored) {}
                } else if (state == State.RIGHT) {
                    try {
                        box.setWristPosition(OuttakeBox.State.P7);
                    } catch (Exception ignored) {}
                }
                wristState = WristState.MANUAL;
            }
        }).start();
    }

    public boolean boxIsFull() {
        return box.boxIsFull();
    }

    public boolean boxIsEmpty() {
        return box.boxIsEmpty();
    }

}
