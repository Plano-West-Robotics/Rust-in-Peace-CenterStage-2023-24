/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.Data;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeController;
import org.firstinspires.ftc.teamcode.vision.PropDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Arrays;

@Autonomous(name="Red Far-Side Auto", group="Auto")
public class RedFarAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Vision
    private PropDetectionProcessor propDetector;
    private VisionPortal visionPortal;

    Drive drive;
    Lift lift;
    Intake intake;
    OuttakeController control;

    @Override
    public void runOpMode() {
        propDetector = new PropDetectionProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "CAM"), propDetector);

        drive = new Drive(hardwareMap, telemetry, true);
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        control = new OuttakeController(hardwareMap, telemetry, lift);

        lift.setManual(false);

        drive.setSpeed(0.7);
        intake.setSpeed(0.5);

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Chassis Motors (FL-FR-BL-BR)", Arrays.toString(drive.getEncoderValues()));
            telemetry.update();
        }
        runtime.reset();

        // run until the end of the match (driver presses STOP)

        // red by default
        visionPortal.getProcessorEnabled(propDetector);

        // Raise lift to prevent drag when moving
        lift.setPower(0.7, true);
        sleep(500);
        lift.setPower(0.1, true);
        control.armUp();
        control.armDown();

        drive.setSpeed(0.7);
        drive.strafeLeft(100);
        drive.moveForward(2000);

        telemetry.addLine(String.valueOf(propDetector.getLocation()));
        telemetry.update();

        Data.liftPosition = lift.getEncoderValue();

        visionPortal.close();
    }
}
