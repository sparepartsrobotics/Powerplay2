
/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous
public class RIGHTaprilTagAuto extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1, 2, or 3 from the 36h11 family
    int leftPosition = 1;
    int middlePosition = 2;
    int rightPosition = 3;

    AprilTagDetection tagOfInterest = null;

    Robot robot;
    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 28.0;    // Started at 28.0 -- eg: AndyMark NeverRest40 Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 13.7;     // Started at 40.0 -- This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.4;
    static final double SLOW_DRIVE_SPEED = 0.2;
    static final double degree_mult = 0.192;

    int autoPosition = 50;
    int neutralPosition = -20;
    int highJunction = 1075;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        robot = new Robot(hardwareMap);

        robot.claw.setPosition(0.2);
        sleep(500);
        robot.rightSlideMotor.setTargetPosition(autoPosition);
        robot.leftSlideMotor.setTargetPosition(autoPosition);
        robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftSlideMotor.setPower(1);
        robot.rightSlideMotor.setPower(1);

        robot.rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.topSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bottomSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.rgbDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == leftPosition || tag.id == middlePosition || tag.id == rightPosition) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
            robot.rgbDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
            robot.rgbDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GRAY);
        }

        /* Actually move now */
        if (tagOfInterest == null || tagOfInterest.id == leftPosition) {
            forward(DRIVE_SPEED, 3, 15);
            strafeRight(DRIVE_SPEED, 8, 15);
            forward(DRIVE_SPEED, 27, 15);

            robot.rightSlideMotor.setTargetPosition(highJunction);
            robot.leftSlideMotor.setTargetPosition(highJunction);
            robot.topSlideMotor.setTargetPosition(highJunction);
            robot.bottomSlideMotor.setTargetPosition(highJunction);

            robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.topSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bottomSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftSlideMotor.setPower(1);
            robot.rightSlideMotor.setPower(1);
            robot.topSlideMotor.setPower(1);
            robot.bottomSlideMotor.setPower(1);

            robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.topSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bottomSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            strafeRight(DRIVE_SPEED, 12, 15);
            forward(SLOW_DRIVE_SPEED, 5, 15);
            robot.claw.setPosition(0.5);
            sleep(500);
            backward(DRIVE_SPEED, 8, 15);
            robot.claw.setPosition(0.2);

            robot.rightSlideMotor.setTargetPosition(neutralPosition);
            robot.leftSlideMotor.setTargetPosition(neutralPosition);
            robot.topSlideMotor.setTargetPosition(neutralPosition);
            robot.bottomSlideMotor.setTargetPosition(neutralPosition);

            robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.topSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bottomSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftSlideMotor.setPower(1);
            robot.rightSlideMotor.setPower(1);
            robot.topSlideMotor.setPower(1);
            robot.bottomSlideMotor.setPower(1);

            strafeLeft(DRIVE_SPEED, 46, 15);

        } else if (tagOfInterest.id == middlePosition) {
            forward(DRIVE_SPEED, 3, 15);
            strafeRight(DRIVE_SPEED, 8, 15);
            forward(DRIVE_SPEED, 27, 15);

            robot.rightSlideMotor.setTargetPosition(highJunction);
            robot.leftSlideMotor.setTargetPosition(highJunction);
            robot.topSlideMotor.setTargetPosition(highJunction);
            robot.bottomSlideMotor.setTargetPosition(highJunction);

            robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.topSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bottomSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftSlideMotor.setPower(1);
            robot.rightSlideMotor.setPower(1);
            robot.topSlideMotor.setPower(1);
            robot.bottomSlideMotor.setPower(1);

            robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.topSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bottomSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            strafeRight(DRIVE_SPEED, 12, 15);
            forward(SLOW_DRIVE_SPEED, 5, 15);
            robot.claw.setPosition(0.5);
            sleep(500);
            backward(DRIVE_SPEED, 8, 15);
            robot.claw.setPosition(0.2);

            robot.rightSlideMotor.setTargetPosition(neutralPosition);
            robot.leftSlideMotor.setTargetPosition(neutralPosition);
            robot.topSlideMotor.setTargetPosition(neutralPosition);
            robot.bottomSlideMotor.setTargetPosition(neutralPosition);

            robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.topSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bottomSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftSlideMotor.setPower(1);
            robot.rightSlideMotor.setPower(1);
            robot.topSlideMotor.setPower(1);
            robot.bottomSlideMotor.setPower(1);

            strafeLeft(DRIVE_SPEED, 16, 15);

        } else if (tagOfInterest.id == rightPosition) {
            forward(DRIVE_SPEED, 3, 15);
            strafeRight(DRIVE_SPEED, 8, 15);
            forward(DRIVE_SPEED, 27, 15);

            robot.rightSlideMotor.setTargetPosition(highJunction);
            robot.leftSlideMotor.setTargetPosition(highJunction);
            robot.topSlideMotor.setTargetPosition(highJunction);
            robot.bottomSlideMotor.setTargetPosition(highJunction);

            robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.topSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bottomSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftSlideMotor.setPower(1);
            robot.rightSlideMotor.setPower(1);
            robot.topSlideMotor.setPower(1);
            robot.bottomSlideMotor.setPower(1);

            robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.topSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bottomSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            strafeRight(DRIVE_SPEED, 12, 15);
            forward(SLOW_DRIVE_SPEED, 5, 15);
            robot.claw.setPosition(0.5);
            sleep(500);
            backward(DRIVE_SPEED, 8, 15);
            robot.claw.setPosition(0.2);

            robot.rightSlideMotor.setTargetPosition(neutralPosition);
            robot.leftSlideMotor.setTargetPosition(neutralPosition);
            robot.topSlideMotor.setTargetPosition(neutralPosition);
            robot.bottomSlideMotor.setTargetPosition(neutralPosition);

            robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.topSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bottomSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftSlideMotor.setPower(1);
            robot.rightSlideMotor.setPower(1);
            robot.topSlideMotor.setPower(1);
            robot.bottomSlideMotor.setPower(1);

            strafeRight(DRIVE_SPEED, 15, 15);
        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public void forward(double speed, double inch, double timeout) {
        encoderDrive(speed, -inch, -inch, -inch, -inch, timeout);
    }

    public void backward(double speed, double inch, double timeout) {
        encoderDrive(speed, inch, inch, inch, inch, timeout);
    }

    public void left(double speed, double degree, double timeout) {
        encoderDrive(speed, 1 * ((degree_mult) * degree), 1 * ((degree_mult) * degree), -1 * ((degree_mult) * degree), -1 * ((degree_mult) * degree), timeout);
    }

    public void right(double speed, double degree, double timeout) {
        encoderDrive(speed, -1 * ((degree_mult) * degree), -1 * ((degree_mult) * degree), 1 * ((degree_mult) * degree), 1 * ((degree_mult) * degree), timeout);
    }

    public void strafeLeft(double speed, double inch, double timeout) {
        encoderDrive(speed, inch, -inch, -inch, inch, timeout);
    }

    public void strafeRight(double speed, double inch, double timeout) {
        encoderDrive(speed, -inch, inch, inch, -inch, timeout);
    }

    public void encoderDrive(double speed,
                             double frontLeftWheel, double rearLeftWheel, double frontRightWheel, double rearRightWheel,
                             double timeoutS) {
        int new_tLeftTarget;
        int new_tRightTarget;
        int new_bLeftTarget;
        int new_bRightTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            new_tLeftTarget = robot.frontLeft.getCurrentPosition() + (int) (frontLeftWheel * COUNTS_PER_INCH);
            new_tRightTarget = robot.frontRight.getCurrentPosition() + (int) (frontRightWheel * COUNTS_PER_INCH);
            new_bLeftTarget = robot.rearLeft.getCurrentPosition() + (int) (rearLeftWheel * COUNTS_PER_INCH);
            new_bRightTarget = robot.rearRight.getCurrentPosition() + (int) (rearRightWheel * COUNTS_PER_INCH);
            robot.frontLeft.setTargetPosition(new_tLeftTarget);
            robot.frontRight.setTargetPosition(new_tRightTarget);
            robot.rearLeft.setTargetPosition(new_bLeftTarget);
            robot.rearRight.setTargetPosition(new_bRightTarget);

            // Turn On RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeft.setPower(speed);
            robot.frontRight.setPower(speed);
            robot.rearLeft.setPower(speed);
            robot.rearRight.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.rearLeft.isBusy() && robot.rearRight.isBusy())) {
            }

            // Stop all motion;
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.rearLeft.setPower(0);
            robot.rearRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move

        }
    }
}

