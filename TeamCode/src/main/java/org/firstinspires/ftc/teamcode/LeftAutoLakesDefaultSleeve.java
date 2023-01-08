/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous (name = "LeftDefaultSleeve", group = "Concept")

public class LeftAutoLakesDefaultSleeve extends LinearOpMode {

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFO
    // D_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };
    Robot robot;
    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 28.0;    // Started at 28.0 -- eg: AndyMark NeverRest40 Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 13.7;     // Started at 40.0 -- This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.4;
    static final double degree_mult = 0.192;

    int autoPosition = 50;
    int neutralPosition = 0;
    int highJunction = 830;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AdSjclj/////AAABGbudF+8chkyGrnyeBGgBf3RWw3N5uISbrxyexQQo09reYhiaN4jWt26ZHHrMXtqJS07ib7fbGBlJI1D69MlboR1gaPg/7MK0/BQ8HwmBrCHvU+yo60VotB/Y4Y5uLbC2eag+w5+AMku+cSKfVNxP52UbVdS4IjzDb6wuWamTGWuY1FVldHyVpU5gUQJZc71heZzI/KKwGr1xbnB12V4hb/xwxpr8kWMVSBknAPpk65keCGOaNsISrMoi/Gl6xvKC0Q2MNEtkX0XUG6ZhhKII0JrpwPsZX8G79Pvi/t3aPb9WQ0ddcuX+USXOO9q4snR+9KwBziiT3wkBQ0qdPLV3ErTeXUxeSfB1qFtfDA3p2UbW";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0 / 9.0);
        }
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

        /* Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        boolean isBoltDetected = false;
                        boolean isBulbDetected = false;
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2;
                            double row = (recognition.getTop() + recognition.getBottom()) / 2;
                            double width = Math.abs(recognition.getRight() - recognition.getLeft());
                            double height = Math.abs(recognition.getTop() - recognition.getBottom());

                            telemetry.addData("", " ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                            telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);

                            if (recognition.getLabel().equals("1 Bolt")) {
                                tfod.deactivate();
                                forward(DRIVE_SPEED,30,15);
                                robot.rightSlideMotor.setTargetPosition(highJunction);
                                robot.leftSlideMotor.setTargetPosition(highJunction);
                                robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                robot.leftSlideMotor.setPower(1);
                                robot.rightSlideMotor.setPower(1);
                                sleep(1000);
                                robot.leftSlideMotor.setPower(0.01);
                                robot.rightSlideMotor.setPower(0.01);
                                robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                strafeRight(DRIVE_SPEED, 14, 15);
                                forward(DRIVE_SPEED,5,15);
                                robot.claw.setPosition(0.5);
                                sleep(500);
                                backward(DRIVE_SPEED,8,15);
                                robot.claw.setPosition(0.2);
                                robot.rightSlideMotor.setTargetPosition(neutralPosition);
                                robot.leftSlideMotor.setTargetPosition(neutralPosition);
                                robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                robot.leftSlideMotor.setPower(1);
                                robot.rightSlideMotor.setPower(1);
                                strafeLeft(DRIVE_SPEED,46,15);
                                break;
                            } if (recognition.getLabel().equals("2 Bulb")) {
                                tfod.deactivate();
                                forward(DRIVE_SPEED,30,15);
                                robot.rightSlideMotor.setTargetPosition(highJunction);
                                robot.leftSlideMotor.setTargetPosition(highJunction);
                                robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                robot.leftSlideMotor.setPower(1);
                                robot.rightSlideMotor.setPower(1);
                                sleep(1000);
                                robot.leftSlideMotor.setPower(0.01);
                                robot.rightSlideMotor.setPower(0.01);
                                robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                strafeRight(DRIVE_SPEED, 14, 15);
                                forward(DRIVE_SPEED,5,15);
                                robot.claw.setPosition(0.5);
                                sleep(500);
                                backward(DRIVE_SPEED,8,15);
                                robot.claw.setPosition(0.2);
                                robot.rightSlideMotor.setTargetPosition(neutralPosition);
                                robot.leftSlideMotor.setTargetPosition(neutralPosition);
                                robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                robot.leftSlideMotor.setPower(1);
                                robot.rightSlideMotor.setPower(1);
                                strafeLeft(DRIVE_SPEED,16,15);
                                break;
                            } if (recognition.getLabel().equals("3 Panel")) {
                                tfod.deactivate();
                                forward(DRIVE_SPEED,30,15);
                                robot.rightSlideMotor.setTargetPosition(highJunction);
                                robot.leftSlideMotor.setTargetPosition(highJunction);
                                robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                robot.leftSlideMotor.setPower(1);
                                robot.rightSlideMotor.setPower(1);
                                sleep(1000);
                                robot.leftSlideMotor.setPower(0.01);
                                robot.rightSlideMotor.setPower(0.01);
                                robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                strafeRight(DRIVE_SPEED, 14, 15);
                                forward(DRIVE_SPEED,5,15);
                                robot.claw.setPosition(0.5);
                                sleep(500);
                                backward(DRIVE_SPEED,8,15);
                                robot.claw.setPosition(0.2);
                                robot.rightSlideMotor.setTargetPosition(neutralPosition);
                                robot.leftSlideMotor.setTargetPosition(neutralPosition);
                                robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                robot.leftSlideMotor.setPower(1);
                                robot.rightSlideMotor.setPower(1);
                                strafeRight(DRIVE_SPEED,15,15);
                                break;
                            } else {
                                isBulbDetected = false;
                            }
                        }

                        telemetry.update();

                    }
                }
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
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

