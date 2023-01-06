package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "TeleOp")

public class teleop extends LinearOpMode {

    public volatile Robot robot;

    double flMotorPower = 1;
    double frMotorPower = 1;
    double brMotorPower = 1;
    double blMotorPower = 1;
    double rightSlideMotor;
    double leftSlideMotor;
    double joystick1LeftX;
    double joystick1RightX;
    double joystick1LeftY;

    int highJunction = 1080;
    int mediumJunction = 775;
    int lowJunction = 500;
    int groundJunction = 100;

    int neutralPosition = 0;

    int conePosition5 = 150;
    int conePosition4 = 115;
    int conePosition3 = 90;
    int conePosition2 = 65;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);

        /*
        robot.claw.setPosition(0.2);
        robot.rightSlideMotor.setTargetPosition(conePosition2);
        robot.leftSlideMotor.setTargetPosition(conePosition2);
        robot.leftSlideMotor.setMode(RUN_TO_POSITION);
        robot.rightSlideMotor.setMode(RUN_TO_POSITION);
        robot.leftSlideMotor.setPower(1);
        robot.rightSlideMotor.setPower(1);
        */

//        robot.leftSlideMotor.setMode(STOP_AND_RESET_ENCODER);
//        robot.rightSlideMotor.setMode(STOP_AND_RESET_ENCODER);
//        robot.topSlideMotor.setMode(STOP_AND_RESET_ENCODER);
//        robot.bottomSlideMotor.setMode(STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            controls();
        }
    }

    public void controls() throws InterruptedException {

//        telemetry.addData("time",time);

        if (time < 80) {
            robot.rgbDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
        } else if (time > 110) {
            robot.rgbDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
        } else if(time > 90) {
            robot.rgbDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
        } else {
            robot.rgbDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
        }

        joystick1LeftX = gamepad1.left_stick_x;
        joystick1LeftY = gamepad1.left_stick_y;
        joystick1RightX = gamepad1.right_stick_x;

        flMotorPower = joystick1LeftY - joystick1LeftX - joystick1RightX;
        frMotorPower = joystick1LeftY + joystick1LeftX + joystick1RightX;
        blMotorPower = joystick1LeftY + joystick1LeftX - joystick1RightX;
        brMotorPower = joystick1LeftY - joystick1LeftX + joystick1RightX;

        if (gamepad1.right_trigger > 0) {
            robot.frontLeft.setPower(flMotorPower / 1.33);
            robot.frontRight.setPower(frMotorPower / 1.33);
            robot.rearRight.setPower(brMotorPower / 1.33);
            robot.rearLeft.setPower(blMotorPower / 1.33);
        } else {
            robot.frontLeft.setPower(flMotorPower * 0.35);
            robot.frontRight.setPower(frMotorPower * 0.35);
            robot.rearRight.setPower(brMotorPower * 0.35);
            robot.rearLeft.setPower(blMotorPower * 0.35);
        }

        if (gamepad1.left_bumper) {
            robot.claw.setPosition(0.5);
        }

        if (gamepad1.right_bumper) {
            robot.claw.setPosition(0.2);
        }

        if (gamepad1.a) {

            robot.rgbDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE);

            int slideJunctionTarget;
            slideJunctionTarget = groundJunction;
            robot.leftSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.rightSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.topSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.bottomSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.leftSlideMotor.setMode(RUN_TO_POSITION);
            robot.rightSlideMotor.setMode(RUN_TO_POSITION);
            robot.topSlideMotor.setMode(RUN_TO_POSITION);
            robot.bottomSlideMotor.setMode(RUN_TO_POSITION);
            robot.leftSlideMotor.setPower(1);
            robot.rightSlideMotor.setPower(1);
            robot.topSlideMotor.setPower(1);
            robot.bottomSlideMotor.setPower(1);

            while (robot.leftSlideMotor.isBusy() && robot.rightSlideMotor.isBusy() && robot.topSlideMotor.isBusy() && robot.bottomSlideMotor.isBusy()) {

                telemetry.addData("leftMotor", robot.leftSlideMotor.getCurrentPosition());
                telemetry.addData("rightMotor", robot.leftSlideMotor.getCurrentPosition());
                telemetry.addData("topMotor", robot.leftSlideMotor.getCurrentPosition());
                telemetry.addData("bottomMotor", robot.leftSlideMotor.getCurrentPosition());
                telemetry.update();

                joystick1LeftX = gamepad1.left_stick_x;
                joystick1LeftY = gamepad1.left_stick_y;
                joystick1RightX = gamepad1.right_stick_x;

                flMotorPower = joystick1LeftY - joystick1LeftX - joystick1RightX;
                frMotorPower = joystick1LeftY + joystick1LeftX + joystick1RightX;
                brMotorPower = joystick1LeftY - joystick1LeftX + joystick1RightX;
                blMotorPower = joystick1LeftY + joystick1LeftX - joystick1RightX;

                robot.frontLeft.setPower(flMotorPower * 0.35);
                robot.frontRight.setPower(frMotorPower * 0.35);
                robot.rearRight.setPower(brMotorPower * 0.35);
                robot.rearLeft.setPower(blMotorPower * 0.35);

                if (gamepad1.right_bumper) {
                    robot.claw.setPosition(0.5);
                }

                if (gamepad1.left_bumper) {
                    robot.claw.setPosition(0.2);
                }
            }
            robot.leftSlideMotor.setPower(0);
            robot.rightSlideMotor.setPower(0);
            robot.topSlideMotor.setPower(0);
            robot.bottomSlideMotor.setPower(0);
            robot.topSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bottomSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (gamepad1.b) {

            robot.rgbDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

            int slideJunctionTarget;
            slideJunctionTarget = lowJunction;
            robot.leftSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.rightSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.topSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.bottomSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.leftSlideMotor.setMode(RUN_TO_POSITION);
            robot.rightSlideMotor.setMode(RUN_TO_POSITION);
            robot.topSlideMotor.setMode(RUN_TO_POSITION);
            robot.bottomSlideMotor.setMode(RUN_TO_POSITION);
            robot.leftSlideMotor.setPower(1);
            robot.rightSlideMotor.setPower(1);
            robot.topSlideMotor.setPower(1);
            robot.bottomSlideMotor.setPower(1);

            while (robot.leftSlideMotor.isBusy() && robot.rightSlideMotor.isBusy() && robot.topSlideMotor.isBusy() && robot.bottomSlideMotor.isBusy()) {

                telemetry.addData("leftMotor", robot.leftSlideMotor.getCurrentPosition());
                telemetry.addData("rightMotor", robot.leftSlideMotor.getCurrentPosition());
                telemetry.addData("topMotor", robot.leftSlideMotor.getCurrentPosition());
                telemetry.addData("bottomMotor", robot.leftSlideMotor.getCurrentPosition());
                telemetry.update();

                joystick1LeftX = gamepad1.left_stick_x;
                joystick1LeftY = gamepad1.left_stick_y;
                joystick1RightX = gamepad1.right_stick_x;

                flMotorPower = joystick1LeftY - joystick1LeftX - joystick1RightX;
                frMotorPower = joystick1LeftY + joystick1LeftX + joystick1RightX;
                brMotorPower = joystick1LeftY - joystick1LeftX + joystick1RightX;
                blMotorPower = joystick1LeftY + joystick1LeftX - joystick1RightX;

                robot.frontLeft.setPower(flMotorPower * 0.35);
                robot.frontRight.setPower(frMotorPower * 0.35);
                robot.rearRight.setPower(brMotorPower * 0.35);
                robot.rearLeft.setPower(blMotorPower * 0.35);

                if (gamepad1.right_bumper) {
                    robot.claw.setPosition(0.5);
                }

                if (gamepad1.left_bumper) {
                    robot.claw.setPosition(0.2);
                }
            }
            robot.leftSlideMotor.setPower(0);
            robot.rightSlideMotor.setPower(0);
            robot.topSlideMotor.setPower(0);
            robot.bottomSlideMotor.setPower(0);
            robot.topSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bottomSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (gamepad1.y) {

            robot.rgbDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

            int slideJunctionTarget;
            slideJunctionTarget = mediumJunction;
            robot.leftSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.rightSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.topSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.bottomSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.leftSlideMotor.setMode(RUN_TO_POSITION);
            robot.rightSlideMotor.setMode(RUN_TO_POSITION);
            robot.topSlideMotor.setMode(RUN_TO_POSITION);
            robot.bottomSlideMotor.setMode(RUN_TO_POSITION);
            robot.leftSlideMotor.setPower(1);
            robot.rightSlideMotor.setPower(1);
            robot.topSlideMotor.setPower(1);
            robot.bottomSlideMotor.setPower(1);

            while (robot.leftSlideMotor.isBusy() && robot.rightSlideMotor.isBusy() && robot.topSlideMotor.isBusy() && robot.bottomSlideMotor.isBusy()) {

                telemetry.addData("leftMotor", robot.leftSlideMotor.getCurrentPosition());
                telemetry.addData("rightMotor", robot.leftSlideMotor.getCurrentPosition());
                telemetry.addData("topMotor", robot.leftSlideMotor.getCurrentPosition());
                telemetry.addData("bottomMotor", robot.leftSlideMotor.getCurrentPosition());
                telemetry.update();

                joystick1LeftX = gamepad1.left_stick_x;
                joystick1LeftY = gamepad1.left_stick_y;
                joystick1RightX = gamepad1.right_stick_x;

                flMotorPower = joystick1LeftY - joystick1LeftX - joystick1RightX;
                frMotorPower = joystick1LeftY + joystick1LeftX + joystick1RightX;
                brMotorPower = joystick1LeftY - joystick1LeftX + joystick1RightX;
                blMotorPower = joystick1LeftY + joystick1LeftX - joystick1RightX;

                robot.frontLeft.setPower(flMotorPower * 0.35);
                robot.frontRight.setPower(frMotorPower * 0.35);
                robot.rearRight.setPower(brMotorPower * 0.35);
                robot.rearLeft.setPower(blMotorPower * 0.35);

                if (gamepad1.right_bumper) {
                    robot.claw.setPosition(0.5);
                }

                if (gamepad1.left_bumper) {
                    robot.claw.setPosition(0.2);
                }
            }
            robot.leftSlideMotor.setPower(0);
            robot.rightSlideMotor.setPower(0);
            robot.topSlideMotor.setPower(0);
            robot.bottomSlideMotor.setPower(0);
            robot.topSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bottomSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (gamepad1.x) {

            robot.rgbDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

            int slideJunctionTarget;
            slideJunctionTarget = highJunction;
            robot.leftSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.rightSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.topSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.bottomSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.leftSlideMotor.setMode(RUN_TO_POSITION);
            robot.rightSlideMotor.setMode(RUN_TO_POSITION);
            robot.topSlideMotor.setMode(RUN_TO_POSITION);
            robot.bottomSlideMotor.setMode(RUN_TO_POSITION);
            robot.leftSlideMotor.setPower(1);
            robot.rightSlideMotor.setPower(1);
            robot.topSlideMotor.setPower(1);
            robot.bottomSlideMotor.setPower(1);

            while (robot.leftSlideMotor.isBusy() && robot.rightSlideMotor.isBusy() && robot.topSlideMotor.isBusy() && robot.bottomSlideMotor.isBusy()) {

                telemetry.addData("leftMotor", robot.leftSlideMotor.getCurrentPosition());
                telemetry.addData("rightMotor", robot.leftSlideMotor.getCurrentPosition());
                telemetry.addData("topMotor", robot.leftSlideMotor.getCurrentPosition());
                telemetry.addData("bottomMotor", robot.leftSlideMotor.getCurrentPosition());
                telemetry.update();

                joystick1LeftX = gamepad1.left_stick_x;
                joystick1LeftY = gamepad1.left_stick_y;
                joystick1RightX = gamepad1.right_stick_x;

                flMotorPower = joystick1LeftY - joystick1LeftX - joystick1RightX;
                frMotorPower = joystick1LeftY + joystick1LeftX + joystick1RightX;
                brMotorPower = joystick1LeftY - joystick1LeftX + joystick1RightX;
                blMotorPower = joystick1LeftY + joystick1LeftX - joystick1RightX;

                robot.frontLeft.setPower(flMotorPower * 0.35);
                robot.frontRight.setPower(frMotorPower * 0.35);
                robot.rearRight.setPower(brMotorPower * 0.35);
                robot.rearLeft.setPower(blMotorPower * 0.35);

                if (gamepad1.right_bumper) {
                    robot.claw.setPosition(0.5);
                }

                if (gamepad1.left_bumper) {
                    robot.claw.setPosition(0.2);
                }
            }
            robot.leftSlideMotor.setPower(0);
            robot.rightSlideMotor.setPower(0);
            robot.topSlideMotor.setPower(0);
            robot.bottomSlideMotor.setPower(0);
            robot.topSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bottomSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (gamepad1.left_stick_button) {

            int slideJunctionTarget;
            slideJunctionTarget = neutralPosition;
            robot.leftSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.rightSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.topSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.bottomSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.claw.setPosition(0.2);
            robot.leftSlideMotor.setMode(RUN_TO_POSITION);
            robot.rightSlideMotor.setMode(RUN_TO_POSITION);
            robot.topSlideMotor.setMode(RUN_TO_POSITION);
            robot.bottomSlideMotor.setMode(RUN_TO_POSITION);
            robot.leftSlideMotor.setPower(1);
            robot.rightSlideMotor.setPower(1);
            robot.topSlideMotor.setPower(1);
            robot.bottomSlideMotor.setPower(1);

            while (robot.leftSlideMotor.isBusy() && robot.rightSlideMotor.isBusy() && robot.topSlideMotor.isBusy() && robot.bottomSlideMotor.isBusy()) {

                telemetry.addData("leftMotor", robot.leftSlideMotor.getCurrentPosition());
                telemetry.addData("rightMotor", robot.leftSlideMotor.getCurrentPosition());
                telemetry.addData("topMotor", robot.leftSlideMotor.getCurrentPosition());
                telemetry.addData("bottomMotor", robot.leftSlideMotor.getCurrentPosition());
                telemetry.update();

                joystick1LeftX = gamepad1.left_stick_x;
                joystick1LeftY = gamepad1.left_stick_y;
                joystick1RightX = gamepad1.right_stick_x;

                flMotorPower = joystick1LeftY - joystick1LeftX - joystick1RightX;
                frMotorPower = joystick1LeftY + joystick1LeftX + joystick1RightX;
                brMotorPower = joystick1LeftY - joystick1LeftX + joystick1RightX;
                blMotorPower = joystick1LeftY + joystick1LeftX - joystick1RightX;

                robot.frontLeft.setPower(flMotorPower * 0.35);
                robot.frontRight.setPower(frMotorPower * 0.35);
                robot.rearRight.setPower(brMotorPower * 0.35);
                robot.rearLeft.setPower(blMotorPower * 0.35);

                if (gamepad1.right_bumper) {
                    robot.claw.setPosition(0.5);
                }

                if (gamepad1.left_bumper) {
                    robot.claw.setPosition(0.2);
                }
            }
            robot.leftSlideMotor.setPower(0);
            robot.rightSlideMotor.setPower(0);
            robot.topSlideMotor.setPower(0);
            robot.bottomSlideMotor.setPower(0);
            robot.topSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bottomSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.claw.setPosition(0.5);
        }

        if (gamepad1.right_stick_button) {

            int slideJunctionTarget;
            slideJunctionTarget = neutralPosition;
            robot.leftSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.rightSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.topSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.bottomSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.claw.setPosition(0.2);
            robot.leftSlideMotor.setMode(RUN_TO_POSITION);
            robot.rightSlideMotor.setMode(RUN_TO_POSITION);
            robot.topSlideMotor.setMode(RUN_TO_POSITION);
            robot.bottomSlideMotor.setMode(RUN_TO_POSITION);
            robot.leftSlideMotor.setPower(1);
            robot.rightSlideMotor.setPower(1);
            robot.topSlideMotor.setPower(1);
            robot.bottomSlideMotor.setPower(1);

            while (robot.leftSlideMotor.isBusy() && robot.rightSlideMotor.isBusy() && robot.topSlideMotor.isBusy() && robot.bottomSlideMotor.isBusy()) {

                joystick1LeftX = gamepad1.left_stick_x;
                joystick1LeftY = gamepad1.left_stick_y;
                joystick1RightX = gamepad1.right_stick_x;

                flMotorPower = joystick1LeftY - joystick1LeftX - joystick1RightX;
                frMotorPower = joystick1LeftY + joystick1LeftX + joystick1RightX;
                brMotorPower = joystick1LeftY - joystick1LeftX + joystick1RightX;
                blMotorPower = joystick1LeftY + joystick1LeftX - joystick1RightX;

                robot.frontLeft.setPower(flMotorPower * 0.35);
                robot.frontRight.setPower(frMotorPower * 0.35);
                robot.rearRight.setPower(brMotorPower * 0.35);
                robot.rearLeft.setPower(blMotorPower * 0.35);

                if (gamepad1.right_bumper) {
                    robot.claw.setPosition(0.5);
                }

                if (gamepad1.left_bumper) {
                    robot.claw.setPosition(0.2);
                }
            }
            robot.leftSlideMotor.setPower(0);
            robot.rightSlideMotor.setPower(0);
            robot.topSlideMotor.setPower(0);
            robot.bottomSlideMotor.setPower(0);
            robot.topSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bottomSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (gamepad1.dpad_left) {

            robot.rgbDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);

            int slideJunctionTarget;
            slideJunctionTarget = conePosition5;
            robot.leftSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.rightSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.topSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.bottomSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.leftSlideMotor.setMode(RUN_TO_POSITION);
            robot.rightSlideMotor.setMode(RUN_TO_POSITION);
            robot.topSlideMotor.setMode(RUN_TO_POSITION);
            robot.bottomSlideMotor.setMode(RUN_TO_POSITION);
            robot.leftSlideMotor.setPower(1);
            robot.rightSlideMotor.setPower(1);
            robot.topSlideMotor.setPower(1);
            robot.bottomSlideMotor.setPower(1);

            while (robot.leftSlideMotor.isBusy() && robot.rightSlideMotor.isBusy() && robot.topSlideMotor.isBusy() && robot.bottomSlideMotor.isBusy()) {
                joystick1LeftX = gamepad1.left_stick_x;
                joystick1LeftY = gamepad1.left_stick_y;
                joystick1RightX = gamepad1.right_stick_x;

                flMotorPower = joystick1LeftY - joystick1LeftX - joystick1RightX;
                frMotorPower = joystick1LeftY + joystick1LeftX + joystick1RightX;
                brMotorPower = joystick1LeftY - joystick1LeftX + joystick1RightX;
                blMotorPower = joystick1LeftY + joystick1LeftX - joystick1RightX;

                robot.frontLeft.setPower(flMotorPower * 0.35);
                robot.frontRight.setPower(frMotorPower * 0.35);
                robot.rearRight.setPower(brMotorPower * 0.35);
                robot.rearLeft.setPower(blMotorPower * 0.35);

                if (gamepad1.right_bumper) {
                    robot.claw.setPosition(0.5);
                }

                if (gamepad1.left_bumper) {
                    robot.claw.setPosition(0.2);
                }
            }
            robot.leftSlideMotor.setPower(0);
            robot.rightSlideMotor.setPower(0);
            robot.topSlideMotor.setPower(0);
            robot.bottomSlideMotor.setPower(0);
            robot.topSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bottomSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (gamepad1.dpad_up) {

            robot.rgbDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);

            int slideJunctionTarget;
            slideJunctionTarget = conePosition4;
            robot.leftSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.rightSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.topSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.bottomSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.leftSlideMotor.setMode(RUN_TO_POSITION);
            robot.rightSlideMotor.setMode(RUN_TO_POSITION);
            robot.topSlideMotor.setMode(RUN_TO_POSITION);
            robot.bottomSlideMotor.setMode(RUN_TO_POSITION);
            robot.leftSlideMotor.setPower(1);
            robot.rightSlideMotor.setPower(1);
            robot.topSlideMotor.setPower(1);
            robot.bottomSlideMotor.setPower(1);

            while (robot.leftSlideMotor.isBusy() && robot.rightSlideMotor.isBusy() && robot.topSlideMotor.isBusy() && robot.bottomSlideMotor.isBusy()) {
                joystick1LeftX = gamepad1.left_stick_x;
                joystick1LeftY = gamepad1.left_stick_y;
                joystick1RightX = gamepad1.right_stick_x;

                flMotorPower = joystick1LeftY - joystick1LeftX - joystick1RightX;
                frMotorPower = joystick1LeftY + joystick1LeftX + joystick1RightX;
                brMotorPower = joystick1LeftY - joystick1LeftX + joystick1RightX;
                blMotorPower = joystick1LeftY + joystick1LeftX - joystick1RightX;

                robot.frontLeft.setPower(flMotorPower * 0.35);
                robot.frontRight.setPower(frMotorPower * 0.35);
                robot.rearRight.setPower(brMotorPower * 0.35);
                robot.rearLeft.setPower(blMotorPower * 0.35);

                if (gamepad1.right_bumper) {
                    robot.claw.setPosition(0.5);
                }

                if (gamepad1.left_bumper) {
                    robot.claw.setPosition(0.2);
                }
            }
            robot.leftSlideMotor.setPower(0);
            robot.rightSlideMotor.setPower(0);
            robot.topSlideMotor.setPower(0);
            robot.bottomSlideMotor.setPower(0);
            robot.topSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bottomSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (gamepad1.dpad_right) {

            robot.rgbDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);

            int slideJunctionTarget;
            slideJunctionTarget = conePosition3;
            robot.leftSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.rightSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.topSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.bottomSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.leftSlideMotor.setMode(RUN_TO_POSITION);
            robot.rightSlideMotor.setMode(RUN_TO_POSITION);
            robot.topSlideMotor.setMode(RUN_TO_POSITION);
            robot.bottomSlideMotor.setMode(RUN_TO_POSITION);
            robot.leftSlideMotor.setPower(1);
            robot.rightSlideMotor.setPower(1);
            robot.topSlideMotor.setPower(1);
            robot.bottomSlideMotor.setPower(1);

            while (robot.leftSlideMotor.isBusy() && robot.rightSlideMotor.isBusy() && robot.topSlideMotor.isBusy() && robot.bottomSlideMotor.isBusy()) {
                joystick1LeftX = gamepad1.left_stick_x;
                joystick1LeftY = gamepad1.left_stick_y;
                joystick1RightX = gamepad1.right_stick_x;

                flMotorPower = joystick1LeftY - joystick1LeftX - joystick1RightX;
                frMotorPower = joystick1LeftY + joystick1LeftX + joystick1RightX;
                brMotorPower = joystick1LeftY - joystick1LeftX + joystick1RightX;
                blMotorPower = joystick1LeftY + joystick1LeftX - joystick1RightX;

                robot.frontLeft.setPower(flMotorPower * 0.35);
                robot.frontRight.setPower(frMotorPower * 0.35);
                robot.rearRight.setPower(brMotorPower * 0.35);
                robot.rearLeft.setPower(blMotorPower * 0.35);

                if (gamepad1.right_bumper) {
                    robot.claw.setPosition(0.5);
                }

                if (gamepad1.left_bumper) {
                    robot.claw.setPosition(0.2);
                }
            }
            robot.leftSlideMotor.setPower(0);
            robot.rightSlideMotor.setPower(0);
            robot.topSlideMotor.setPower(0);
            robot.bottomSlideMotor.setPower(0);
            robot.topSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bottomSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (gamepad1.dpad_down) {

            robot.rgbDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);

            int slideJunctionTarget;
            slideJunctionTarget = conePosition2;
            robot.leftSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.rightSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.topSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.bottomSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.leftSlideMotor.setMode(RUN_TO_POSITION);
            robot.rightSlideMotor.setMode(RUN_TO_POSITION);
            robot.topSlideMotor.setMode(RUN_TO_POSITION);
            robot.bottomSlideMotor.setMode(RUN_TO_POSITION);
            robot.leftSlideMotor.setPower(1);
            robot.rightSlideMotor.setPower(1);
            robot.topSlideMotor.setPower(1);
            robot.bottomSlideMotor.setPower(1);

            while (robot.leftSlideMotor.isBusy() && robot.rightSlideMotor.isBusy() && robot.topSlideMotor.isBusy() && robot.bottomSlideMotor.isBusy()) {
                joystick1LeftX = gamepad1.left_stick_x;
                joystick1LeftY = gamepad1.left_stick_y;
                joystick1RightX = gamepad1.right_stick_x;

                flMotorPower = joystick1LeftY - joystick1LeftX - joystick1RightX;
                frMotorPower = joystick1LeftY + joystick1LeftX + joystick1RightX;
                brMotorPower = joystick1LeftY - joystick1LeftX + joystick1RightX;
                blMotorPower = joystick1LeftY + joystick1LeftX - joystick1RightX;

                robot.frontLeft.setPower(flMotorPower * 0.35);
                robot.frontRight.setPower(frMotorPower * 0.35);
                robot.rearRight.setPower(brMotorPower * 0.35);
                robot.rearLeft.setPower(blMotorPower * 0.35);

                if (gamepad1.right_bumper) {
                    robot.claw.setPosition(0.5);
                }

                if (gamepad1.left_bumper) {
                    robot.claw.setPosition(0.2);
                }
            }
            robot.leftSlideMotor.setPower(0);
            robot.rightSlideMotor.setPower(0);
            robot.topSlideMotor.setPower(0);
            robot.bottomSlideMotor.setPower(0);
            robot.topSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bottomSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
