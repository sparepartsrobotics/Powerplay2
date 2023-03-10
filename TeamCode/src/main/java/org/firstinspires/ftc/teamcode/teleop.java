package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "TeleOp")

public class teleop extends LinearOpMode {

    public volatile Robot robot;

    double flMotorPower = 1;
    double frMotorPower = 1;
    double brMotorPower = 1;
    double blMotorPower = 1;

    double joystick1LeftX;
    double joystick1RightX;
    double joystick1LeftY;

    double motorVelocity = 1400;

    int highJunction = 1135;
    int mediumJunction = 775;
    int lowJunction = 475;
    int beaconPosition = 24;

    int neutralPosition = 0;

    int conePosition5 = 190;
    int conePosition4 = 140;
    int conePosition3 = 90;
    int conePosition2 = 50;

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

        robot.leftSlideMotor.setMode(STOP_AND_RESET_ENCODER);
        robot.rightSlideMotor.setMode(STOP_AND_RESET_ENCODER);
        robot.topSlideMotor.setMode(STOP_AND_RESET_ENCODER);
        robot.bottomSlideMotor.setMode(STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            controls();
        }
    }

    public void controls() throws InterruptedException {

        telemetry.addData("time",time);
        telemetry.addData("leftMotor", robot.leftSlideMotor.getCurrentPosition());
        telemetry.addData("rightMotor", robot.rightSlideMotor.getCurrentPosition());
        telemetry.addData("topMotor", robot.topSlideMotor.getCurrentPosition());
        telemetry.addData("bottomMotor", robot.bottomSlideMotor.getCurrentPosition());

        //teleOp
        if(time < 85) {
            robot.rgbDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
            //ENDGAME
        } else if (time > 90) {
            robot.rgbDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            //Right BEFORE ENDGAME
        } else{
            robot.rgbDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
        }

        joystick1LeftX = gamepad1.left_stick_x;
        joystick1LeftY = gamepad1.left_stick_y;
        joystick1RightX = gamepad1.right_stick_x;

        flMotorPower = joystick1LeftY - joystick1LeftX - joystick1RightX;
        frMotorPower = joystick1LeftY + joystick1LeftX + joystick1RightX;
        blMotorPower = joystick1LeftY + joystick1LeftX - joystick1RightX;
        brMotorPower = joystick1LeftY - joystick1LeftX + joystick1RightX;

        if (gamepad1.right_trigger > 0) {
            robot.leftSlideMotor.setVelocity(motorVelocity*.7);
            robot.rightSlideMotor.setVelocity(motorVelocity*.7);
            robot.topSlideMotor.setVelocity(motorVelocity*.7);
            robot.bottomSlideMotor.setVelocity(motorVelocity*.7);
        } else {
            robot.leftSlideMotor.setPower(0);
            robot.rightSlideMotor.setPower(0);
            robot.topSlideMotor.setPower(0);
            robot.bottomSlideMotor.setPower(0);
        }

        if (gamepad1.left_bumper) {
            robot.claw.setPosition(0.5);
        }

        if (gamepad1.right_bumper) {
            robot.claw.setPosition(0.15);
        }

        if (gamepad1.left_trigger > 0)  {

            robot.leftSlideMotor.setVelocity(-.70*motorVelocity);
            robot.rightSlideMotor.setVelocity(-.70*motorVelocity);
            robot.topSlideMotor.setVelocity(-.70*motorVelocity);
            robot.bottomSlideMotor.setVelocity(-.70*motorVelocity);
        } else {
            robot.leftSlideMotor.setPower(0);
            robot.rightSlideMotor.setPower(0);
            robot.topSlideMotor.setPower(0);
            robot.bottomSlideMotor.setPower(0);
        }
        if (gamepad1.a) {

            robot.rgbDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);

            int slideJunctionTarget;
            slideJunctionTarget = beaconPosition;
            robot.leftSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.rightSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.topSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.bottomSlideMotor.setTargetPosition(slideJunctionTarget);
            robot.leftSlideMotor.setMode(RUN_TO_POSITION);
            robot.rightSlideMotor.setMode(RUN_TO_POSITION);
            robot.topSlideMotor.setMode(RUN_TO_POSITION);
            robot.bottomSlideMotor.setMode(RUN_TO_POSITION);
            robot.leftSlideMotor.setVelocity(motorVelocity);
            robot.rightSlideMotor.setVelocity(motorVelocity);
            robot.topSlideMotor.setVelocity(motorVelocity);
            robot.bottomSlideMotor.setVelocity(motorVelocity);

            while (robot.leftSlideMotor.isBusy() && robot.rightSlideMotor.isBusy() && robot.topSlideMotor.isBusy() && robot.bottomSlideMotor.isBusy()) {

                robot.rgbDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN);

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
                    robot.claw.setPosition(0.15);
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

            robot.rgbDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);

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
            robot.leftSlideMotor.setVelocity(motorVelocity);
            robot.rightSlideMotor.setVelocity(motorVelocity);
            robot.topSlideMotor.setVelocity(motorVelocity);
            robot.bottomSlideMotor.setVelocity(motorVelocity);

            while (robot.leftSlideMotor.isBusy() && robot.rightSlideMotor.isBusy() && robot.topSlideMotor.isBusy() && robot.bottomSlideMotor.isBusy()) {

                robot.rgbDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);

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
                    robot.claw.setPosition(0.15);
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
            robot.leftSlideMotor.setVelocity(motorVelocity);
            robot.rightSlideMotor.setVelocity(motorVelocity);
            robot.topSlideMotor.setVelocity(motorVelocity);
            robot.bottomSlideMotor.setVelocity(motorVelocity);

            while (robot.leftSlideMotor.isBusy() && robot.rightSlideMotor.isBusy() && robot.topSlideMotor.isBusy() && robot.bottomSlideMotor.isBusy()) {

                robot.rgbDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

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
                    robot.claw.setPosition(0.15);
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

            robot.rgbDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);

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
            robot.leftSlideMotor.setVelocity(motorVelocity);
            robot.rightSlideMotor.setVelocity(motorVelocity);
            robot.topSlideMotor.setVelocity(motorVelocity);
            robot.bottomSlideMotor.setVelocity(motorVelocity);

            long highTime = System.currentTimeMillis();
            while ((System.currentTimeMillis()- highTime)<1800 && robot.leftSlideMotor.isBusy() && robot.rightSlideMotor.isBusy() && robot.topSlideMotor.isBusy() && robot.bottomSlideMotor.isBusy()) {

                robot.rgbDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);

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
                    robot.claw.setPosition(0.15);
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
            robot.leftSlideMotor.setVelocity(motorVelocity);
            robot.rightSlideMotor.setVelocity(motorVelocity);
            robot.topSlideMotor.setVelocity(motorVelocity);
            robot.bottomSlideMotor.setVelocity(motorVelocity);

            long neutralTime = System.currentTimeMillis();
            while ((System.currentTimeMillis() - neutralTime) < 2300 && robot.leftSlideMotor.isBusy() && robot.rightSlideMotor.isBusy() && robot.topSlideMotor.isBusy() && robot.bottomSlideMotor.isBusy()) {

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
                    robot.claw.setPosition(0.15);
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

            robot.frontLeft.setPower(flMotorPower / 1.33);
            robot.frontRight.setPower(frMotorPower / 1.33);
            robot.rearRight.setPower(brMotorPower / 1.33);
            robot.rearLeft.setPower(blMotorPower / 1.33);
        } else {
            robot.frontLeft.setPower(flMotorPower * 0.35);
            robot.frontRight.setPower(frMotorPower * 0.35);
            robot.rearRight.setPower(brMotorPower * 0.35);
            robot.rearLeft.setPower(blMotorPower * 0.35);

            long neutral2Time = System.currentTimeMillis();
            while ((System.currentTimeMillis() - neutral2Time) < 2300 && robot.leftSlideMotor.isBusy() && robot.rightSlideMotor.isBusy() && robot.topSlideMotor.isBusy() && robot.bottomSlideMotor.isBusy()) {


                if (gamepad1.right_bumper) {
                    robot.claw.setPosition(0.5);
                }

                if (gamepad1.left_bumper) {
                    robot.claw.setPosition(0.15);
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
            robot.leftSlideMotor.setVelocity(motorVelocity);
            robot.rightSlideMotor.setVelocity(motorVelocity);
            robot.topSlideMotor.setPower(motorVelocity);
            robot.bottomSlideMotor.setPower(motorVelocity);

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
                    robot.claw.setPosition(0.15);
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
            robot.leftSlideMotor.setVelocity(motorVelocity);
            robot.rightSlideMotor.setVelocity(motorVelocity);
            robot.topSlideMotor.setVelocity(motorVelocity);
            robot.bottomSlideMotor.setVelocity(motorVelocity);

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
                    robot.claw.setPosition(0.15);
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
            robot.leftSlideMotor.setVelocity(motorVelocity);
            robot.rightSlideMotor.setVelocity(motorVelocity);
            robot.topSlideMotor.setVelocity(motorVelocity);
            robot.bottomSlideMotor.setVelocity(motorVelocity);

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
                    robot.claw.setPosition(0.15);
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
            robot.leftSlideMotor.setVelocity(motorVelocity);
            robot.rightSlideMotor.setVelocity(motorVelocity);
            robot.topSlideMotor.setVelocity(motorVelocity);
            robot.bottomSlideMotor.setVelocity(motorVelocity);

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
                    robot.claw.setPosition(0.15);
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
        telemetry.update();
    }
}