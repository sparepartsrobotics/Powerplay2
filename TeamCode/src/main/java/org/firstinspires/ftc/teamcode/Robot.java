package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;



public class Robot {

    //Drive Motors
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor rearRight;
    public DcMotor rearLeft;
    //Linear Slide Motors
    public DcMotor leftSlideMotor;
    public DcMotor rightSlideMotor;
    public DcMotor topSlideMotor;
    public DcMotor bottomSlideMotor;
    //servo for claw
    public Servo claw;

    public RevBlinkinLedDriver rgbDriver;

    public Robot(HardwareMap hardwareMap){

        //Drive Motors                    (names on REV driver hub in "green")
        frontLeft = hardwareMap.dcMotor.get("lfWheel");
        frontRight = hardwareMap.dcMotor.get("rfWheel");
        rearLeft = hardwareMap.dcMotor.get("lrWheel");
        rearRight = hardwareMap.dcMotor.get("rrWheel");
        //Linear Slide Motors
        leftSlideMotor = hardwareMap.dcMotor.get("leftSlideMotor");
        rightSlideMotor = hardwareMap.dcMotor.get("rightSlideMotor");
        topSlideMotor = hardwareMap.dcMotor.get("topSlideMotor");
        bottomSlideMotor = hardwareMap.dcMotor.get("bottomSlideMotor");
        //Servo for claw
        claw = hardwareMap.servo.get("claw");
        //LED lights
        rgbDriver = hardwareMap.get(RevBlinkinLedDriver.class,"ledDriver");
        //reverse drive motors
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
    }
}

