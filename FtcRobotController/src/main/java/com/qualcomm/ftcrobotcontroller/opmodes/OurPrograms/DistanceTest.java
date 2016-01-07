package com.qualcomm.ftcrobotcontroller.opmodes.OurPrograms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


public class DistanceTest extends OpMode {

    DcMotor rwa;
    DcMotor rwb;
    DcMotor liftL;
    DcMotor liftR;
    Servo leftComb;
    Servo rightComb;
    Servo trigL;
    Servo trigR;
    DcMotor collector;
    Servo leftCR;
    Servo rightCR;
    Servo dds;
    DcMotor lwa;
    DcMotor lwb;
    Servo leftPivot;
    Servo rightPivot;
    OpticalDistanceSensor lineSensor;
    TouchSensor touch;

    @Override
    public void init() {

        touch = hardwareMap.touchSensor.get("touch");
        lwa = hardwareMap.dcMotor.get("leftwheelA");
        lwb = hardwareMap.dcMotor.get("leftwheelB");
        rwa = hardwareMap.dcMotor.get("rightwheelA");
        rwb = hardwareMap.dcMotor.get("rightwheelB");
        rwa.setDirection(DcMotor.Direction.REVERSE);
        rwb.setDirection(DcMotor.Direction.REVERSE);
        liftL = hardwareMap.dcMotor.get("liftL");
        liftR = hardwareMap.dcMotor.get("liftR");
        liftR.setDirection(DcMotor.Direction.REVERSE);
        liftL.setDirection(DcMotor.Direction.REVERSE);
        collector = hardwareMap.dcMotor.get("collector");
        rightCR = hardwareMap.servo.get("rightCR");
        leftCR = hardwareMap.servo.get("leftCR");
        leftComb = hardwareMap.servo.get("leftComb");
        rightComb = hardwareMap.servo.get("rightComb");
        trigL = hardwareMap.servo.get("trigL");
        trigR = hardwareMap.servo.get("trigR");
        dds = hardwareMap.servo.get("dds");
        leftPivot = hardwareMap.servo.get("leftPivot");
        rightPivot = hardwareMap.servo.get("rightPivot");
        lineSensor = hardwareMap.opticalDistanceSensor.get("dist1");
        touch = hardwareMap.touchSensor.get("touch");
    }
    @Override
    public void loop()
    {
        double lineSensor = touch.getValue();

        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", lineSensor));
    }

}
