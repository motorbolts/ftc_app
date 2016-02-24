package com.qualcomm.ftcrobotcontroller.opmodes.OurPrograms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


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
    // Servo leftCR;
    // Servo rightCR;
    Servo dds;
    Servo holdL;
    Servo holdR;
    DcMotor lwa;
    DcMotor lwb;
    //  Servo leftPivot;
    //  Servo rightPivot;
    OpticalDistanceSensor lineSensor;
    TouchSensor touch;
    ElapsedTime timer;

    @Override
    public void init() {

        lwa = hardwareMap.dcMotor.get("leftwheelA");
        lwb = hardwareMap.dcMotor.get("leftwheelB");
        rwa = hardwareMap.dcMotor.get("rightwheelA");
        rwb = hardwareMap.dcMotor.get("rightwheelB");
        rwa.setDirection(DcMotor.Direction.REVERSE);
        rwb.setDirection(DcMotor.Direction.REVERSE);
        liftL = hardwareMap.dcMotor.get("liftL");
        liftR = hardwareMap.dcMotor.get("liftR");
        //liftR.setDirection(DcMotor.Direction.REVERSE);
        //liftL.setDirection(DcMotor.Direction.REVERSE);
        collector = hardwareMap.dcMotor.get("collector");
        //rightCR = hardwareMap.servo.get("rightCR");
        //leftCR = hardwareMap.servo.get("leftCR");
        leftComb = hardwareMap.servo.get("leftComb");
        rightComb = hardwareMap.servo.get("rightComb");
        trigL = hardwareMap.servo.get("trigL");
        trigR = hardwareMap.servo.get("trigR");
        dds = hardwareMap.servo.get("dds");
        holdL = hardwareMap.servo.get("holdL");
        holdR = hardwareMap.servo.get("holdR");
        lineSensor = hardwareMap.opticalDistanceSensor.get("dist1");


    }
    @Override
    public void loop()
    {
        double lineSensorValue = lineSensor.getLightDetectedRaw();

        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", lineSensorValue));
    }

}
