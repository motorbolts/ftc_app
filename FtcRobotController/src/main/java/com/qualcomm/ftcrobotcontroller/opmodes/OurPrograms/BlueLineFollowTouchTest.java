package com.qualcomm.ftcrobotcontroller.opmodes.OurPrograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 6705 on 1/6/2016.
 */
public class BlueLineFollowTouchTest extends LinearOpMode {

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
    public void runOpMode() throws  InterruptedException
    {

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

        leftPivot.setPosition(1);
        rightPivot.setPosition(0);
        leftComb.setPosition(0);
        rightComb.setPosition(1);
        trigL.setPosition(0.7);
        trigR.setPosition(0.35);
        leftCR.setPosition(0.5);
        rightCR.setPosition(0.5);
        dds.setPosition(1);

        double lineSensorValue = lineSensor.getLightDetectedRaw();


        waitForStart();

        sleep(100);

        lineSensorValue = lineSensor.getLightDetectedRaw();

        telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", lineSensorValue));

        sleep(1000);


        while (!touch.isPressed()) {

            lineSensorValue = lineSensor.getLightDetectedRaw();
//hi

            if (lineSensorValue > 15) {

                lwa.setPower(0.5);
                lwb.setPower(0.5);
                rwa.setPower(0.0);
                rwb.setPower(0.0);
            }
            else {
                lwa.setPower(0.0);
                lwb.setPower(0.0);
                rwa.setPower(.50);
                rwb.setPower(.50);
            }
        }

        lwa.setPower(0.0);
        lwb.setPower(0.0);
        rwa.setPower(.0);
        rwb.setPower(.0);

        sleep(100);

        lwa.setPower(-0.35);
        lwb.setPower(-0.35);
        rwa.setPower(-0.35);
        rwb.setPower(-0.35);

        sleep(200);

        lwa.setPower(0.0);
        lwb.setPower(0.0);
        rwa.setPower(.0);
        rwb.setPower(.0);

        sleep(100);

        dds.setPosition(0);
    }
}
