package com.qualcomm.ftcrobotcontroller.opmodes.OurPrograms;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by waltboettge on 3/3/16.
 */
public class GyroStraightDrive extends LinearOpMode {

    DcMotor rwa; //rightwheels
    DcMotor rwb;

    DcMotor lwa; //leftwheels
    DcMotor lwb;

    DcMotor liftL;
    DcMotor liftR;

    Servo dump; //servo to score with dumper
    Servo swivel; //servo that 'spreads' with dumper

    Servo holdR; // churro holders
    Servo holdL;

    Servo trigL; //zip-line triggers
    Servo trigR;

    Servo holdC; //servo to guard lift

    DcMotor collector; //bottom collector motor
    DcMotor collector2; //top collector motor

    Servo dds;

    ModernRoboticsI2cGyro Gyro;

    double heading = 0;
    double drivesteering;
    double driveGain = 0.025;
    double leftPower;
    double rightPower;
    double midPower = -0.75;
    double minPowerPositive = 0.2;
    double minPowerNegative = -0.2;

    @Override
    public void runOpMode() throws InterruptedException {

        lwa = hardwareMap.dcMotor.get("leftwheelA"); //leftwheels
        lwb = hardwareMap.dcMotor.get("leftwheelB");

        rwa = hardwareMap.dcMotor.get("rightwheelA"); //rightwheels
        rwb = hardwareMap.dcMotor.get("rightwheelB");
        rwa.setDirection(DcMotor.Direction.REVERSE);
        rwb.setDirection(DcMotor.Direction.REVERSE);

        liftL = hardwareMap.dcMotor.get("liftL");
        liftR = hardwareMap.dcMotor.get("liftR");
        liftR.setDirection(DcMotor.Direction.REVERSE);

        collector = hardwareMap.dcMotor.get("collector");
        collector2 = hardwareMap.dcMotor.get("collector2");

        dump = hardwareMap.servo.get("dump"); //dumper control
        swivel = hardwareMap.servo.get("swivel");

        trigL = hardwareMap.servo.get("trigL"); //triggers
        trigR = hardwareMap.servo.get("trigR");

        dds = hardwareMap.servo.get("dds");

        holdL = hardwareMap.servo.get("holdL"); //churro holder
        holdR = hardwareMap.servo.get("holdR");

        holdC = hardwareMap.servo.get("holdC"); //lift holder

        Gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("Gyro");
        //***INIT***//

        dump.setPosition(0);
        swivel.setPosition(1);

        trigL.setPosition(0.8);
        trigR.setPosition(0.05);

        dds.setPosition(1);

        holdL.setPosition(0.75);
        holdR.setPosition(0.05);

        holdC.setPosition(1);

        Gyro.calibrate();

        telemetry.addData("Event", "Waiting for Start");

        waitForStart();

        while(rwa.getCurrentPosition() < 5000)
        {
            waitOneFullHardwareCycle();

            telemetry.addData("Encoder Value", rwa.getCurrentPosition());

            heading = Gyro.getIntegratedZValue();

            telemetry.addData("Heading", heading);

            drivesteering = driveGain * heading;

            rightPower = midPower - drivesteering;

            leftPower = midPower + drivesteering;

            telemetry.addData("leftPower", leftPower);
            telemetry.addData("rightPower", rightPower);

            lwa.setPower(leftPower);
            lwb.setPower(leftPower);
            rwa.setPower(rightPower);
            rwb.setPower(rightPower);
        }

        waitOneFullHardwareCycle();

        lwa.setPower(0);
        lwb.setPower(0);
        rwa.setPower(0);
        rwb.setPower(0);

    }
}
