package com.qualcomm.ftcrobotcontroller.opmodes.OurPrograms;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.GyroSensor;
//import com.qualcomm.hardware.ModernRoboticsI2cGyro;

public class GyroTestPControlCW extends LinearOpMode {

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

    GyroSensor Gyro;

    double heading = 0;
    //    double integratedHeading = Gyro.getIntegratedZValue();
    double headingError;
    double targetHeading;
    double drivesteering;
    double driveGain = 0.005;
    double leftPower;
    double rightPower;
    double midPower = 0;
    double minPower = 0.2;

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
        Gyro = hardwareMap.gyroSensor.get("Gyro");
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


        waitForStart();

        while(heading < 90 || heading > 90)
        {
            heading = Gyro.getHeading();
            telemetry.addData("heading",heading);

            while(heading < 90) {
                heading = Gyro.getHeading();
                //          integratedHeading = Gyro.getIntegratedZValue();

                telemetry.addData("heading", heading);
                //        telemetry.addData("Integrated", integratedHeading);

                targetHeading = 90;

                headingError = targetHeading - heading;

                drivesteering = driveGain * headingError;

                if (drivesteering > 1) {
                    drivesteering = 1;
                    telemetry.addData("Caught illegal value", "reset drivesteering to 1");
                }


                leftPower = midPower + drivesteering;
                rightPower = midPower - drivesteering;

                if (leftPower < minPower) {
                    leftPower = minPower;
                }
                if (rightPower > minPower) {
                    rightPower = minPower;
                }

                lwa.setPower(leftPower);
                lwb.setPower(leftPower);
                rwa.setPower(rightPower);
                rwb.setPower(rightPower);
            }

            while(heading > 90)
            {
                heading = Gyro.getHeading();
                //          integratedHeading = Gyro.getIntegratedZValue();

                telemetry.addData("heading", heading);
                //        telemetry.addData("Integrated", integratedHeading);

                targetHeading = 90;

                headingError = targetHeading - heading;

                drivesteering = driveGain * headingError;

                if (drivesteering > 1) {
                    drivesteering = 1;
                    telemetry.addData("Caught illegal value", "reset drivesteering to 1");
                }

                rightPower = midPower + drivesteering;
                leftPower = midPower - drivesteering;

                if (rightPower < minPower) {
                    rightPower = minPower;
                }
                if (leftPower > minPower) {
                    leftPower = minPower;
                }

                lwa.setPower(leftPower);
                lwb.setPower(leftPower);
                rwa.setPower(rightPower);
                rwb.setPower(rightPower);
            }
        }

        lwa.setPower(0);
        lwb.setPower(0);
        rwa.setPower(0);
        rwb.setPower(0);
        telemetry.addData("Event", "Done");
        sleep(100);

        lwa.setPower(0);
        lwb.setPower(0);
        rwa.setPower(0);
        rwb.setPower(0);

    }

}