package com.qualcomm.ftcrobotcontroller.opmodes.OurPrograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.GyroSensor;

public class AutoDDS extends LinearOpMode{

    DcMotor rwa; // P0 port 1
    DcMotor rwb; // P0 port 2
    DcMotor liftL; // P1 port 1
    DcMotor liftR; // P1 port 2
    Servo leftComb; // P2 channel 1
    Servo rightComb; // P2 channel 2
    Servo trigL; // P2 channel 3
    Servo trigR; // P2 channel 4
    DcMotor scoopArm; // P3 port 1
    //DcMotor winch; - Winch not currently on robot will be //P3 port 2
    //Servo leftCR; // Not on robot- P4 channel 1
    //Servo rightCR; //Not on robot- P4 channel 2
    Servo wrist;
    Servo ddspivot; // P4 channel 3
    Servo ddsclaw; // P4 channel 4
    DcMotor lwa; // P5 port 1
    DcMotor lwb; // P5 port 2

    GyroSensor Gyro;

    /*
    final static int ENCODER_CPR = 1440;     //Encoder Counts per Revolution
    final static double GEAR_RATIO = 2;      //Gear Ratio
    final static int WHEEL_DIAMETER = 4;     //Diameter of the wheel in inches
    final static int DISTANCE = 24;          //Distance in inches to drive

    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    final static double ROTATIONS = DISTANCE / CIRCUMFERENCE;
    final static double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;
    */
    @Override
    public void runOpMode() throws InterruptedException {

        lwa = hardwareMap.dcMotor.get("leftwheelA");
        lwb = hardwareMap.dcMotor.get("leftwheelB");
        rwa = hardwareMap.dcMotor.get("rightwheelA");
        rwb = hardwareMap.dcMotor.get("rightwheelB");
        rwa.setDirection(DcMotor.Direction.REVERSE);
        rwb.setDirection(DcMotor.Direction.REVERSE);

        liftL = hardwareMap.dcMotor.get("liftL");
        liftR = hardwareMap.dcMotor.get("liftR");
        liftR.setDirection(DcMotor.Direction.REVERSE);

        scoopArm = hardwareMap.dcMotor.get("scoopArm");
        scoopArm.setDirection(DcMotor.Direction.REVERSE);

        wrist = hardwareMap.servo.get("wrist");

        //rightCR = hardwareMap.servo.get("rightCR");

        leftComb = hardwareMap.servo.get("leftComb");
        rightComb = hardwareMap.servo.get("rightComb");

        trigL = hardwareMap.servo.get("trigL");
        trigR = hardwareMap.servo.get("trigR");

        ddspivot = hardwareMap.servo.get("ddspivot");
        ddsclaw = hardwareMap.servo.get("ddsclaw");

        Gyro = hardwareMap.gyroSensor.get("gyro");

        //lwa.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        lwb.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        rwb.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        rwa.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        double motorspeed = 0.8;

        //int zVal = 0;
        int heading = 0;

        Gyro.calibrate();

        if(Gyro.isCalibrating())
        {
            waitOneFullHardwareCycle();
        }

        // Wait for the start button to be pressed
        waitForStart();

        ddspivot.setPosition(1);
        ddsclaw.setPosition(0.1);
        sleep(500);

        //24 in. is 1.9 rotations is 2750 counts
        while(rwa.getCurrentPosition() < 2750){

            lwa.setPower(motorspeed);
            lwb.setPower(motorspeed);
            rwb.setPower(motorspeed);
            rwa.setPower(motorspeed);
            waitOneFullHardwareCycle();
        }

        //stop
        lwa.setPower(0);
        lwb.setPower(0);
        rwb.setPower(0);
        rwa.setPower(0);

        sleep(2000);

        //unpack arm/scoop
        scoopArm.setPower(0.2);
        sleep(800);    //UNTESTED VALUE
        scoopArm.setPower(0);

        wrist.setPosition(0.8);
        sleep(200);

        //keep going
        //70 in. is 5.57 rotations is 8022 counts
        while(rwa.getCurrentPosition() < 8022){

            lwa.setPower(motorspeed);
            lwb.setPower(motorspeed);
            rwb.setPower(motorspeed);
            rwa.setPower(motorspeed);
            waitOneFullHardwareCycle();
        }

        //stop
        lwa.setPower(0);
        lwb.setPower(0);
        rwb.setPower(0);
        rwa.setPower(0);

        //turn 45 degrees w/ gyro
        heading = Gyro.getHeading();

        while(heading < 45){

            lwa.setPower(motorspeed);
            lwb.setPower(motorspeed);
            rwb.setPower(-motorspeed);
            rwa.setPower(-motorspeed);
            waitOneFullHardwareCycle();

        }

        //stop
        lwa.setPower(0);
        lwb.setPower(0);
        rwb.setPower(0);
        rwa.setPower(0);

        sleep(100);

    }
}

