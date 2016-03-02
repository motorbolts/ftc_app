/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes.OurPrograms;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


//yo

public class BACKBlueDDS_NSR_SETPICK extends LinearOpMode {
    DcMotor rwa;
    DcMotor rwb;
    DcMotor liftL;
    DcMotor liftR;
    Servo swivel;
    Servo dump;
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
    Servo holdC;
    //  Servo leftPivot;
    //  Servo rightPivot;
    OpticalDistanceSensor lineSensor;
    TouchSensor touch;
    GyroSensor Gyro;
    ElapsedTime timer;

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
    //liftR.setDirection(DcMotor.Direction.REVERSE);
    //liftL.setDirection(DcMotor.Direction.REVERSE);
    collector = hardwareMap.dcMotor.get("collector");
    //rightCR = hardwareMap.servo.get("rightCR");
    //leftCR = hardwareMap.servo.get("leftCR");
    swivel = hardwareMap.servo.get("swivel");
    dump = hardwareMap.servo.get("dump");
    trigL = hardwareMap.servo.get("trigL");
    trigR = hardwareMap.servo.get("trigR");
    dds = hardwareMap.servo.get("dds");
    holdL = hardwareMap.servo.get("holdL");
    holdR = hardwareMap.servo.get("holdR");
    holdC = hardwareMap.servo.get("holdC");
    //leftPivot = hardwareMap.servo.get("leftPivot");
//rightPivot = hardwareMap.servo.get("rightPivot");
    lineSensor = hardwareMap.opticalDistanceSensor.get("dist1");
    touch = hardwareMap.touchSensor.get("touch");
    Gyro  = hardwareMap.gyroSensor.get("Gyro");


    //  leftPivot.setPosition(1);
    // rightPivot.setPosition(0);
    dump.setPosition(0);
           trigL.setPosition(0.8);
    trigR.setPosition(0.05);
    //   leftCR.setPosition(0.5);
    //     rightCR.setPosition(0.5);
    dds.setPosition(1);
    holdL.setPosition(0.75);
    holdR.setPosition(0.05);
    holdC.setPosition(1);
    double lineSensorValue;

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

    Gyro.calibrate();


    // wait for the start button to be pressed
    waitForStart();

    timer = new ElapsedTime();

    collector.setPower(0);

    while(rwa.getCurrentPosition() > -7700 && timer.time() < 15)
    {
        rwa.setPower(-0.75);
        rwb.setPower(-0.75);
        lwa.setPower(-0.75);
        lwb.setPower(-0.75);
    }

    rwa.setPower(0);
    rwb.setPower(0);
    lwa.setPower(0);
    lwb.setPower(0);

    sleep(100);


    heading = Gyro.getHeading();
    telemetry.addData("heading",heading);

    heading = 359;
    sleep(100);

    while(heading > 225) {
        heading = Gyro.getHeading();
        //          integratedHeading = Gyro.getIntegratedZValue();

        telemetry.addData("heading", heading);
        //        telemetry.addData("Integrated", integratedHeading);

        targetHeading = 225;

        headingError = targetHeading - heading;

        drivesteering = driveGain * headingError;

        if (drivesteering > 1) {
            drivesteering = 1;
            telemetry.addData("Caught illegal value", "reset drivesteering to 1");
        }


        leftPower = midPower + drivesteering;
        rightPower = midPower + drivesteering;

        if (leftPower > minPower) {
            leftPower = minPower;
        }
        if (rightPower < minPower) {
            rightPower = minPower;
        }

        lwa.setPower(leftPower);
        lwb.setPower(leftPower);
        rwa.setPower(rightPower);
        rwb.setPower(rightPower);
    }

    while(heading < 225)
    {
        heading = Gyro.getHeading();
        //          integratedHeading = Gyro.getIntegratedZValue();

        telemetry.addData("heading", heading);
        //        telemetry.addData("Integrated", integratedHeading);

        targetHeading = 225;

        headingError = targetHeading - heading;

        drivesteering = driveGain * headingError;

        if (drivesteering > 1) {
            drivesteering = 1;
            telemetry.addData("Caught illegal value", "reset drivesteering to 1");
        }

        rightPower = midPower - drivesteering;
        leftPower = midPower + drivesteering;

        if (rightPower > minPower) {
            rightPower = minPower;
        }
        if (leftPower < minPower) {
            leftPower = minPower;
        }

        lwa.setPower(leftPower);
        lwb.setPower(leftPower);
        rwa.setPower(rightPower);
        rwb.setPower(rightPower);
    }


    lwa.setPower(1);
    lwb.setPower(1);
    rwa.setPower(1);
    rwb.setPower(1);
    sleep(400);

    lwa.setPower(0);
    lwb.setPower(0);
    rwa.setPower(0);
    rwb.setPower(0);
    telemetry.addData("Event", "Done");
    sleep(100);



    while (!touch.isPressed() && timer.time() < 20) {

        lineSensorValue = lineSensor.getLightDetectedRaw();

        if (lineSensorValue < 10) {

            lwa.setPower(0.5);
            lwb.setPower(0.5);
            rwa.setPower(0.0);
            rwb.setPower(0.0);
        } else {
            lwa.setPower(0.0);
            lwb.setPower(0.0);
            rwa.setPower(0.5);
            rwb.setPower(0.5);
        }
    }

    lwa.setPower(0.0);
    lwb.setPower(0.0);
    rwa.setPower(0.0);
    rwb.setPower(0.0);
    sleep(100);


    collector.setPower(0.0);
    sleep(100);

    if(timer.time() < 20) {
        lwa.setPower(-0.35);
        lwb.setPower(-0.35);
        rwa.setPower(-0.35);
        rwb.setPower(-0.35);

        sleep(225);


        lwa.setPower(0.0);
        lwb.setPower(0.0);
        rwa.setPower(0.0);
        rwb.setPower(0.0);

        sleep(100);
        dds.setPosition(0);

        sleep(900);
    }

    lwa.setPower(-1);
    lwb.setPower(-1);
    rwa.setPower(-1);
    rwb.setPower(-1);

    sleep(500);


    lwa.setPower(0.0);
    lwb.setPower(0.0);
    rwa.setPower(0.0);
    rwb.setPower(0.0);
    sleep(100);

    while(heading > 120)
    {
        heading = Gyro.getHeading();
        telemetry.addData("heading", heading);

        rwa.setPower(0.5);
        rwb.setPower(0.5);
        lwa.setPower(-0.5);
        lwb.setPower(-0.5);
    }


    lwa.setPower(0.0);
    lwb.setPower(0.0);
    rwa.setPower(0.0);
    rwb.setPower(0.0);
    sleep(500);
    dds.setPosition(1);
    lwa.setPower(0.5);
    lwb.setPower(0.5);
    rwa.setPower(0.5);
    rwb.setPower(0.5);
    sleep(1500);
    dds.setPosition(1);

    lwa.setPower(0.0);
    lwb.setPower(0.0);
    rwa.setPower(0.0);
    rwb.setPower(0.0);

    sleep(500);

}
}
