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
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;



public class BlueDDS_CH extends LinearOpMode {

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
  OpticalDistanceSensor distanceSensor;

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
    distanceSensor = hardwareMap.opticalDistanceSensor.get("dist2");


    leftPivot.setPosition(0.5);
    rightPivot.setPosition(0.5);
    leftComb.setPosition(0);
    rightComb.setPosition(1);
    trigL.setPosition(0.7);
    trigR.setPosition(0.35);
    leftCR.setPosition(0.5);
    rightCR.setPosition(0.5);

    double lineSensorValue = lineSensor.getLightDetectedRaw();
    double distanceSensorValue = distanceSensor.getLightDetectedRaw();

    // wait for the start button to be pressed
    waitForStart();

    while( lineSensorValue < 30)
    {
      lineSensorValue = lineSensor.getLightDetectedRaw();
      rwa.setPower(50);
      rwb.setPower(50);
      lwa.setPower(50);
      lwb.setPower(50);
      sleep(20);
    }

    rwa.setPower(0);
    rwb.setPower(0);
    lwa.setPower(0);
    lwb.setPower(0);

    sleep(100);

    lineSensorValue = lineSensor.getLightDetectedRaw();

    if(lineSensorValue > 30)
    {
      while(lineSensorValue > 30)
      {
        lineSensorValue = lineSensor.getLightDetectedRaw();
        rwa.setPower(50);
        rwb.setPower(50);
        lwa.setPower(50);
        lwb.setPower(50);
        sleep(20);
      }
    }

    rwa.setPower(0);
    rwb.setPower(0);
    lwa.setPower(0);
    lwb.setPower(0);

    sleep(100);

    lwa.setPower(50);
    lwb.setPower(50);
    sleep(100);

    while(distanceSensorValue < 17)
    {
      lineSensorValue = lineSensor.getLightDetectedRaw();
      distanceSensorValue = distanceSensor.getLightDetectedRaw();

      if(lineSensorValue > 30)
      {
        rwa.setPower(20);
        rwb.setPower(20);
        lwa.setPower(50);
        lwb.setPower(50);
        sleep(20);
      }

      if(lineSensorValue < 30)
      {
        rwa.setPower(50);
        rwb.setPower(50);
        lwa.setPower(20);
        lwb.setPower(20);
        sleep(20);
      }
    }

  }
}
