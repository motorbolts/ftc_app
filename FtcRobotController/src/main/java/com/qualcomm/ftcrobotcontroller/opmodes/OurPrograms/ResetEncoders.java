package com.qualcomm.ftcrobotcontroller.opmodes.OurPrograms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by 6705 on 3/7/2016.
 */
public class ResetEncoders extends OpMode {

    DcMotor rwa;
    DcMotor lwa;

    @Override
    public void init() {

        lwa = hardwareMap.dcMotor.get("leftwheelA");
        rwa = hardwareMap.dcMotor.get("rightwheelA");
    }

    @Override
    public void loop(){

        telemetry.addData("Left Encoder Value", lwa.getCurrentPosition());
        telemetry.addData("Right Encoder Value", rwa.getCurrentPosition());
        lwa.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        rwa.setMode(DcMotorController.RunMode.RESET_ENCODERS);

    }


}
