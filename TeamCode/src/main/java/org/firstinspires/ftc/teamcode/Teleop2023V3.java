package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Teleop2023V3 extends OpMode{
    Hardware hardwareMap=new Hardware();
    double test;
    @Override
    public void init() {



        //i exist and am real
        //i really exist and I really really work
        hardwareMap.backLeftMotor.setPower(0.7);
        test=gamepad1.left_stick_x;
    }

    public void init_loop() {

    }

    public void start() {

    }
    @Override
    public void loop() {

    }
}
