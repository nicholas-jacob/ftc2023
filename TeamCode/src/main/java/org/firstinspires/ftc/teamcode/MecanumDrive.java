package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class MecanumDrive {
    public DcMotorEx frontRightMotor;
    public DcMotorEx frontLeftMotor;
    public DcMotorEx backRightMotor;
    public DcMotorEx backLeftMotor;

    public MecanumDrive(DcMotorEx frm, DcMotorEx flm, DcMotorEx brm, DcMotorEx blm) {

        frontRightMotor=frm;
        frontLeftMotor=flm;
        backRightMotor=brm;
        backLeftMotor=blm;
        frontRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        frontRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
    }
    //Takes in x y and rotation each with range [-1,1] and sets the 4 corners of the chasis to the correct power
    public void Drive (double X, double Y, double W) {
        double denominator=Math.max(Math.abs(Y) + Math.abs(X) + Math.abs(W), 1);
        frontRightMotor.setPower((Y+X+W)/denominator);
        frontLeftMotor.setPower((Y-X-W)/denominator);
        backRightMotor.setPower((Y-X+W)/denominator);
        backLeftMotor.setPower((Y+X-W)/denominator);
    }
}
