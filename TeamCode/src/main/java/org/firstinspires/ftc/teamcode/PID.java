package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {

    //public DcMotorEx slaveMotor = null;
    double integralSum=0;
    double Kp=0;
    double Ki=0;
    double Kd=0;
    ElapsedTime timer = new ElapsedTime();
    double lastError = 0;
    DcMotorEx allMotors[];
    double error = 0;

    public PID(DcMotorEx... motor) {

        allMotors = motor;

        for (int motorIndex=0; motorIndex<motor.length; motorIndex++){
            motor[motorIndex].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor[motorIndex].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motor[motorIndex].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor[motorIndex].setPower(0);
        }

    }    //Takes in x y and rotation each with range [-1,1] and sets the 4 corners of the DT to the correct power
    public void setValues (double setKp, double setKi, double setKd){
        Kp=setKp;
        Ki=setKi;
        Kd=setKd;
    }
    public void resetIntegralSum () {
        integralSum=0;
    }


    public void Iterate (double target) {
        lastError = error;
        error = target- allMotors[0].getCurrentPosition();
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        for (int motorIndex=0; motorIndex<allMotors.length; motorIndex++) {
            allMotors[motorIndex].setPower(output);
        }
        timer.reset();
    }
}
