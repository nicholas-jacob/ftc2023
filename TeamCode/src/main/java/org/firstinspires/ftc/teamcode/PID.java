package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    public DcMotorEx mainMotor=null;
    public DcMotorEx slaveMotor = null;
    double integralSum;
    double Kp;
    double Ki;
    double Kd;
    boolean slaveExists=false;
    ElapsedTime timer= new ElapsedTime();
    double lastError = 0;

    public PID(DcMotorEx mainMotor) {

        mainMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mainMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        mainMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mainMotor.setPower(0);

        integralSum=0;
        Kp=0;
        Ki=0;
        Kd=0;

    }
    public void AddSlave(DcMotorEx slaveMotor) {
        slaveMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slaveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slaveMotor.setPower(0);
        slaveExists=true;
    }
    //Takes in x y and rotation each with range [-1,1] and sets the 4 corners of the DT to the correct power
    public void Iterate (double target) {
        double error = target- mainMotor.getCurrentPosition();
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        mainMotor.setPower(output);
        if (slaveExists) {
            slaveMotor.setPower(output);
        }

        timer.reset();
    }
}
