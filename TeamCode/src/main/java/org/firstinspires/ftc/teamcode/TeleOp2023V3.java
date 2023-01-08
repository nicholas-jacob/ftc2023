package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TeleOp2023V3 extends OpMode{

    //DT
    private DcMotorEx frontRightMotor=null;
    private DcMotorEx frontLeftMotor=null;
    private DcMotorEx backRightMotor=null;
    private DcMotorEx backLeftMotor=null;

    //TW
    private DcMotorEx towerRight=null;
    private DcMotorEx towerLeft=null;

    //ARM
    private DcMotorEx armMotor = null;

    //GR
    private Servo gripperRotationServo = null;
    private Servo alignmentBarServo = null;
    private CRServo frontRollerServo = null;
    private CRServo backRollerServo = null;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {

        //set up mecanum drive
        frontRightMotor=hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        frontLeftMotor=hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backRightMotor=hardwareMap.get(DcMotorEx.class, "backRightMotor");
        backLeftMotor=hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        MecanumDrive mecanum=new MecanumDrive(frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor);

        frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        mecanum.Drive(1,1,1);


        //setUp tower
        towerRight=hardwareMap.get(DcMotorEx.class, "towerRight");
        towerLeft=hardwareMap.get(DcMotorEx.class, "towerLeft");
        PID tower=new PID(towerRight);
        tower.AddSlave(towerLeft);

        towerRight.setDirection(DcMotorEx.Direction.FORWARD);
        towerLeft.setDirection(DcMotorEx.Direction.REVERSE);
        tower.Iterate(100);

        //setUp arm
        armMotor=hardwareMap.get(DcMotorEx.class, "armMotor");
        PID arm=new PID(armMotor);

        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.Iterate(100);





        gripperRotationServo = hardwareMap.get(Servo.class, "gripperRotationServo");
        alignmentBarServo = hardwareMap.get(Servo.class, "alignmentBarServo");
        frontRollerServo = hardwareMap.get(CRServo.class, "frontRollerServo");
        backRollerServo = hardwareMap.get(CRServo.class, "backRollerServo");





    }

    public void init_loop() {

    }

    public void start() {

    }
    @Override
    public void loop() {

    }
}
