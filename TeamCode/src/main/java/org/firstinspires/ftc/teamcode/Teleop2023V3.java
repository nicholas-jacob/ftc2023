package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Teleop2023V3 extends OpMode{


    //TW
    private DcMotorEx towerRight = null;
    private DcMotorEx towerLeft = null;

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

        //set up mechanum drive

        MechanumDrive mechanum=new MechanumDrive(hardwareMap.get(DcMotorEx.class, "frontRightMotor"), hardwareMap.get(DcMotorEx.class, "frontLeftMotor"), hardwareMap.get(DcMotorEx.class, "backRightMotor"), hardwareMap.get(DcMotorEx.class, "backLeftMotor"));
        mechanum.frontRightMotor.setPower(1);

        mechanum.frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        mechanum.backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        mechanum.frontLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        mechanum.backLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        mechanum.Drive(1,1,1);






        towerRight = hardwareMap.get(DcMotorEx.class, "towerRight");
        towerLeft = hardwareMap.get(DcMotorEx.class, "towerLeft");

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        gripperRotationServo = hardwareMap.get(Servo.class, "gripperRotationServo");
        alignmentBarServo = hardwareMap.get(Servo.class, "alignmentBarServo");
        frontRollerServo = hardwareMap.get(CRServo.class, "frontRollerServo");
        backRollerServo = hardwareMap.get(CRServo.class, "backRollerServo");

        //setDirections


        towerRight.setDirection(DcMotorEx.Direction.FORWARD);
        towerLeft.setDirection(DcMotorEx.Direction.REVERSE);

        armMotor.setDirection(DcMotorEx.Direction.FORWARD);

        //setMotorMode

        //towerRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //towerLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        towerRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        towerLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //setZeroPowerBehavior

        towerRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        towerLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //setMotorToZeroPower
        towerRight.setPower(0);
        towerLeft.setPower(0);
        armMotor.setPower(0);


    }

    public void init_loop() {

    }

    public void start() {

    }
    @Override
    public void loop() {

    }
}
