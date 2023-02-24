package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


import java.util.List;



@Config
@TeleOp
public class TeleOp2023V3 extends OpMode {
    // if auto was just ran set true else set flase
    private final boolean auto=false;




    //DT
    //used to have =null but dont think that is nesecary now
    private MecanumDrive mecanum;
    private DcMotorEx frontRightMotor;
    private DcMotorEx frontLeftMotor;
    private DcMotorEx backRightMotor;
    private DcMotorEx backLeftMotor;

    //TW
    private PIDController twController;
    public static double Tp=0.03, Ti = 0, Td = 0.0014;
    public static double Tf = 0;//0.27;

    public static int twTarget = 0;
    private final double ticksPerMM = 1.503876;
    private DcMotorEx towerRight;
    private DcMotorEx towerLeft;

    public static double towerMaxPower = 1;

    //ARM
    private PIDController armController;

    public static double Ap = 0.006, Ai = 0, Ad = 0.0006;
    public static double Af = 0.13;

    public static int armTarget = 0;
    private final double ticksPerRadian = 28 * (2.89655) * (3.61905) * (5.23077) * (2.4) / (2 * Math.PI);
    private DcMotorEx armMotor;

    public static double armMaxPower = 1;

    private double targetX=0;
    private double targetY=0;

    //GR
    private Servo gripperRotationServo;
    private Servo alignmentBarServo;
    private CRServo frontRollerServo;
    private CRServo backRollerServo;
    private int retractAlignmentBar = 0;
    private final double alignmentBarDownPos = 0;
    private final double alignmentBarUpPos = 0.65;
    private InverseKinematics inverseKinematics;
    public static double gripperRotationServoPosition=1;




    @Override
    public void init() {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        //set up mecanum drive
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "motorBackRight");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        mecanum = new MecanumDrive(frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor);

        frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);

        //setUpServos
        frontRollerServo = hardwareMap.get(CRServo.class, "frontRollerServo");
        backRollerServo = hardwareMap.get(CRServo.class, "backRollerServo");
        alignmentBarServo = hardwareMap.get(Servo.class, "alignmentBarServo");
        gripperRotationServo = hardwareMap.get(Servo.class, "gripperRotationServo");
        gripperRotationServoPosition=1;
        //setUp tower
        twController = new PIDController(Tp, Ti, Td);
        towerRight = hardwareMap.get(DcMotorEx.class, "towerRight");
        towerLeft = hardwareMap.get(DcMotorEx.class, "towerLeft");

        towerRight.setDirection(DcMotorEx.Direction.REVERSE);
        towerLeft.setDirection(DcMotorEx.Direction.FORWARD);
        towerRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        towerLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);




        //setUp arm
        armController = new PIDController(Ap, Ai, Ad);
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        armMotor.setDirection(DcMotorEx.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        inverseKinematics = new InverseKinematics(ticksPerRadian, ticksPerMM);

        //reset encoders only if auto wasn't just run
        if (!auto){
            towerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            towerRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        towerRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        towerLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        towerRight.setPower(0);
        towerLeft.setPower(0);
        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setPower(0);

        //ftc dashboard stuff
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void init_loop() {
        //inverse kinematics
        int towerPos = towerRight.getCurrentPosition();
        int armPos = armMotor.getCurrentPosition();
        twController.setPID(Tp, Ti, Td);
        armController.setPID(Ap, Ai, Ad);
        //set targets
        if (auto) {
            armMotor.setPower(0);
            towerRight.setPower(0);
            towerLeft.setPower(0);
        }
        else {
            targetX=288.43871245;
            targetY=-340.67571868;

            //calculate with invserse kinematics here
            int armTargetLast=armTarget;
            int twTargetLast=twTarget;
            if (inverseKinematics.calculate(targetX, targetY, armPos, towerPos)){
                armTarget=inverseKinematics.armTarget;
                twTarget=inverseKinematics.towerTarget;
            }

            //tower controller

            double towerPid = twController.calculate(towerPos, twTarget);
            double towerFf = Tf;

            double towerPower = towerPid + towerFf;
            towerRight.setPower(towerPower);
            towerLeft.setPower(towerPower);

            //arm controller
            armController.setPID(Ap, Ai, Ad);

            double armPid = armController.calculate(armPos, armTarget);
            double armFf = -Math.sin((armPos / ticksPerRadian)-0.236) * Af;

            double armPower = armPid + armFf;
            armMotor.setPower(armPower);

            //set servos
            gripperRotationServo.setPosition(gripperRotationServoPosition);
            alignmentBarServo.setPosition(0.35);
            telemetry.addData("armTarget", armTarget);
            telemetry.addData("armPos", armPos);
            telemetry.addData("towerTarget", twTarget);
            telemetry.addData("towerPos", towerPos);
        }
        telemetry.update();

    }

    public void start() {
        gripperRotationServoPosition=0.35;
        alignmentBarServo.setPosition(alignmentBarUpPos);
        targetX=0;
        targetY=545;
    }


    @Override
    public void loop() {

        //dt code
        mecanum.Drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        // finite state machine goes here
        int towerPos = towerRight.getCurrentPosition();
        int armPos = armMotor.getCurrentPosition();
        double targetXLast=targetX;
        double targetYLast=targetY;


        //intakePosition
        if (gamepad2.right_bumper){
            targetX=493.6;
            targetY=-169.86;
            alignmentBarServo.setPosition(alignmentBarUpPos);
        }
        //groundJunction
        if (gamepad2.dpad_down){
            targetX=-133.9373;
            targetY=-157.6818;
            alignmentBarServo.setPosition(alignmentBarUpPos);
        }
        //lowJunction
        if (gamepad2.dpad_left){
            targetX=-483.1;
            targetY=773.17;
            alignmentBarServo.setPosition(alignmentBarUpPos);
        }
        //midJunction
        if (gamepad2.dpad_right){
            targetX=447.9;
            targetY=-248.1;
            alignmentBarServo.setPosition(alignmentBarUpPos);

        }
        //highJunction
        if (gamepad2.dpad_up){
            targetX=-15;
            targetY=722.4;
            alignmentBarServo.setPosition(alignmentBarUpPos);
        }
        //highCycle
        if (gamepad2.left_bumper){
            targetX=-95.96;
            targetY=714.6;
        }

        targetX+=(gamepad2.left_stick_x)*7;
        targetY-=(gamepad2.left_stick_y)*7;

        if (gamepad2.a){
            frontRollerServo.setPower(1);
            backRollerServo.setPower(1);
            alignmentBarServo.setPosition(alignmentBarUpPos);

        }
        else{
            if (gamepad2.b){
                frontRollerServo.setPower(-1);
                backRollerServo.setPower(-1);
                retractAlignmentBar=5;
            }
            else{
                frontRollerServo.setPower(0.1);
                backRollerServo.setPower(0.1);
            }
        }
        if (gamepad2.x){
            alignmentBarServo.setPosition(alignmentBarDownPos);
        }
        if (gamepad2.y){
            alignmentBarServo.setPosition(alignmentBarUpPos);
        }
        if (retractAlignmentBar > 0) {
            if (retractAlignmentBar == 1) {
                alignmentBarServo.setPosition(alignmentBarUpPos);
            }
            retractAlignmentBar -= 1;
        }
        gripperRotationServo.setPosition(gripperRotationServoPosition);







        //calculate with inverse kinematics
        if (inverseKinematics.calculate(targetX, targetY, armPos, towerPos)){
            armTarget=inverseKinematics.armTarget;
            twTarget=inverseKinematics.towerTarget;
        }
        else {
            targetX=targetXLast;
            targetY=targetYLast;
        }


        //tower controller
        twController.setPID(Tp, Ti, Td);
        double towerPid = twController.calculate(towerPos, twTarget);
        double towerFf = Tf;
        if (towerPid>towerMaxPower){
            towerPid=towerMaxPower;
        }
        if (towerPid<-towerMaxPower){
            towerPid=-towerMaxPower;
        }
        double towerPower = towerPid + towerFf;
        towerRight.setPower(towerPower);
        towerLeft.setPower(towerPower);

        //arm controller
        armController.setPID(Ap, Ai, Ad);

        double armPid = armController.calculate(armPos, armTarget);
        double armFf = -Math.sin((armPos / ticksPerRadian)-0.236) * Af;
        if (armPid>armMaxPower){
            armPid=armMaxPower;
        }
        if (armPid<-armMaxPower){
            armPid=-armMaxPower;
        }

        double armPower = armPid + armFf;
        armMotor.setPower(armPower);




        //uncomment for arm tuning
        telemetry.addData("armPos", armPos);
        telemetry.addData("armTarget", armTarget);
        telemetry.addData("armPower", armPower);

        //uncomment for tower tuning
        telemetry.addData("twPos", towerPos);
        telemetry.addData("towerTarget", twTarget);
        telemetry.addData("towerPower", towerPower);

        telemetry.addData("TargetX", targetX);
        telemetry.addData("TargetY", targetY);

        telemetry.update();

    }



}