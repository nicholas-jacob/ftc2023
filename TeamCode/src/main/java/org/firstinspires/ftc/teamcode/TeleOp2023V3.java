package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
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
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.List;



@Config
@TeleOp
public class TeleOp2023V3 extends OpMode {
    // if auto was just ran set true else set flase
    private final boolean auto=false;




    //DT
    ElapsedTime loopTime = new ElapsedTime();
    private MecanumDrive mecanum;
    private DcMotorEx frontRightMotor;
    private DcMotorEx frontLeftMotor;
    private DcMotorEx backRightMotor;
    private DcMotorEx backLeftMotor;
    private BNO055IMU imu;
    private double angleOffset=0;
    private boolean foc=true;
    //TW
    private PIDController twController;
    public static double Tp=0.014, Ti = 0, Td = 0.00055;
    public static double Tf = 0.175, Ts=0.175, towerTolerance=6;//0.27;

    public static int twTarget = 0;
    private final double ticksPerMM = 2.02696328861;
    private DcMotorEx towerRight;
    private DcMotorEx towerLeft;

    public static double towerMaxPower = 1.5;

    //ARM
    private PIDController armController;

    public static double Ap = 0.002, Ai = 0, Ad = 0.00013;
    public static double Af = 0.11, As = 0, armTolerance= 6;

    public static int armTarget = 0;
    private final double ticksPerRadian = 8192 / (2 * Math.PI);
    private static int armOffset=3025;
    private DcMotorEx armMotor;

    public static double armMaxPower = 1.5;

    private double targetX=0;
    private double targetY=0;

    //GR
    private Servo gripperRotationServo;
    private Servo alignmentBarServo;
    private Servo leftClaw;
    private Servo rightClaw;
    private Servo wheelieBarServo;
    public static double wheelieBarPosition=.56;
    private int retractAlignmentBar = 0;
    public static int retractAlignmentBarDelay=3;
    public static double alignmentBarDownPos = 0.8;
    public static double alignmentBarMidPos = 0.35;
    public static double alignmentBarUpPos = 0.05;
    private InverseKinematics inverseKinematics;
    public static double gripperRotationServoPosition=.5;
    //in distance away from 0.5;
    public static double gripperOpenPos=0.2;
    public static double gripperHalfOpenPos=0.03;
    public static double gripperClosePos=-0.07;
    public static double gripperCloseTight=-.15;
    public String gripperState="open";
    private static boolean gp2ANotPressed;




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
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        angleOffset=imu.getAngularOrientation().firstAngle;


        frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);

        //setUpServos
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        alignmentBarServo = hardwareMap.get(Servo.class, "alignmentBarServo");
        gripperRotationServo = hardwareMap.get(Servo.class, "gripperRotationServo");
        wheelieBarServo = hardwareMap.get(Servo.class, "wheelieBarServo");
        wheelieBarServo.setPosition(wheelieBarPosition);
        gripperRotationServoPosition=0.5;
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
        //set targets
        armMotor.setPower(0);
        towerRight.setPower(0);
        towerLeft.setPower(0);

        //set servos
        gripperRotationServo.setPosition(gripperRotationServoPosition);
        alignmentBarServo.setPosition(alignmentBarUpPos);

    }

    public void start() {
        gripperRotationServoPosition=0.5;
        alignmentBarServo.setPosition(alignmentBarUpPos);
        targetX=231;
        targetY=494;
    }


    @Override
    public void loop() {
        //dt code
        //code to let driver reset starting angle
        if (gamepad1.dpad_up){
            angleOffset=imu.getAngularOrientation().firstAngle;
        }
        if (gamepad1.dpad_right){
            angleOffset=imu.getAngularOrientation().firstAngle+Math.toRadians(90);
        }
        if (gamepad1.dpad_down){
            angleOffset=imu.getAngularOrientation().firstAngle+Math.toRadians(180);
        }
        if (gamepad1.dpad_left){
            angleOffset=imu.getAngularOrientation().firstAngle+Math.toRadians(270);
        }

        if (gamepad1.a){
            foc = true;
        }
        if (gamepad1.b){
            foc = false;
        }
        if (foc){ //disables foc if foc variable is false
            double angle=imu.getAngularOrientation().firstAngle-angleOffset;
            double X= gamepad1.left_stick_x;
            double Y= gamepad1.left_stick_y;
            double Xprime=X*Math.cos(angle)-Y*Math.sin(angle);
            double Yprime=X*Math.sin(angle)+Y*Math.cos(angle);
            mecanum.Drive(Xprime, Yprime, gamepad1.right_stick_x);
        } else{
            mecanum.Drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
        // finite state machine goes here
        int towerPos = (int)Math.round((towerLeft.getCurrentPosition()+towerRight.getCurrentPosition())/2);
        int armPos = armMotor.getCurrentPosition()+armOffset;
        double targetXLast=targetX;
        double targetYLast=targetY;


        //intakePosition
        if (gamepad2.right_bumper){
            targetX=330;
            targetY=-306;
            alignmentBarServo.setPosition(alignmentBarUpPos);
            gripperRotationServoPosition=0.5;
            gripperState="open";
        }
        //intake thrust
        if (gamepad2.left_trigger>0.5){
            targetX=426;
            targetY=-319;
            alignmentBarServo.setPosition(alignmentBarUpPos);
            gripperRotationServoPosition=0.5;
            gripperState="open";
        }
        //low
        if (gamepad2.dpad_left){
            //low front
            if (gamepad2.right_trigger>0.5){
                targetX = 397;
                targetY = -3;
                alignmentBarServo.setPosition(alignmentBarUpPos);
                gripperRotationServoPosition=0.25;
            } else {
                //low back
                targetX = -449;
                targetY = 250;
                alignmentBarServo.setPosition(alignmentBarUpPos);
                gripperRotationServoPosition=0.5;
            }
        }
        //mid
        if (gamepad2.dpad_right){
            //mid front
            if (gamepad2.right_trigger>0.5){
                targetX=532;
                targetY=284;
                alignmentBarServo.setPosition(alignmentBarUpPos);
                gripperRotationServoPosition=0.5;
            } else{
                targetX=-318;
                targetY=504;
                alignmentBarServo.setPosition(alignmentBarUpPos);
                gripperRotationServoPosition=0.5;
            }
        }
        //high
        if (gamepad2.dpad_up){
            if (gamepad2.right_trigger>0.5){
                targetX=526;
                targetY=554;
                alignmentBarServo.setPosition(alignmentBarMidPos);
                gripperRotationServoPosition=0.5;
            } else{
                targetX=-119;
                targetY=687;
                alignmentBarServo.setPosition(alignmentBarMidPos);
                gripperRotationServoPosition=0.5;
            }
        }

        //travelPos
        if (gamepad2.left_bumper){
            targetX=231;
            targetY=515;
            alignmentBarServo.setPosition(alignmentBarMidPos);
            gripperRotationServoPosition=0.5;
        }

        targetX+=(gamepad2.left_stick_x)*15;
        targetY-=(gamepad2.left_stick_y)*15;

        if (gamepad2.b) {
                gripperState="open";
                retractAlignmentBar = retractAlignmentBarDelay;
        }

        if (gripperState=="open"){
            if (armPos>1000){
                leftClaw.setPosition(0.5+gripperOpenPos);
                rightClaw.setPosition(0.5-gripperOpenPos);
                telemetry.addData("gripperState", 1);
            } else{
                leftClaw.setPosition(0.5+gripperHalfOpenPos);
                rightClaw.setPosition(0.5-gripperHalfOpenPos);
                telemetry.addData("gripperState", 2);
            }
            if (gamepad2.a){
                gripperState="close";
                alignmentBarServo.setPosition(alignmentBarUpPos);
                gp2ANotPressed=false;

            }
        } else if(gripperState=="close") {
            leftClaw.setPosition(0.5+gripperClosePos);
            rightClaw.setPosition(0.5-gripperClosePos);
            telemetry.addData("gripperState", 3);
            if (gamepad2.a == false){
                gp2ANotPressed=true;
            }
            if (gamepad2.a && gp2ANotPressed){
                gripperState="closeTight";
            }
        } else {
            leftClaw.setPosition(0.5+gripperCloseTight);
            rightClaw.setPosition(0.5-gripperCloseTight);
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


        //set wheelie bar
        wheelieBarServo.setPosition(wheelieBarPosition);





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
        double towerFs=0;
        if (twController.getPositionError()>towerTolerance){ //static friction code
            towerFs=Ts;
        } else if(twController.getPositionError()<-towerTolerance) {
            towerFs = -Ts;
        }
        double towerPower = towerPid + towerFs;
        if (towerPower>towerMaxPower){
            towerPower=towerMaxPower;
        }
        if (towerPower<-towerMaxPower){
            towerPower=-towerMaxPower;
        }
        towerPower+=towerFf;



        towerRight.setPower(towerPower);
        towerLeft.setPower(towerPower);

        //arm controller
        armController.setPID(Ap, Ai, Ad);
        double armPid = armController.calculate(armPos, armTarget);
        double armFf = -Math.sin((armPos / ticksPerRadian)-0.236) * Af;
        double armFs = 0;
        if (armController.getPositionError()>armTolerance){
            armFs=As;
        } else if(armController.getPositionError()<-armTolerance) {
            armFs=-As;
        }
        double armPower = armPid + armFs;
        if (armPower>armMaxPower){
            armPid=armMaxPower;
        }
        if (armPid<-armMaxPower){
            armPid=-armMaxPower;
        }
        armPower+=armFf;
        armMotor.setPower(armPower);



        telemetry.addData("loopTime", loopTime.milliseconds());
        loopTime.reset();


        //uncomment for arm tuning
        telemetry.addData("armPos", armPos);
        telemetry.addData("armError", armTarget-armPos);
        telemetry.addData("armPower", armPower);

        //uncomment for tower tuning
        telemetry.addData("twPos", towerPos);
        telemetry.addData("towerError", twTarget-towerPos);
        telemetry.addData("towerPower", towerPower);

        telemetry.addData("TargetX", targetX);
        telemetry.addData("TargetY", targetY);

        telemetry.update();

    }



}