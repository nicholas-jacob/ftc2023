package org.firstinspires.ftc.teamcode;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.outoftheboxrobotics.photoncore.PhotonLynxModule;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;



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
    private BHI260IMU imu;
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
    private static int armOffset=3050;
    private DcMotorEx armMotor;

    public static double armMaxPower = 1.5;

    private double targetX=0;
    private double targetY=0;

    //GR
    private Servo leftOdoLift;
    private Servo rightOdoLift;
    private Servo gripperRotationServo;
    private ServoImplEx alignmentBarServo;
    private Servo leftClaw;
    private Servo rightClaw;
    private Servo wheelieBarServo;
    public static double leftOdoLiftPos=0.06; //0.52 up pos
    public static double rightOdoLiftPos=0.98; //0.48 down pos
    public static double wheelieBarPosition=.56;
    private int retractAlignmentBar = 0;
    public static int retractAlignmentBarDelay=3;
    public static double alignmentBarDownPos = 0.768;
    public static double alignmentBarMidPos = 0.4;
    public static double alignmentBarUpPos = 0.25;
    private InverseKinematics inverseKinematics;
    public static double gripperRotationServoPosition=.5;
    //in distance away from 0.5;
    public static double gripperOpenPos=0.3; //important //192.168.43.1:8080/dash
    public static double gripperHalfOpenPos=0.47;
    public static double gripperClosePos=0.71;
    public static double gripperCloseTight=0.71;
    public String gripperState="open";
    private static boolean gp2ANotPressed;




    @Override
    public void init() {


//        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
//
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }
        //set up mecanum drive
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "motorBackRight");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        mecanum = new MecanumDrive(frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor);
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);

        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        angleOffset=robotOrientation.getYaw(AngleUnit.RADIANS);


        frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);

        //setUpServos
        leftOdoLift = hardwareMap.get(Servo.class, "leftOdoLift");
        rightOdoLift = hardwareMap.get(Servo.class, "rightOdoLift");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        alignmentBarServo = hardwareMap.get(ServoImplEx.class, "alignmentBarServo");
        alignmentBarServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
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
            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        towerRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        towerLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        towerRight.setPower(0);
        towerLeft.setPower(0);
        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setPower(0);


        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.enable();
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
        leftOdoLift.setPosition(leftOdoLiftPos);
        rightOdoLift.setPosition(rightOdoLiftPos);
    }

    public void start() {
        gripperRotationServoPosition=0.5;
        alignmentBarServo.setPosition(alignmentBarUpPos);
        targetX=231;
        targetY=494;
    }


    @Override
    public void loop() {
        PhotonCore.CONTROL_HUB.clearBulkCache();
        //List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        //for (LynxModule hub : allHubs) {
        //    hub.clearBulkCache();
        //}
        //dt code
        //code to let driver reset starting angle
        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        if (gamepad1.dpad_up){
            angleOffset=robotOrientation.getYaw(AngleUnit.RADIANS);
        }
        if (gamepad1.dpad_right){
            angleOffset=robotOrientation.getYaw(AngleUnit.RADIANS)+Math.toRadians(90);
        }
        if (gamepad1.dpad_down){
            angleOffset=robotOrientation.getYaw(AngleUnit.RADIANS)+Math.toRadians(180);
        }
        if (gamepad1.dpad_left){
            angleOffset=robotOrientation.getYaw(AngleUnit.RADIANS)+Math.toRadians(270);
        }

        if (gamepad1.a){
            foc = true;
        }
        if (gamepad1.b){
            foc = false;
        }
        if (foc){ //disables foc if foc variable is false
            double angle=robotOrientation.getYaw(AngleUnit.RADIANS)-angleOffset;
            double X= gamepad1.left_stick_x;
            double Y= gamepad1.left_stick_y;
            double Xprime=X*Math.cos(angle)-Y*Math.sin(angle);
            double Yprime=X*Math.sin(angle)+Y*Math.cos(angle);
            mecanum.Drive(Xprime, Yprime, gamepad1.right_stick_x);
        } else{
            mecanum.Drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
        // finite state machine goes here
        int towerPos = -backLeftMotor.getCurrentPosition();
        int armPos = -backRightMotor.getCurrentPosition()+armOffset;
        double targetXLast=targetX;
        double targetYLast=targetY;

        //wheelie bar deploy
        if (gamepad1.right_trigger>0.5){
            wheelieBarPosition=0.03;
        } else {
            wheelieBarPosition=0.56;
        }


        //intakePosition
        if (gamepad2.right_bumper){
            targetX=330;
            targetY=-317; //-296
            alignmentBarServo.setPosition(alignmentBarUpPos);
            gripperRotationServoPosition=0.5;
        }
        //intake thrust
        if (gamepad2.left_trigger>0.5){
            targetX=421;
            targetY=-281;
            alignmentBarServo.setPosition(alignmentBarUpPos);
            gripperRotationServoPosition=0.5;
            gripperState="open";
        }
        //low
        if (gamepad2.dpad_left){
            //low front
            if (gamepad2.right_trigger>0.5){
                targetX = 397;
                targetY = -23;
                alignmentBarServo.setPosition(alignmentBarUpPos);
                gripperRotationServoPosition=0.25;
            } else {
                //low back
                targetX = -449;
                targetY = 246;
                alignmentBarServo.setPosition(alignmentBarUpPos);
                gripperRotationServoPosition=0.5;
            }
        }
        //mid
        if (gamepad2.dpad_right){
            //mid front
            if (gamepad2.right_trigger>0.5){
                targetX=482;
                targetY=300;
                alignmentBarServo.setPosition(alignmentBarUpPos);
                gripperRotationServoPosition=0.5;
            } else{
                targetX=-318;
                targetY=500;
                alignmentBarServo.setPosition(alignmentBarUpPos);
                gripperRotationServoPosition=0.5;
            }
        }
        //high
        if (gamepad2.dpad_up){
            if (gamepad2.right_trigger>0.5){
                targetX=481;
                targetY=557;
                alignmentBarServo.setPosition(alignmentBarMidPos);
                gripperRotationServoPosition=0.5;
            } else{
                targetX=-119;
                targetY=678;
                alignmentBarServo.setPosition(alignmentBarMidPos);
                gripperRotationServoPosition=0.5;
            }
        }

        //travelPos
        if (gamepad2.left_bumper){
            targetX=231;
            targetY=515;
            alignmentBarServo.setPosition(alignmentBarUpPos);
            gripperRotationServoPosition=0.5;
        }

        targetX+=(gamepad2.left_stick_x)*6;
        targetY-=(gamepad2.left_stick_y)*6;

        if (gamepad2.b) {
                gripperState="open";
                retractAlignmentBar = retractAlignmentBarDelay;
        }

        if (gripperState=="open"){
            if (armPos>1000){
                leftClaw.setPosition(gripperOpenPos);
                rightClaw.setPosition(gripperOpenPos);
                telemetry.addData("gripperState", 1);
            } else{
                leftClaw.setPosition(gripperHalfOpenPos);
                rightClaw.setPosition(gripperHalfOpenPos);
                telemetry.addData("gripperState", 2);
            }
            if (gamepad2.a){
                gripperState="close";
                alignmentBarServo.setPosition(alignmentBarUpPos);
                gp2ANotPressed=false;

            }
        } else if(gripperState=="close") {
            leftClaw.setPosition(gripperClosePos);
            rightClaw.setPosition(gripperClosePos);
            telemetry.addData("gripperState", 3);
            if (gamepad2.a == false){
                gp2ANotPressed=true;
            }
            if (gamepad2.a && gp2ANotPressed){
                gripperState="closeTight";
            }
        } else {
            leftClaw.setPosition(gripperCloseTight);
            rightClaw.setPosition(gripperCloseTight);
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

        leftOdoLift.setPosition(leftOdoLiftPos);
        rightOdoLift.setPosition(rightOdoLiftPos);



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
//        telemetry.addData("robotYaw", robotOrientation.getYaw(AngleUnit.RADIANS));
        loopTime.reset();


        //uncomment for arm tuning
        telemetry.addData("armPos", armPos);
        telemetry.addData("armError", armTarget-armPos);
        telemetry.addData("armPower", armPower);
//
//        //uncomment for tower tuning
        telemetry.addData("twPos", towerPos);
        telemetry.addData("towerError", twTarget-towerPos);
        telemetry.addData("towerPower", towerPower);
//
        telemetry.addData("TargetX", targetX);
        telemetry.addData("TargetY", targetY);

        telemetry.update();



    }



}