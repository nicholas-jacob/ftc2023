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
    private BNO055IMU imu;
    private double angleOffset=0;
    private boolean foc=true;
    //TW
    private PIDController twController;
    public static double Tp=0.03, Ti = 0, Td = 0.001;
    public static double Tf = 0.04, Ts=0, towerTolerance=8;//0.27;

    public static int twTarget = 0;
    private final double ticksPerMM = 1.503876;
    private DcMotorEx towerRight;
    private DcMotorEx towerLeft;

    public static double towerMaxPower = 1.5;

    //ARM
    private PIDController armController;

    public static double Ap = 0.005, Ai = 0, Ad = 0.0006;
    public static double Af = 0.15, As = 0, armTolerance= 50;

    public static int armTarget = 0;
    private final double ticksPerRadian = 28 * (2.89655) * (3.61905) * (5.23077) * (2.4) / (2 * Math.PI);
    private DcMotorEx armMotor;

    public static double armMaxPower = 1.5;

    private double targetX=0;
    private double targetY=0;

    //GR
    private Servo gripperRotationServo;
    private Servo alignmentBarServo;
    private CRServo frontRollerServo;
    private CRServo backRollerServo;
    private int retractAlignmentBar = 0;
    private final double alignmentBarDownPos = 0;
    private final double alignmentBarUpPos = 0.75;
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
        int towerPos = towerLeft.getCurrentPosition();
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
        int towerPos = towerLeft.getCurrentPosition();
        int armPos = armMotor.getCurrentPosition();
        double targetXLast=targetX;
        double targetYLast=targetY;


        //intakePosition
        if (gamepad2.right_bumper){
            targetX=480;
            targetY=-202;
            alignmentBarServo.setPosition(alignmentBarUpPos);
            gripperRotationServoPosition=0.2;
        }
        //intake stack
        if (gamepad2.right_trigger>=0.5){
            targetX = 486;
            targetY = -48;
            alignmentBarServo.setPosition(alignmentBarUpPos);
            gripperRotationServoPosition=0.2;
        }
        //groundJunction
        if (gamepad2.dpad_down){
            targetX=-359;
            targetY=-17;
            alignmentBarServo.setPosition(alignmentBarUpPos);
            gripperRotationServoPosition=0.35;
        }
        //lowJunction
        if (gamepad2.dpad_left){
            targetX=409.7;
            targetY=112;
            alignmentBarServo.setPosition(alignmentBarUpPos);
            gripperRotationServoPosition=0.35;
        }
        //midJunction
        if (gamepad2.dpad_right){
            targetX=-299;
            targetY=518;
            alignmentBarServo.setPosition(0.4);
            gripperRotationServoPosition=0.35;

        }
        //highJunction
        if (gamepad2.dpad_up){
            targetX=18;
            targetY=671;
            alignmentBarServo.setPosition(0.4);
            gripperRotationServoPosition=0.35;
        }
        //highCycle
        if (gamepad2.left_bumper){
            targetX=-137;
            targetY=716;
            alignmentBarServo.setPosition(0.4);
            gripperRotationServoPosition=0.35;
        }
        //highStack
        if (gamepad2.left_trigger>0.5){
            targetX = -356;
            targetY = 798;
            alignmentBarServo.setPosition(0.4);
            gripperRotationServoPosition=0.35;
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
                retractAlignmentBar=3;
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