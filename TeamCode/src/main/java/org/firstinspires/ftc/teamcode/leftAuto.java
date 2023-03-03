/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Timer;


@Autonomous
public class leftAuto extends OpMode {




    boolean done=false;
    String phase="toCycle";
    boolean depositing=false;
    boolean collecting=false;
    int state=0;
    double collectX=0;
    double collectY=0;




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
    private final double alignmentBarUpPos = 0.65;
    private InverseKinematics inverseKinematics;
    public static double gripperRotationServoPosition=1;




    //camera stuff
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;


    static final double FEET_PER_METER = 3.28084;


    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;


    // UNITS ARE METERS
    double tagsize = 0.166;


    // Tag IDs of sleeve
    int left = 1;
    int middle = 2;
    int right = 3;
    int signal = 3;


    AprilTagDetection tagOfInterest = null;


    //roadrunner setup
    SampleMecanumDrive drive = null;


    //points of interest


    Pose2d startPose=null;
    Vector2d cycle_positionVector=null;
    Trajectory middlePark=null;
    Trajectory leftPark=null;
    Trajectory rightPark=null;
    Trajectory middleParkFinal=null;
    Trajectory leftParkFinal=null;
    Trajectory rightParkFinal=null;
    Trajectory cycle_position=null;
    Trajectory transition=null;






    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timeSinceStart = new ElapsedTime();




    public void init() {


        //bulk read stuff
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);


        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


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


        towerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        towerRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        towerRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        towerLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        towerRight.setPower(0);
        towerLeft.setPower(0);
        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setPower(0);


        //ftc dashboard stuff
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());






        //camera setup
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);


        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }


            @Override
            public void onError(int errorCode) {


            }
        });


        telemetry.setMsTransmissionInterval(50);




        //roadrunner stuff


        //roadrunner setup
        drive = new SampleMecanumDrive(hardwareMap);




        startPose = new Pose2d(-36, -61.3125, Math.toRadians(90));
        cycle_positionVector = new Vector2d(-58, -6.5);



        cycle_position = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-36,-24))
                .splineTo(new Vector2d(-49.5, -14), Math.toRadians(192.0362))//og angle is 194.0362
                .splineToConstantHeading(cycle_positionVector, Math.toRadians(90+14.0362))
                .addDisplacementMarker(() -> {
                    state+=1;
                    drive.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    drive.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    drive.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    drive.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                })
                .build();
        leftPark = drive.trajectoryBuilder(cycle_position.end())
                .addDisplacementMarker(() -> {
                    drive.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    drive.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    drive.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    drive.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                })
                .lineToSplineHeading(new Pose2d(-59, -11.75, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .addDisplacementMarker(() -> {
                    state+=1;
                    targetX=288.43871245;
                    targetY=-350.67571868;
                })
                .build();
        middlePark = drive.trajectoryBuilder(leftPark.end())
                .lineToConstantHeading(new Vector2d(-35.5, -11.75),
                        SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .addDisplacementMarker(() -> {
                    state+=1;
                })
                .build();
        rightPark = drive.trajectoryBuilder(leftPark.end())
                .lineToConstantHeading(new Vector2d(-13, -11.75),
                        SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .addDisplacementMarker(() -> {
                    state+=1;
                })
                .build();


    }




    public void init_loop() {
        //inverse kinematics
        int towerPos = towerRight.getCurrentPosition();
        int armPos = armMotor.getCurrentPosition();
        twController.setPID(Tp, Ti, Td);
        armController.setPID(Ap, Ai, Ad);
        //set targets
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


        telemetry.update();






        //iterate the april tags
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();


        if (currentDetections.size() != 0) {
            boolean tagFound = false;


            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == left || tag.id == middle || tag.id == right) {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }


        }
        if (tagOfInterest != null) {
            if (tagOfInterest.id == left) {
                telemetry.addLine("will go left");
                signal = left;
            } else if (tagOfInterest.id == right) {
                telemetry.addLine("will go right");
                signal = right;
                //tagToTelemetry(tagOfInterest);
            } else if (tagOfInterest.id == middle) {
                telemetry.addLine("will go middle");
                signal = middle;
            }
        } else {
            telemetry.addLine("will go right Bc didn't find anything");
            signal = right;
        }
        telemetry.update();
    }


    /*
     * The START command just came in: now work off the latest snapshot acquired
     * during the init loop.
     */


    public void start() {
        /* Update the telemetry */
        timeSinceStart.reset();
        gripperRotationServoPosition=0.35;
        alignmentBarServo.setPosition(alignmentBarUpPos);
        frontRollerServo.setPower(0.1);
        backRollerServo.setPower(0.1);

        //set final parking zone from the camera
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            //tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }
        drive.setPoseEstimate(startPose);






        //finite state machine reset
    }




    public void loop() {
        if (Objects.equals(phase, "toCycle")) {
            if (state == 0) {
                targetX=231;
                targetY=494;
                state += 1;
            } else if (state == 1) {
                if (withinTolerance(armController.getPositionError(), 100, twController.getPositionError(), 20)) {
                    state += 1;
                }
            } else if (state == 2) {
                drive.followTrajectoryAsync(cycle_position);
                state += 1;
            } else if (state == 4) {
                phase = "deposit1";
                state = 0;
            }
        } else if (Objects.equals(phase, "deposit1")) { //deposit cone #1
            if (state != 0 && depositing == false) {
                phase = "collect2";
                state = 0;
            } else {
                depositing = true;
            }
        } else if (Objects.equals(phase, "collect2")){ //collect cone #2
            if (state != 0 && collecting == false) {
                phase = "deposit2";
                state = 0;
            } else {
                collecting = true;
                gripperRotationServoPosition=0.1;
                collectX=401;
                collectY=-170;
            }
        } else if (Objects.equals(phase, "deposit2")) { //deposit cone #2
            if (state != 0 && depositing == false) {
                phase = "collect3";
                state = 0;
            } else {
                depositing = true;
            }
        } else if (Objects.equals(phase, "collect3")){ //collect cone #3
            if (state != 0 && collecting == false) {
                phase = "deposit3";
                state = 0;
            } else {
                collecting = true;
                gripperRotationServoPosition=0.1;
                collectX=393;
                collectY=-200;
            }
        } else if (Objects.equals(phase, "deposit3")) { //deposit cone #3
            if (state != 0 && depositing == false) {
                phase = "collect4";
                state = 0;
            } else {
                depositing = true;
            }
        } else if (Objects.equals(phase, "collect4")){ //collect cone #4
            if (state != 0 && collecting == false) {
                phase = "deposit4";
                state = 0;
            } else {
                collecting = true;
                gripperRotationServoPosition=0.1;
                collectX=386;
                collectY=-230;
            }
        }else if (Objects.equals(phase, "deposit4")) { //deposit cone #4
            if (state != 0 && depositing == false) {
                phase = "collect5";
                state = 0;
            } else {
                depositing = true;
            }
        } else if (Objects.equals(phase, "collect5")){ //collect cone #5
            if (state != 0 && collecting == false) {
                phase = "deposit5";
                state = 0;
            } else {
                collecting = true;
                gripperRotationServoPosition=0.1;
                collectX=378;
                collectY=-260;
            }
        }else if (Objects.equals(phase, "deposit5")) { //deposit cone #5
            if (state != 0 && depositing == false) {
                phase = "collect6";
                state = 0;
            } else {
                depositing = true;
            }
        } else if (Objects.equals(phase, "collect6")){ //collect cone #6
            if (state != 0 && collecting == false) {
                phase = "deposit6";
                state = 0;
            } else {
                collecting = true;
                gripperRotationServoPosition=0.1;
                collectX=371;
                collectY=-290;
            }
        }else if (Objects.equals(phase, "deposit6")) { //deposit cone #6
            if (state != 0 && depositing == false) {
                phase = "park";
                state = 0;
            } else {
                depositing = true;
            }
        }else if (Objects.equals(phase, "park")) {
            if (state == 0) {
                targetX=288.43871245;
                targetY=-350.67571868+300;
                gripperRotationServoPosition=1;
                alignmentBarServo.setPosition(0.35);
                state += 1;
            } else if (state == 1) {
                drive.followTrajectoryAsync(leftPark);
                state+=1;
            } else if (state == 3) {
                if (signal==left){
                    state+=2;
                } else if (signal==middle){
                    drive.followTrajectoryAsync(middlePark);
                    state+=1;
                } else if (signal==right){
                    drive.followTrajectoryAsync(rightPark);
                    state+=1;
                }
            } else if (state == 5) {
                phase = "finish";
                state = 0;
            }


        } else if (Objects.equals(phase, "finish")) {
            if (state == 0) {
                targetX=288.43871245;
                targetY=-340.67571868;
                gripperRotationServoPosition=1;
                alignmentBarServo.setPosition(0.35);
                state += 1;
            } else if (state == 1) {
                if (withinTolerance(armController.getPositionError(), 100, twController.getPositionError(), 20)) {
                    state += 1;
                }
            } else if (state == 2) {
                done = true;
            }
        }


        if (depositing == true) { //deposit macro if depsiting is set to true it will caryout nesecary deposting steps then when finish sets depositng to false
            if (state == 0) { //to above junction
                if (timeSinceStart.milliseconds()>=40000){
                    state=50;
                    depositing = false;
                } else {
                    targetX = -360;
                    targetY = 790;
                    state += 1;
                    alignmentBarServo.setPosition(0.35);
                    timer.reset();
                }
            } else if (state == 1) {
                if (timer.milliseconds()>300){
                    gripperRotationServoPosition=0.35;
                }
                if (withinTolerance(armController.getPositionError(), 30, twController.getPositionError(), 10) || timer.milliseconds()>1200)  {
                    state += 1;
                    alignmentBarServo.setPosition(alignmentBarDownPos);
                    gripperRotationServoPosition=0.35;
                    timer.reset();
                }
            } else if (state == 2) {
                if (timer.milliseconds() >= 500) {//deposit
                    frontRollerServo.setPower(-1);
                    backRollerServo.setPower(-1);
                    retractAlignmentBar=6;
                    state += 1;
                }
            } else if (state ==3) {
                if (timer.milliseconds() >= 700) {
                    frontRollerServo.setPower(0);
                    backRollerServo.setPower(0);
                    depositing = false;
                }
            }

        }
        if (collecting == true) { //collect macro if collecting is set to true it will caryout nesecary collecting steps then when finish sets colecting to false
            if (state == 0) { //to above junction
                if (timeSinceStart.milliseconds()>=40000){
                    state=50;
                    collecting = false;
                }
                else {
                    targetX = 431-15;
                    targetY = -48;
                    state += 1;
                    alignmentBarServo.setPosition(alignmentBarUpPos);
                    timer.reset();
                }

            } else if (state == 1) {
                if (withinTolerance(armController.getPositionError(), 30, twController.getPositionError(), 10) || timer.milliseconds()>=1000) {
                    state += 1;
                    targetX = collectX;
                    targetY = collectY;
                    frontRollerServo.setPower(1);
                    backRollerServo.setPower(1);
                    timer.reset();
                }
            } else if (state == 2) {
                if (timer.milliseconds() >= 600) {
                    targetX = 431-15;
                    targetY = -48;
                    frontRollerServo.setPower(0.1);
                    backRollerServo.setPower(0.1);
                    state += 1;
                }
            } else if(state == 3){
                if (timer.milliseconds() >= 700) {
                    collecting = false;
                }
            }

        }


        telemetry.addData("currentFSMState", state);
        telemetry.addData("phase", phase);
        telemetry.addData("depositing?", depositing);
        telemetry.addData("collecting", collecting);
        telemetry.addData("targetX", targetX);
        telemetry.addData("targetY", targetY);
        telemetry.addData("armTarget", armTarget);
        telemetry.addData("armError", armController.getPositionError());
        telemetry.addData("towerTarget", twTarget);
        telemetry.addData("towerError", twController.getPositionError());
        drive.update();


        //inverse kinemetics
        int towerPos = towerRight.getCurrentPosition();
        int armPos = armMotor.getCurrentPosition();
        double targetXLast = targetX;
        double targetYLast = targetY;


        //calculate with inverse kinematics
        if (inverseKinematics.calculate(targetX, targetY, armPos, towerPos)) {
            armTarget = inverseKinematics.armTarget;
            twTarget = inverseKinematics.towerTarget;
        } else {
            targetX = targetXLast;
            targetY = targetYLast;
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
        if (!done){
            towerRight.setPower(towerPower);
            towerLeft.setPower(towerPower);
            armMotor.setPower(armPower);
        }
        else{
            armMotor.setPower(0);
            towerRight.setPower(0);
            towerLeft.setPower(0);
        }


        if (retractAlignmentBar > 0) {
            if (retractAlignmentBar == 1) {
                alignmentBarServo.setPosition(alignmentBarUpPos);
            }
            retractAlignmentBar -= 1;
        }
        gripperRotationServo.setPosition(gripperRotationServoPosition);


        telemetry.update();
    }
    public Boolean withinTolerance(double armError, int armTolerance, double towerError, int towerTolerance){ //arm position checking function to allow only moving on if it is in pos
        if (Math.abs(armError)<=armTolerance && Math.abs(towerError)<=towerTolerance){
            return true;
        }
        else{
            telemetry.addLine("im not within tolerance yet");
            return false;
        }




    }
}





