package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Config
@TeleOp
public class TeleOp2023V3 extends OpMode {

    //DT
    //used to have =null but dont think that is nesecary now
    private MecanumDrive mecanum;
    private DcMotorEx frontRightMotor;
    private DcMotorEx frontLeftMotor;
    private DcMotorEx backRightMotor;
    private DcMotorEx backLeftMotor;

    //TW
    private PIDController twController;
    public static double Tp = 0, Ti = 0, Td = 0;
    public static double Tf = 0;

    public static int twTarget = 0;
    private final double ticksPerMM = 1.503876;
    private DcMotorEx towerRight;
    private DcMotorEx towerLeft;

    //ARM
    private PIDController armController;

    public static double Ap = 0, Ai = 0, Ad = 0;
    public static double Af = 0;

    public static int armTarget = 0;
    private final double ticksPerRadian = 28 * (2.89655) * (3.61905) * (5.23077) * (2.4) / (2 * Math.PI);
    private DcMotorEx armMotor;


    //GR
    private Servo gripperRotationServo = null;
    private Servo alignmentBarServo = null;
    private CRServo frontRollerServo = null;
    private CRServo backRollerServo = null;

    private ElapsedTime runtime = new ElapsedTime();
    //private InverseKinematics inverseKinematics = new InverseKinematics();


    @Override
    public void init() {

        //set up mecanum drive
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        MecanumDrive mecanum = new MecanumDrive(frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor);

        frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        mecanum.Drive(1, 1, 1);


        //setUp tower
        //twController = new PIDController(Tp, Ti, Td);
        towerRight = hardwareMap.get(DcMotorEx.class, "towerRight");
        towerLeft = hardwareMap.get(DcMotorEx.class, "towerLeft");

        towerRight.setDirection(DcMotorEx.Direction.FORWARD);
        towerLeft.setDirection(DcMotorEx.Direction.REVERSE);
        towerRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        towerLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        towerRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        towerLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        //setUp arm
        //armController = new PIDController(Ap, Ai, Ad);
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        armMotor.setDirection(DcMotorEx.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        gripperRotationServo = hardwareMap.get(Servo.class, "gripperRotationServo");
        alignmentBarServo = hardwareMap.get(Servo.class, "alignmentBarServo");
        frontRollerServo = hardwareMap.get(CRServo.class, "frontRollerServo");
        backRollerServo = hardwareMap.get(CRServo.class, "backRollerServo");


        //ftc dashboard stuff
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void init_loop() {

    }

    public void start() {

    }

    @Override
    public void loop() {

        //dt code
        mecanum.Drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        //tower controller
        //twController.setPID(Tp, Ti, Td);
        int towerPos = towerRight.getCurrentPosition();
        //double towerPid = twController.calculate(towerPos, twTarget);
        double towerFf = Tf;

        double towerPower = 0;//towerPid + towerFf;
        towerRight.setPower(towerPower);
        towerLeft.setPower(towerPower);

        //arm controller
        //armController.setPID(Ap, Ai, Ad);
        int armPos = armMotor.getCurrentPosition();
        //double armPid = armController.calculate(armPos, armTarget);
        //double armFf = Math.cos(armTarget / ticksPerRadian) * Af;

        double armPower = 0;//armPid + armFf;
        armMotor.setPower(armPower);

        //inverseKinematics.inverse(new Position(0, 0), new Position(0, 0), new ArmTowerPosition(0.0, 0.0), new ArmTowerPosition(0.0, 0.0));

        //uncomment for arm tuning
        telemetry.addData("armPos", armPos);
        telemetry.addData("armTarget", armTarget);


        telemetry.update();
    }
}