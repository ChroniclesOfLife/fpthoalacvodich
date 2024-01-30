package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.hardware.motors.RevRoboticsUltraPlanetaryHdHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
public class RobotHardware {

    private LinearOpMode OpMode = null;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private DcMotor Arm = null;
    private DcMotor Intake = null;

    private Servo AirplaneServo = null;

    private Servo Cap = null;

    public static final double INTAKE_SPEED = 0.5;

    public RobotHardware (LinearOpMode opMode) {OpMode = opMode; }

    public void init(){
        leftDrive = OpMode.hardwareMap.get(DcMotor.class, "LeftDrive");
        rightDrive = OpMode.hardwareMap.get(DcMotor.class, "RightDrive");
        Arm = OpMode.hardwareMap.get(DcMotor.class, "Arm");
        Intake = OpMode.hardwareMap.get(DcMotor.class, "Intake");

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }
}
