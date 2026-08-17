package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "CombinedTeleOp")
public class CombinedTeleOp extends LinearOpMode {

    // ---- Drive ----
    private IMU imu;

    // ---- Lift ----
    private DcMotor liftMotor;
    // Adjust these after testing
    private static final double LIFT = 0.5;
    private static final double LOWER = -0.34;

    // ---- Bucket servo ----
    // TODO: uncomment once the servo is wired up, and set the real hardware name below
    // private Servo bucketServo;
    private static final double NORMAL_POS = 0.50;
    private static final double DUMP_POS = 0.25;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Hardware map ---
        // Make sure these names match your robot configuration on the Driver Hub
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontleft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backleft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontright");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backright");
        DcMotor intake = hardwareMap.dcMotor.get("intake");

        liftMotor = hardwareMap.get(DcMotor.class, "lyft");
        // bucketServo = hardwareMap.get(Servo.class, "NAME OF TILT SERVO");

        // Reverse the right side motors. If the robot drives backwards when
        // commanded forward, reverse the left side instead.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // IMU setup for field-centric drive
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        // bucketServo.setPosition(NORMAL_POS);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // ================= GAMEPAD1: Field-centric mecanum drive =================
            double y = -gamepad1.left_stick_y; // Y stick is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double intakePower = gamepad1.right_trigger;
            double intakePowerReverse = -gamepad1.left_trigger;

            // Reset field-centric heading. Equivalent to "start" on Xbox controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1; // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower  = (rotY + rotX + rx) / denominator;
            double backLeftPower   = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower  = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // ================= GAMEPAD2: Lift (linear actuator) =================
            if (gamepad2.right_trigger > 0.1) {
                liftMotor.setPower(LIFT);
            } else if (gamepad2.left_trigger > 0.1) {
                liftMotor.setPower(LOWER);

            } else {
                liftMotor.setPower(0);
            }
            intake.setPower(intakePower*0.95);
            intake.setPower(intakePowerReverse*0.95);

            // ================= GAMEPAD2: Bucket servo =================
            // Uncomment once bucketServo is wired up above
            /*
            if (gamepad2.b) {
                bucketServo.setPosition(DUMP_POS);
            } else {
                bucketServo.setPosition(NORMAL_POS);
            }
            */

            // ================= Telemetry =================
            telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
            telemetry.addData("Right Trigger", gamepad2.right_trigger);
            telemetry.addData("Lift Power", liftMotor.getPower());
            // telemetry.addData("Bucket Servo", bucketServo.getPosition());
            telemetry.update();
        }
    }
}