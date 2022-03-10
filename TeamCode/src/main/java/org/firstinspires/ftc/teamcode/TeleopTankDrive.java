package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleopTankDrive", group = "Teleop")
public class TeleopTankDrive extends OpMode {

    // Default speed
    final public static double speedMultDefault = 0.5;

    // Speed when left bumper is held down
    final public static double speedMultBoosted = 1.0;

    // The threshold when the pivot action starts
    final public static double pivotThreshold = 0.25;


    private DcMotor tankLeft;
    private DcMotor tankRight;

    private DcMotor motorSpinner;
    private DcMotor motorArm;
    private DcMotor motorIntake;

    private Servo servoBucket;


    @Override
    public void init() {

        telemetry.addData(">", "Program initiated.");

        tankLeft = hardwareMap.dcMotor.get("motor tank left");
        tankRight = hardwareMap.dcMotor.get("motor tank right");

        motorSpinner = hardwareMap.dcMotor.get("motor spinner");
        motorIntake = hardwareMap.dcMotor.get("motor intake");

        motorArm = hardwareMap.dcMotor.get("motor arm");
        motorArm.setTargetPosition(0);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        servoBucket = hardwareMap.servo.get("servo bucket");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    @Override
    public void loop() {

        if (gamepad1.left_trigger >= 0.5) {
            motorIntake.setPower(0.5);
        } else if (gamepad1.left_bumper) {
            motorIntake.setPower(-0.5);
        } else {
            motorIntake.setPower(0.0);
        }

        // Joystick position
        final double x = -gamepad1.left_stick_y;
        final double y = -gamepad1.left_stick_x;

        // Convert joystick linear scale to quadratic
        // scale to grant better control over movement
        double sX = Easings.easeInQuad(x) * (x < 0 ? -1 : 1);
        double sY = Easings.easeInQuad(y) * (y < 0 ? -1 : 1);

        // Convert joystick position into motor powers
        double[] power = differentialDrive(sX, sY);

        // Write the values to the motors
        tankLeft.setPower(power[0]);
        tankRight.setPower(power[1]);

        telemetry.addData("X", x);
        telemetry.addData("Y", y);

        telemetry.addData("sX", sX);
        telemetry.addData("sY", sY);

        telemetry.addData("Tank Left", power[0]);
        telemetry.addData("Tank Right", power[1]);

    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    /*
     * Joystick Differential Steering Algorithm
     * ========================================
     * Concept adapted from Calvin Hass's site
     * https://www.impulseadventure.com/elec/
     *
     * Converts a single joystick control into
     * a differential drive train motor control
     * which supports both driving and turning
     */
    public double[] differentialDrive(double x, double y) {

        // Motor outputs
        double nMotMixL;
        double nMotMixR;

        // Temporary variables
        double nMotPremixL;
        double nMotPremixR;

        // Pivot speed
        double nPivSpeed;

        // Balance scale between drive and pivot
        double fPivScale;

        // Calculate drive turn output due to Joystick X input
        if (y >= 0) {

            // Forward
            nMotPremixL = (x >= 0) ? 1 : (1 + x);
            nMotPremixR = (x >= 0) ? (1 - x) : 1;

        } else {

            // Backward
            nMotPremixL = (x >= 0) ? (1 - x) : 1;
            nMotPremixR = (x >= 0) ? 1 : (1 + x);
        }

        // Throttle the drive output by the joystick Y
        nMotPremixL *= y;
        nMotPremixR *= y;

        // Calculate pivot speed based on joystick X position
        nPivSpeed = x;

        // Blend pivot and drive based on joystick Y input
        fPivScale = (Math.abs(y) > pivotThreshold) ? 0 : (1 - Math.abs(y) / pivotThreshold);

        // Calculate final mix of driving and pivoting
        nMotMixL = ((1 - fPivScale) * nMotPremixL + fPivScale *  nPivSpeed);
        nMotMixR = ((1 - fPivScale) * nMotPremixR + fPivScale * -nPivSpeed);

        // Modify speed when left bumper is pressed
        nMotMixL *= gamepad1.b ? speedMultBoosted : speedMultDefault;
        nMotMixR *= gamepad1.b ? speedMultBoosted : speedMultDefault;

        // Make sure the output is in the range (-1..1)
        nMotMixL = Math.max(-1, Math.min(1, nMotMixL));
        nMotMixR = Math.max(-1, Math.min(1, nMotMixR));

        // Return the results
        return new double[]{ nMotMixL, nMotMixR };
    }
}