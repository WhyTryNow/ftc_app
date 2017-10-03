import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Test Bed", group = "practice")

public class Testbed extends OpMode {

    public DcMotor dcMotor;
    public Servo servoMotor;
    ModernRoboticsTouchSensor touchSensor;
    ModernRoboticsI2cRangeSensor rangeSensor;

    @Override
    public void init() {

        // display an initialization string on the robot controller
        telemetry.addData("Status", "Testbed Initialized");

        // initialize motor and sensor objects
        dcMotor = hardwareMap.get(DcMotor.class, "motor_dc");
        servoMotor = hardwareMap.get(Servo.class, "motor_servo");
        touchSensor = hardwareMap.get(ModernRoboticsTouchSensor.class, "sensor_touch");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
    }

    @Override
    public void loop() {

        // gamepad #1 left stick x-axis controls the DC motor power level
        dcMotor.setPower(gamepad1.left_stick_x);

        // gamepad #2 right stick x-axis controls the servo motor position
        servoMotor.setPosition(gamepad1.right_stick_x);

        // display the button status
        telemetry.addData("button value", touchSensor.getValue());

        // display range sensor data
        telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
        telemetry.addData("raw optical", rangeSensor.rawOptical());
        telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
        telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));

        // push telemetry output to driver controller
        telemetry.update();
    }
}
