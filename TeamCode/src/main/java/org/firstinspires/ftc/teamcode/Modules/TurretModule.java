package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class TurretModule {
    private Servo homing_servo;
    private final static double HOMING_SERVO_UP = 0.7;
    private final static double HOMING_SERVO_DOWN = 0.4;
    public ElapsedTime homing_servo_timer = new ElapsedTime();
    public ElapsedTime homing_timer = new ElapsedTime();
    private boolean homing_timer_started = false;
    private DcMotorEx turret_motor;

    public ModuleStates current_state = ModuleStates.UNKNOWN;
    public boolean home_found = false;

    private static final double BACKWARD_POSITION = -100;
    private static final double FORWARD_POSITION = 693;
    private static final double LEFT_POSITION = 302;
    private static final double RIGHT_POSITION = 1000;

    private static final double COUNTERS_PER_DEGREE = (FORWARD_POSITION - BACKWARD_POSITION) / 180;

    public double target_position = 0;
    public PIDController turret_controller = new PIDController((float)0.001, 0 , (float)0.002);
    private static final double MAXIMUM_POWER = 1;

    public enum ModuleStates {
        UNKNOWN,
        WAITING_FOR_HOMING_UP,
        HOMING,
        WAITING_FOR_HOMING_DOWN,
        HOME,
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT,
        ACTIVE;
    }

    public TurretModule(Servo homing_servo, DcMotor turret_motor)
    {
        this.homing_servo = homing_servo;
        this.turret_motor = (DcMotorEx) turret_motor;
        this.turret_controller.set_acceleration_limit(0.2);
    }

    public void home_turret()
    {
        if (this.home_found && this.turret_motor.getCurrentPosition() > -100)
        {
            return;
        }
        this.home_found = false;
        this.homing_servo.setPosition(HOMING_SERVO_UP);
        this.homing_servo_timer.reset();
        this.homing_timer_started = false;
        this.current_state = ModuleStates.WAITING_FOR_HOMING_UP;
    }

    public void go_backwards()
    {
        this.current_state = ModuleStates.BACKWARD;
    }

    public void go_forwards()
    {
        this.current_state = ModuleStates.FORWARD;
    }

    public void go_right()
    {
        this.current_state = ModuleStates.RIGHT;
    }

    public void go_left()
    {
        this.current_state = ModuleStates.LEFT;
    }

    public void go_active()
    {
        this.current_state = ModuleStates.ACTIVE;
    }


    public void set_desired_turret_angle(double desired_angle_robot)
    {
        double count_from_forward = desired_angle_robot * COUNTERS_PER_DEGREE;

        double unlimited_counts = FORWARD_POSITION - count_from_forward;

        /* yo this is the limit stuff*/
        if (unlimited_counts < BACKWARD_POSITION)
        {
            this.target_position = BACKWARD_POSITION;
        } else if (unlimited_counts > FORWARD_POSITION) {
            this.target_position = FORWARD_POSITION;
        } else {
            this.target_position = unlimited_counts;
        }
    }

    public void update()
    {
        switch (this.current_state)
        {
            case UNKNOWN:
                break;

            case WAITING_FOR_HOMING_UP:
                this.homing_servo.setPosition(HOMING_SERVO_UP);

                if (this.homing_servo_timer.seconds() > 1)
                {
                    this.current_state = ModuleStates.HOMING;
                }
                break;

            case HOMING:
                /* If the homing servo is all the way up,
                    start moving blindly towards the servo.
                    Our goal is to hit it so we know where we are.
                 */
                this.turret_motor.setPower(0.3);

                if (!this.homing_timer_started)
                {
                    this.homing_timer.reset();
                    this.homing_timer_started = true;
                }

                if (this.turret_motor.getVelocity(AngleUnit.DEGREES) < 30 &&
                        this.homing_timer_started && this.homing_timer.seconds() > 0.25)
                {
                    this.turret_motor.setPower(0);

                    this.turret_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    this.turret_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    this.homing_servo.setPosition(HOMING_SERVO_DOWN);
                    this.homing_servo_timer.reset();

                    this.home_found = true;
                    this.current_state = ModuleStates.WAITING_FOR_HOMING_DOWN;
                }
                break;

            case WAITING_FOR_HOMING_DOWN:
                this.homing_servo.setPosition(HOMING_SERVO_DOWN);

                if (this.homing_servo_timer.seconds() > 1)
                {
                    this.current_state = ModuleStates.HOME;
                }
                break;

            case BACKWARD:
                if (this.home_found)
                {
                    this.target_position = BACKWARD_POSITION;
                    drive_to_target();
                }
                break;

            case FORWARD:
                if (this.home_found)
                {
                    this.target_position = FORWARD_POSITION;
                    drive_to_target();
                }
                break;

            case LEFT:
                if (this.home_found)
                {
                    this.target_position = LEFT_POSITION;
                    drive_to_target();
                }
                break;

            case RIGHT:
                if (this.home_found)
                {
                    this.target_position = RIGHT_POSITION;
                    drive_to_target();
                }
                break;

            case ACTIVE:
                if (this.home_found)
                {
                    drive_to_target();
                }
                break;

            case HOME:
                break;
        }
    }

    private void drive_to_target()
    {
        this.turret_controller.set_target((float)this.target_position);

        double turret_motor_power = this.turret_controller.update(this.turret_motor.getCurrentPosition());

        double limited_power;

        if (Math.abs(turret_motor_power) > MAXIMUM_POWER)
        {
            limited_power = MAXIMUM_POWER * Math.signum(turret_motor_power);
        } else {
            limited_power = turret_motor_power;
        }

        this.turret_motor.setPower(limited_power);
    }
}
