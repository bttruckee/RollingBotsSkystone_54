package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

public class Annika
{
    //Number of wheels
    private final int NUM_WHEELS = 4;

    //Speed of the arm to lock its position
    private static final double LOCKED_SPEED = 0.2;
    //Declaraton of the hardwaremap object
    private HardwareMap hwMap;

    /*Defines the motors
    0 = leftFront
    1 = rightFront
    2 =leftRear
    3 = rightRear

    4 = arm (set in port 0 of the other expansion hub)*/
    private DcMotor[] motors;

    //Defines the array of motor power
    private double[] motorPower;

    //Defines the array of locked motors
    private boolean[] lockedMotors;

    //The constant the rear wheels are multiplied by when strafing (to prevent oversteer)
    private final double strafeMultiplierLeft = 0.9;
    private final double strafeMultiplierRight = 1;

    //Defines hashmaps to get motor and servo indexes
    public static final HashMap<String, Integer> MotorIndexes = new HashMap<String, Integer>();
        static{
            MotorIndexes.put("leftFront", 0);
            MotorIndexes.put("rightFront", 1);
            MotorIndexes.put("leftRear", 2);
            MotorIndexes.put("rightRear", 3);
            MotorIndexes.put("arm", 4);
        }

    public static final HashMap<String, Integer> ServoIndexes = new HashMap<String, Integer>();
        static{
            ServoIndexes.put("groundLock", 0);
            ServoIndexes.put("finger", 1);
        }

    //Defines the servo positions ([servo index (groundLock, wrist, finger)], [position (open/up, closed/down)]
    private static final double[][] SERVO_POSITIONS = {{0.5, 1}, {0.5, 0.0}};

    //Defines the motors and servos for the arm
    //private DcMotor arm;

    //An array of the servos (groundLock and finger)
    private Servo[] servos;

    //Whether or not the arm is locked in position
    //private boolean armLocked;

    public Annika()
    {}

    public void init(HardwareMap hwMap)
    {
        //Define the arrays
        motors = new DcMotor[5];
        motorPower = new double[5];
        lockedMotors = new boolean[5];

        servos = new Servo[2];

        //Set hardwaremap to parameter
        this.hwMap = hwMap;

        //Set motor/servo variables to motors/servos in hwMap
        motors[0] = hwMap.get(DcMotor.class, "left_front");
        motors[1] = hwMap.get(DcMotor.class, "right_front");
        motors[2] = hwMap.get(DcMotor.class, "left_rear");
        motors[3] = hwMap.get(DcMotor.class, "right_rear");

        motors[4] = hwMap.get(DcMotor.class, "arm");

        servos[Annika.ServoIndexes.get("groundLock")] = hwMap.get(Servo.class, "ground_lock");

        servos[Annika.ServoIndexes.get("finger")] = hwMap.get(Servo.class, "finger");

        //Set motor and servo directions
        motors[0].setDirection(DcMotor.Direction.REVERSE);
        motors[1].setDirection(DcMotor.Direction.FORWARD);
        motors[2].setDirection(DcMotor.Direction.REVERSE);
        motors[3].setDirection(DcMotor.Direction.FORWARD);

        motors[4].setDirection(DcMotor.Direction.REVERSE);
        //motors[4].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motors[4].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servos[Annika.ServoIndexes.get("groundLock")].setDirection(Servo.Direction.FORWARD);

        servos[Annika.ServoIndexes.get("finger")].setDirection(Servo.Direction.REVERSE);

        //Sets motor modes and sets initializes motors and servos
        for(DcMotor motor: motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(0);
        }

        servos[Annika.ServoIndexes.get("groundLock")].setPosition(SERVO_POSITIONS[Annika.ServoIndexes.get("groundLock")][0]);

        servos[Annika.ServoIndexes.get("finger")].setPosition(SERVO_POSITIONS[Annika.ServoIndexes.get("finger")][0]);
    }

    /**
     * Sets the forward speed of the robot
     *
     * @param spd the speed the motors will be set to (positive for forward, negative for backward)
     */
    public void setForwardSpeed(double spd)
    {
        for(int i = 0; i < NUM_WHEELS; i++)
        {
            motorPower[i] = spd;
        }
    }

    /**
     * Sets the rotational speed of the robot
     *
     * @param spd the speed the motors will be set to (positive for right, negative for left)
     */
    public void setTurnSpeed(double spd)
    {
        for(int i = 0; i < NUM_WHEELS; i++)
        {
            if(i % 2 == 0)
                motorPower[i] = spd;
            else
                motorPower[i] = -spd;
        }
    }

    /**
     * Sets the rotational speed of the robot
     *
     * @param spd the speed the motors will be set to (positive for right, negative for left)
     */
    public void setStrafeSpeed(double spd)
    {
        for(int i = 0; i < NUM_WHEELS; i++)
        {
            if(i % 3 == 0)
                motorPower[i] = spd;
            else
                motorPower[i] = -spd;

            if(i < 2)
            {
                if(spd > 0)
                    motorPower[i] = motorPower[i] * strafeMultiplierRight;
                else
                    motorPower[i] = motorPower[i] * strafeMultiplierLeft;
            }
        }
    }

    //Sets the motors to the current values of motorPower
    public void move()
    {
        for(int i = 0; i < NUM_WHEELS; i++)
        {
            if(motorPower[i] != 0)
            {
                lockMotor(i,false);
                motors[i].setPower(motorPower[i]);
            }
            else
            {
                lockMotor(i,true);
                motors[i].setPower(LOCKED_SPEED);
            }
            //motors[i].setPower(motorPower[i]);
        }
    }

    //Sets the power of the wheels to four separate values. Used for testing motors
    public void testWheels(double leftFrontPower, double rightFrontPower, double leftRearPower, double rightRearPower)
    {
        motorPower[MotorIndexes.get("leftFront")] = leftFrontPower;
        motorPower[MotorIndexes.get("rightFront")] = rightFrontPower;
        motorPower[MotorIndexes.get("leftRear")] = leftRearPower;
        motorPower[MotorIndexes.get("rightRear")] = rightRearPower;

    }

    /**
     * Moves the arm up and down
     * Keeps the arm in place if the speed is 0
     *
     * @param spd Speed the arm moves
     * @return the position of the arm
     */
    public void moveArm(double spd)
    {
        if(spd != 0)
        {
            lockMotor(MotorIndexes.get("arm"),false);
            motors[4].setPower(spd);
        }
        else
        {
            lockMotor(MotorIndexes.get("arm"),true);
            motors[4].setPower(LOCKED_SPEED);
        }
    }

    //Encoders not working as intended, suspending code until functional
    /**
     * Makes the wheels to travel a given distance in a given direction
     * @param speed = the speed to set the wheels to
     * @param direction = the direction the robot is travelling (0 = forward, 1 = turn, 2 = strafe)
     * @param distance = how many inches the robot will travel
     */
    /*public void encoderDrive(double speed, int direction, double distance)
    {
        if(direction <= 2 && direction <= 0) {
            int[] newPositions = new int[motors.length];

            //Set the motor positions
            for (int i = 0; i < motors.length; i++) {
                if(direction == 0)
                {
                    newPositions[i] = motors[i].getCurrentPosition() + (int)(distance * Jacobie.COUNTS_PER_INCH);
                }
                else if(direction == 1)
                {
                    if (i % 2 == 0)
                        newPositions[i] = motors[i].getCurrentPosition() + (int)(distance * Jacobie.COUNTS_PER_INCH);
                    else
                        newPositions[i] = motors[i].getCurrentPosition() - (int)(distance * Jacobie.COUNTS_PER_INCH);
                }
                else {
                    if (i % 3 == 0)
                        newPositions[i] = motors[i].getCurrentPosition() + (int)(distance * Jacobie.COUNTS_PER_INCH);
                    else
                        newPositions[i] = motors[i].getCurrentPosition() - (int)(distance * Jacobie.COUNTS_PER_INCH);
                }
            }

            //Setting the motors to move
            for(int m = 0; m < motors.length; m++)
            {
                motors[m].setTargetPosition(newPositions[m]);
                motors[m].setPower(speed);
                motors[m].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            while (motorsBusy())
            { }

            //Stop the motors and disable RUN_TO_POSITION
            for(DcMotor motor: motors)
            {
                motor.setPower(0);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }*/

    private boolean motorsBusy()
    {
        for(DcMotor motor: motors)
        {
            if(motor.isBusy())
            {
                return true;
            }
        }
        return false;
    }

    public void testEncoders(int motorIndex, int position)
    {
        motors[motorIndex].setTargetPosition(position);
        motors[motorIndex].setPower(1);
        motors[motorIndex].setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (motors[motorIndex].isBusy())
        { }

        motors[motorIndex].setPower(0);
        motors[motorIndex].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Activates/Deactivates "lock mode" of the given motor
     *
     * @param motorInedex the index of the motor being called
     * @param toLocked
     */
    private void lockMotor(int motorInedex, boolean toLocked)
    {
        if(lockedMotors[motorInedex] != toLocked) {
            lockedMotors[motorInedex] = toLocked;
            motors[motorInedex].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (toLocked)
            {
                motors[motorInedex].setTargetPosition(motors[motorInedex].getCurrentPosition());
                motors[motorInedex].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
    }

    public double getServoPosition(int servo)
    {
        return servos[servo].getPosition();
    }

    /**
     * Returns the position of the given wheel motor
     * @param motorIndex index of motor in wheelMotor array
     * @return the position of the encoder
     */
    public double getMotorPosition(int motorIndex)
    {
        return motors[motorIndex].getCurrentPosition();
    }

    /**
     *
     * @param index = the index of the applied servo (ServoIndexes.get("Name"));
     * @param isOpen = whether or not the servo is in the "open" position
     */
    public void setServo (int index, boolean isOpen)
    {
        if(isOpen)
            servos[index].setPosition(SERVO_POSITIONS[index][0]);
        else
            servos[index].setPosition(SERVO_POSITIONS[index][1]);
    }
}