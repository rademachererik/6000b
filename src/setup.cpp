#include "setup.hpp"
#include <cassert>
#include <string>
#include "MotorSaver.hpp"
#include "Point.hpp"
#include "main.h"
#include "pid.hpp"

// motors
pros::Motor mtr3(19);  // intake
pros::Motor mtr6(17);  // flywheel
pros::Motor mtr7(11);  // drfb
pros::Motor mtr8(16);  // claw
/* bad ports:
5,
15(claw),
18(flywheel: during practice),
13(claw: during practice),
12 (claw: first time turning on the robot in the morning)
*/
// bad ports: 11, 12, 14, 15, 1, 2, 3, 4, 5, 6, 7

// motor savers
MotorSaver dlSaver(35);
MotorSaver drSaver(35);
MotorSaver drfbSaver(25);
MotorSaver clawSaver(35);
MotorSaver intakeSaver(40);
MotorSaver flySaver(40);

pros::Controller ctlr(pros::E_CONTROLLER_MASTER);
pros::Controller ctlr2(pros::E_CONTROLLER_PARTNER);
using pros::delay;
// sensors
pros::ADILineSensor* ballSensL;
pros::ADILineSensor* ballSensR;
pros::ADIEncoder* perpindicularWheelEnc;
pros::ADIEncoder* DLEnc;
pros::ADIEncoder* DREnc;

//----------- Constants ----------------
const int drfbMaxPos = 2390, drfbPos0 = -60, drfbMinPos = -80, drfbPos1 = 1220, drfbPos1Plus = 1493, drfbPos2 = 1793, drfbPos2Plus = 2280;
const int drfbMinClaw0 = 350, drfbMaxClaw0 = 640, drfbMinClaw1 = 1087, drfb18Max = 350;

const double dShotSpeed1 = 2.62, dShotSpeed2 = 2.83;
const double sShotSpeed = 2.9;

const int intakeShootTicks = -600;

const int dblClickTime = 450, claw180 = 1370;
const double /*ticksPerInch = 52.746, */ ticksPerInchADI = 35.2426, ticksPerRadian = 368.309;
const double PI = 3.14159265358979323846;
const int BIL = 1000000000, MIL = 1000000;

/*
 #### ##    ## ########    ###    ##    ## ########
  ##  ###   ##    ##      ## ##   ##   ##  ##
  ##  ####  ##    ##     ##   ##  ##  ##   ##
  ##  ## ## ##    ##    ##     ## #####    ######
  ##  ##  ####    ##    ######### ##  ##   ##
  ##  ##   ###    ##    ##     ## ##   ##  ##
 #### ##    ##    ##    ##     ## ##    ## ########
*/
namespace intake {
int requestedVoltage = 0;
int altT0;
double target;
int wait;
double posBias = 0.0;
void init(double tgt, int w) {
    posBias -= getIntakePos();
    target = tgt;
    wait = w;
    intakePid.doneTime = BIL;
}
}  // namespace intake
void pidIntakeInit(double target, int wait) { intake::init(target, wait); }
double getIntakePos() { return mtr3.get_position() + intake::posBias; }
bool pidIntake() {
    using intake::target;
    using intake::wait;
    intakePid.sensVal = getIntakePos();
    intakePid.target = target;
    setIntake(lround(intakeSlew.update(intakePid.update())));
    return millis() - intakePid.doneTime > wait;
}
void setIntake(int n) {  // +: front, -: back
    n = clamp(n, -12000, 12000);
    // n = intakeSlew.update(n);
    n = intakeSaver.getPwr(n, mtr3.get_position());
    mtr3.move_voltage(n);
    intake::requestedVoltage = n;
}
int getIntakeVoltage() { return intake::requestedVoltage; }
void setIntake(IntakeState is) {
    static IntakeState prev = IntakeState::NONE;
    if (is == IntakeState::NONE) {
        setIntake(0);
    } else if (is == IntakeState::FRONT) {
        setIntake(12000);
    } else if (is == IntakeState::BACK) {
        setIntake(-12000);
    } else if (is == IntakeState::BACK_SLOW) {
        setIntake(-7000);
    } else if (is == IntakeState::ALTERNATE) {
        if (isBallIn()) {
            setIntake(1000);
        } else {
            if (prev != IntakeState::ALTERNATE) intake::altT0 = millis();
            if (((millis() - intake::altT0) / 400) % 3 < 2) {
                setIntake(-12000);
            } else {
                setIntake(12000);
            }
        }
    }
    prev = is;
}
IntakeState getISLoad() {
    if (isBallIn()) {
        return IntakeState::NONE;
    } else {
        return IntakeState::ALTERNATE;
    }
}
int getBallSensL() { return ballSensL->get_value(); }
int getBallSensR() { return ballSensR->get_value(); }
bool isBallIn() { return getBallSensL() < 2000 || getBallSensR() < 2000; }

//----------- DRFB functions ---------
int drfb_requested_voltage = 0;
int drfbPowerLimit = 3500;
int drfbFullRangePowerLimit = 12000;
void setDrfb(int n) {
    n = clamp(n, -drfbFullRangePowerLimit, drfbFullRangePowerLimit);
    if (getDrfb() < drfbMinPos + 150 && n < -drfbPowerLimit) n = -drfbPowerLimit;
    if (getDrfb() > drfbMaxPos - 150 && n > drfbPowerLimit) n = drfbPowerLimit;
    n = drfbSlew.update(n);
    n = drfbSaver.getPwr(n, getDrfb());
    mtr7.move_voltage(n);
    drfb_requested_voltage = n;
}
void setDrfbDumb(int n) {
    mtr7.move_voltage(n);
    drfb_requested_voltage = n;
}
double drfbIMEBias = 0;
double getDrfb() { return drfbIMEBias + mtr7.get_position(); }
int getDrfbVoltage() { return drfb_requested_voltage; }
int getDrfbCurrent() { return mtr7.get_current_draw(); }
int drfbPidBias = 0;
bool pidDrfb(double pos, int wait) {
    drfbPid.sensVal = getDrfb();
    int out = drfbPid.update();
    setDrfb(drfbPidBias + out);
    if (drfbPid.doneTime + wait < millis()) return true;
}
void pidDrfb() { pidDrfb(drfbPid.target, 9999999); }

/*
  ######  ##          ###    ##      ##
 ##    ## ##         ## ##   ##  ##  ##
 ##       ##        ##   ##  ##  ##  ##
 ##       ##       ##     ## ##  ##  ##
 ##       ##       ######### ##  ##  ##
 ##    ## ##       ##     ## ##  ##  ##
  ######  ######## ##     ##  ###  ###
*/

namespace clawOpctl {
double bias = 0.0;
}
int claw_requested_voltage = 0;
int clawPowerLimit = 12000;
void setClaw(int n, bool limit) {
    n = clamp(n, -clawPowerLimit, clawPowerLimit);
    if (limit) {
        if (getDrfb() < drfbMinClaw0 || (getDrfb() > drfbMaxClaw0 && getDrfb() < drfbMinClaw1)) n = 0;
    }
    // int maxPwr = 1200;
    // if (getClaw() < 80 && n < -maxPwr) n = -maxPwr;
    // if (getClaw() > claw180 - 80 && n > maxPwr) n = maxPwr;
    n = clawSaver.getPwr(n, mtr8.get_position());
    n = clawSlew.update(n);
    mtr8.move_voltage(n);
    claw_requested_voltage = n;
}
void setClaw(int n) { setClaw(n, false); }
void setClawDumb(int n) {
    mtr8.move_voltage(n);
    claw_requested_voltage = n;
}
void setClawPosition(double pos) { clawOpctl::bias += pos - getClaw(); };
double getClaw() { return mtr8.get_position() + clawOpctl::bias; }
int getClawVoltage() { return claw_requested_voltage; }
bool pidClaw(double a, int wait) {
    clawPid.target = a;
    clawPid.sensVal = getClaw();
    setClaw(clawPid.update());
    if (clawPid.doneTime + wait < millis()) return true;
    return false;
}
void pidClaw() { pidClaw(clawPid.target, 999999); }

/*
 ######## ##       ##    ## ##      ## ##     ## ######## ######## ##
 ##       ##        ##  ##  ##  ##  ## ##     ## ##       ##       ##
 ##       ##         ####   ##  ##  ## ##     ## ##       ##       ##
 ######   ##          ##    ##  ##  ## ######### ######   ######   ##
 ##       ##          ##    ##  ##  ## ##     ## ##       ##       ##
 ##       ##          ##    ##  ##  ## ##     ## ##       ##       ##
 ##       ########    ##     ###  ###  ##     ## ######## ######## ########
*/
namespace flywheel {
int requestedVoltage = 0;
double target = 0;
int wait = 999999;
void init(double t, int w) {
    target = t;
    wait = w;
    flywheelPid.doneTime = BIL;
}
}  // namespace flywheel
void setFlywheel(int n) {
    n = clamp(n, 0, 12000);
    // n = flywheelSlew.update(n);
    // n = flySaver.getPwr(n, getFlywheel());
    mtr6.move_voltage(-n);
    flywheel::requestedVoltage = n;
}
double getFlywheel() { return -mtr6.get_position(); }
double getFlywheelFromMotor() { return -3.1 / 200.0 * mtr6.get_actual_velocity(); }
int getFlywheelVoltage() { return flywheel::requestedVoltage; }

double FWSpeeds[][2] = {{0, 0}, {1.0, 4200}, {2.0, 7700}, {2.2, 8400}, {2.4, 9100}, {2.5, 9850}, {2.6, 10200}, {2.65, 10400}, {2.7, 10600}, {2.8, 10600}, {2.9, 11480}};
void pidFlywheelInit(double speed, int wait) { flywheel::init(speed, wait); }
bool pidFlywheel() {
    double speed = flywheel::target;
    static double prevSpeed = 0.0;
    static int prevT = 0, prevPosition = 0;
    static double output = 0.0;
    static bool crossedTarget = false;
    static int dir = 1;
    static int prevUpdateT = -9999;
    static double sumVel = 0.0;
    static int numVel = 0;
    if (fabs(speed - prevSpeed) > 0.1) {
        output = 0.0;
        crossedTarget = false;
        if (speed > prevSpeed) {
            dir = 1;
        } else {
            dir = -1;
        }
    }
    prevSpeed = speed;
    double dt = millis() - prevT;
    prevT = millis();
    int FWSpeedsLen = sizeof(FWSpeeds) / sizeof(double[2]);
    int bias = 0;
    double smallestDist = 99999999;
    for (int i = 0; i < FWSpeedsLen; i++) {
        double dist = fabs(speed - FWSpeeds[i][0]);
        if (dist < smallestDist) {
            bias = FWSpeeds[i][1];
            smallestDist = dist;
        }
    }
    if (dt < 1000 && dt > 0) {
        sumVel += getFlywheelFromMotor();
        numVel++;
        if (millis() - prevUpdateT > 50) {  // 95

            flywheelPid.sensVal = sumVel / numVel;  //(getFlywheel() - prevPosition) / (millis() - prevUpdateT);
            sumVel = 0.0;
            numVel = 0;
            prevUpdateT = millis();
            prevPosition = getFlywheel();
            flywheelPid.target = speed;
            if (!crossedTarget) {
                if (flywheelPid.sensVal > flywheelPid.target - 0.1 * dir) {
                    if (dir == 1) {
                        crossedTarget = true;
                    } else {
                        setFlywheel(0);
                    }
                } else if (flywheelPid.sensVal < flywheelPid.target - 0.1 * dir) {
                    if (dir == -1) {
                        crossedTarget = true;
                    } else {
                        setFlywheel(12000);
                    }
                }
            }
            if (crossedTarget) {
                output += flywheelPid.update();
                output = clamp(output, -6000.0, 6000.0);
                setFlywheel(bias + lround(output));
            }
        }
    }
    return millis() - flywheelPid.doneTime > flywheel::wait;
}
bool isPidFlywheelDone() { return millis() - flywheelPid.doneTime > flywheel::wait; }

/*
 ##     ## ####  ######   ######
 ###   ###  ##  ##    ## ##    ##
 #### ####  ##  ##       ##
 ## ### ##  ##   ######  ##
 ##     ##  ##        ## ##
 ##     ##  ##  ##    ## ##    ##
 ##     ## ####  ######   ######
*/

int clamp(int n, int a, int b) { return n < a ? a : (n > b ? b : n); }
double clamp(double n, double a, double b) { return n < a ? a : (n > b ? b : n); }
Point polarToRect(double mag, double angle) {
    Point p(mag * cos(angle), mag * sin(angle));
    return p;
}

const int ctlrIdxLeft = 0, ctlrIdxUp = 1, ctlrIdxRight = 2, ctlrIdxDown = 3, ctlrIdxY = 4, ctlrIdxX = 5, ctlrIdxA = 6, ctlrIdxB = 7, ctlrIdxL1 = 8, ctlrIdxL2 = 9, ctlrIdxR1 = 10, ctlrIdxR2 = 11;
const std::string clickIdxNames[] = {"Left", "Up", "Right", "Down", "Y", "X", "A", "B", "L1", "L2", "R1", "R2"};
const pros::controller_digital_e_t clickIdxIds[] = {DIGITAL_LEFT, DIGITAL_UP, DIGITAL_RIGHT, DIGITAL_DOWN, DIGITAL_Y, DIGITAL_X, DIGITAL_A, DIGITAL_B, DIGITAL_L1, DIGITAL_L2, DIGITAL_R1, DIGITAL_R2};
bool** getAllClicks() {
    static bool prevClicks[] = {false, false, false, false, false, false, false, false, false, false, false, false};
    static int prevTimes[] = {-9999999, -9999999, -9999999, -9999999, -9999999, -9999999, -9999999, -9999999, -9999999, -9999999, -9999999, -9999999};
    bool curClicks[12];
    static bool dblClicks[12] = {false, false, false, false, false, false, false, false, false, false, false, false};
    bool** allClicks = new bool*[3];
    for (int i = 0; i < 3; i++) { allClicks[i] = new bool[12]; }
    for (int i = 0; i < 12; i++) {
        curClicks[i] = ctlr.get_digital(clickIdxIds[i]);
        if (!curClicks[i]) dblClicks[i] = false;
        if (curClicks[i] && !prevClicks[i]) {
            // double tap
            if (millis() - prevTimes[i] < dblClickTime) dblClicks[i] = true;
            prevTimes[i] = millis();
        }
        allClicks[0][i] = prevClicks[i];
        allClicks[1][i] = curClicks[i];
        allClicks[2][i] = dblClicks[i];
        prevClicks[i] = curClicks[i];
    }
    return allClicks;
}
void printAllClicks(int line, bool** allClicks) {
    std::string line1 = "prevClicks: ";
    for (int i = 0; i < 12; i++) { line1 += (allClicks[0][i] ? (clickIdxNames[i] + ", ") : ""); }
    line1 = line1.substr(0, line1.size() - 2);
    std::string line2 = "curClicks: ";
    for (int i = 0; i < 12; i++) { line2 += (allClicks[1][i] ? (clickIdxNames[i] + ", ") : ""); }
    line2 = line2.substr(0, line2.size() - 2);
    std::string line3 = "dblClicks: ";
    for (int i = 0; i < 12; i++) { line3 += (allClicks[2][i] ? (clickIdxNames[i] + ", ") : ""); }
    line3 = line3.substr(0, line3.size() - 2);
    pros::lcd::print(line, line1.c_str());
    pros::lcd::print(line + 1, line2.c_str());
    pros::lcd::print(line + 2, line3.c_str());
}
int millis() { return pros::millis(); }
void stopMotors() {
    setDrfb(0);
    setDL(0);
    setDR(0);
    setClaw(0);
    setFlywheel(0);
    setIntake(0);
}
void stopMotorsBlock() {
    while (1) {
        setDrfb(0);
        setDL(0);
        setDR(0);
        setClaw(0);
        setFlywheel(0);
        setIntake(0);
        delay(10);
    }
}
void printPidValues() {
    printf("drfb%2d %4d/%4d fly%d %1.3f/%1.3f claw%2d %4d/%4d intake%2d %4d/%4d\n", (int)(getDrfbVoltage() / 1000 + 0.5), (int)drfbPid.sensVal, (int)drfbPid.target, getFlywheelVoltage(), flywheelPid.sensVal, flywheelPid.target, (int)(getClawVoltage() / 1000 + 0.5), (int)clawPid.sensVal, (int)clawPid.target, (int)(getIntakeVoltage() / 1000 + 0.5), (int)intakePid.sensVal, (int)intakePid.target);
    std::cout << std::endl;
}
extern Point g_target;
void printDrivePidValues() {
    printf("DL%d DR%d vel %f drive %3.2f/%3.2f turn %2.2f/%2.2f curve %2.2f/%2.2f x %3.2f/%3.2f y %3.2f/%3.2f a %.2f\n", (int)(getDLVoltage() / 100 + 0.5), (int)(getDRVoltage() / 100 + 0.5), getDriveVel(), drivePid.sensVal, drivePid.target, turnPid.sensVal, turnPid.target, curvePid.sensVal, curvePid.target, odometry.getX(), g_target.x, odometry.getY(), g_target.y, odometry.getA());
    std::cout << std::endl;
}
void printPidSweep() { printf("DL%d %.1f/%.1f DR%d %.1f/%.1f\n", getDLVoltage, DLPid.sensVal, DLPid.target, getDRVoltage, DRPid.sensVal, DRPid.target); }
void odoTaskRun(void* param) {
    while (true) {
        odometry.update();
        delay(3);
    }
}
void startOdoTask() {
    std::string s("param");
    pros::Task odoTask(odoTaskRun, &s);
}

void opctlDrive(int driveDir) {
    int joy[] = {lround(ctlr.get_analog(ANALOG_RIGHT_X) * 12000.0 / 127.0), lround(driveDir * ctlr.get_analog(ANALOG_LEFT_Y) * 12000.0 / 127.0)};
    if (abs(joy[0]) < 10) joy[0] = 0;
    if (abs(joy[1]) < 10) joy[1] = 0;
    setDL(joy[1] + joy[0]);
    setDR(joy[1] - joy[0]);
}
/*
  ######  ######## ######## ##     ## ########
 ##    ## ##          ##    ##     ## ##     ##
 ##       ##          ##    ##     ## ##     ##
  ######  ######      ##    ##     ## ########
       ## ##          ##    ##     ## ##
 ##    ## ##          ##    ##     ## ##
  ######  ########    ##     #######  ##
*/

void setDrfbParams(bool auton) {
    if (auton) {
        drfbPid.kp = 45.0;
        drfbPid.ki = 0.5;
        drfbPid.unwind = 20;
        drfbPid.iActiveZone = 150;
        drfbPid.maxIntegral = 3000;
        drfbPid.kd = 4500;
    } else {
        drfbPid.kp = 7.0;
        drfbPid.ki = 0.0;
        drfbPid.kd = 0.0;
    }
}
void setDriveSlew(bool auton) {
    if (auton) {
        DLSlew.slewRate = DRSlew.slewRate = 120;
    } else {
        DLSlew.slewRate = DRSlew.slewRate = 120;
    }
}
void setup() {
    static bool first = true;
    if (!first) {
        printf("setup already.\n");
        return;
    } else {
        printf("setting up...\n");
    }
    flywheelSlew.slewRate = 999999;  // 60;
    flywheelPid.kp = 1000.0;         // 3500
    flywheelPid.kd = 250000.0;       // 300000
    flywheelPid.DONE_ZONE = 0.1;
    flySaver.setConstants(1, 1, 0, 0);

    intakePid.kp = 70;
    intakePid.ki = 0.05;
    intakePid.kd = 3000;
    intakePid.maxIntegral = 4000;
    intakePid.iActiveZone = 300;
    intakeSlew.slewRate = 200;
    intakePid.unwind = 0;
    intakePid.DONE_ZONE = 50;
    intakeSaver.setConstants(.15, 0.3, 0.05, 0.2);

    clawPid.kp = 90.0;
    clawPid.ki = 0.03;
    clawPid.kd = 2500.0;
    clawPid.iActiveZone = 300;
    clawPid.maxIntegral = 4000;
    clawPid.unwind = 0;
    clawSlew.slewRate = 200;
    clawSaver.setConstants(0.5, .3, 0.3, .15);

    drfbSlew.slewRate = 99999;
    setDrfbParams(true);
    drfbSaver.setConstants(0.15, 0.6, 0.01, 0.1);
    drfbPid.DONE_ZONE = 100;
    drfbPid.target = drfbPos0;

    setDriveSlew(false);
    dlSaver.setConstants(1, 1, 0, 0);
    drSaver.setConstants(1, 1, 0, 0);

    drivePid.kp = 2000;
    drivePid.ki = 8;
    drivePid.iActiveZone = 2;
    drivePid.maxIntegral = 5000;
    drivePid.kd = 110000;
    drivePid.DONE_ZONE = 1.0;

    DLPid.kp = 2000 / ticksPerInchADI;
    DLPid.ki = 8 / ticksPerInchADI;
    DLPid.kd = 110000 / ticksPerInchADI;
    DLPid.iActiveZone = 2 * ticksPerInchADI;
    DLPid.maxIntegral = 5000;
    DLPid.DONE_ZONE = ticksPerInchADI;
    DRPid = DLPid;

    turnPid.kp = 26000;
    turnPid.ki = 250;
    turnPid.kd = 2000000;
    turnPid.iActiveZone = 0.1;
    turnPid.unwind = 0.003;
    turnPid.maxIntegral = 5000;
    turnPid.DONE_ZONE = PI / 20;

    curvePid.kp = 50000;

    curveVelPid.kp = 10000000;

    drfbPid.target = drfbPos0;
    clawPid.target = 0;
    flywheelPid.target = 0;

    ballSensL = new pros::ADILineSensor(7);
    ballSensR = new pros::ADILineSensor(8);
    perpindicularWheelEnc = new pros::ADIEncoder(3, 4, false);
    DLEnc = new pros::ADIEncoder(1, 2, false);
    DREnc = new pros::ADIEncoder(5, 6, false);

    // ballSensL->calibrate(); fix this: calibrate in a seperate thread
    // ballSensR->calibrate();
    int t0 = millis();
    // while (millis() - t0 < 800) { int n = getDL() + getDR() + getDS();delay(10); }
    first = false;
}

void morningRoutine() {
    static bool first = true;
    if (first) {
        printf("good morning.\n");
    } else {
        printf("its not the morning any more...\n");
        return;
    }
    while (!ctlr.get_digital(DIGITAL_R1) && !ctlr.get_digital(DIGITAL_R2)) {
        pros::lcd::print(1, "drfb, claw down. then press R");
        delay(10);
    }
    int t0 = millis();
    while (millis() - t0 < 600) {
        setDrfbDumb(-6000);
        setClawDumb(-2500);
        delay(10);
    }
    drfbIMEBias = -getDrfb() - 98;
    clawOpctl::bias = -getClaw() - 70;
    setDrfb(0);
    setClaw(0);
    pros::lcd::print(1, "Move the Drive");
    odometry.reset();
    double prevDL = 0.0, prevDR = 0.0, prevDS = 0.0;
    bool initDL = false, initDR = false, initDS = false;
    while (1) {
        // printf("%d %d %d %d %d <%d %d>\n", (int)getDL(), (int)getDR(), (int)getDS(), getBallSensL(), getBallSensR(), (int)mtr1.get_position(), (int)mtr4.get_position());
        if (fabs(getDL() - prevDL) > 0.001) initDL = true;
        if (fabs(getDR() - prevDR) > 0.001) initDR = true;
        if (fabs(getDS() - prevDS) > 0.001) initDS = true;
        if (initDL && initDR && initDS) break;
        prevDL = getDL();
        prevDR = getDR();
        prevDS = getDS();
        delay(10);
    }
    pros::lcd::print(1, "Press A to confirm");
    while (!ctlr.get_digital(DIGITAL_A)) delay(10);
    first = false;
    startOdoTask();
}
