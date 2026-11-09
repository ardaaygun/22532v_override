#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// eski kodlar: yedek/main_eski_portlar.cpp (önceki port düzeni), yedek/main_temel.cpp (LemLib'siz)

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// ===========================================================================
// PORTLAR
// ===========================================================================
// şase: 9, 10, 20 sol (ters) / 1, 2, 19 sağ
// IMU: 13
// tracking wheel rotation: 14 dikey (yatay yok)
// DR4B: motorlar 11, 12 - rotation 7
// pnömatik: claw H

// ===========================================================================
// ŞASE
// ===========================================================================
// her tarafta 11W, 11W, 5.5W
// 11W: mavi kartuş 600 rpm, 36T:48T -> 450 rpm
// 5.5W: 200 rpm, 36T:12T ile 600 rpm'e çıkıp aynı 36T:48T'ye aktarıyor -> 450 rpm
pros::MotorGroup leftMotors({-9, -10, -20}, pros::MotorGearset::blue); // sol - ters
pros::MotorGroup rightMotors({1, 2, 19}, pros::MotorGearset::blue); // sağ

pros::Imu imu(13);

// dikey tracking wheel - rotation sensöre doğrudan bağlı
// ileri itince değer azalıyorsa portu negatif yap: pros::Rotation verticalEnc(-14);
pros::Rotation verticalEnc(-14);
// 2" omni, merkezin 0.275" sağında
// offset: robot merkezinin solundaysa negatif, sağındaysa pozitif
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 0.275);
// TODO: yatay tracking wheel takılınca:
// pros::Rotation horizontalEnc(PORT);
// lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, OFFSET);
// ve OdomSensors'ta 3. satıra &horizontal

lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12.25, // track width (inç)
                              lemlib::Omniwheel::NEW_325, // 3.25" omni
                              450, // drivetrain rpm
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// düz gitme PID'i (moveToPoint)
lemlib::ControllerSettings linearController(8, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            40, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// dönüş PID'i (turnToHeading, moveToPoint'te yön düzeltme)
// ayar: kP'yi salınım başlayana kadar artır, sonra kD ile salınımı söndür
lemlib::ControllerSettings angularController(3, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             18, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2
                            nullptr, // horizontal tracking wheel - henüz yok
                            nullptr, // horizontal tracking wheel 2
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

// otonom dönüşlerinde hız sınırı - tam hızda robot savruluyor ve konumu kayıyor
const int TURN_MAX_SPEED = 70;

// ===========================================================================
// DR4B
// ===========================================================================
// 2 kırmızı motor aynı mile karşılıklı bağlı, 12T -> 48T (4:1)
// pozitif güç = yukarı. ters gidiyorsa iki portun işaretini değiştir: {11, -12}
pros::MotorGroup liftMotors({-11, 12}, pros::MotorGearset::red);
// rotation sensör DR4B kolunun milinde (48T) -> kolun gerçek açısını ölçer
// kaldırınca "Lift" açısı azalıyorsa portu negatif yap: pros::Rotation liftRotation(-7);
pros::Rotation liftRotation(7);

// DR4B en aşağıdayken ekranda yazan "ham" açı (derece)
// TODO: DR4B'yi en aşağı indir, ekrandaki ham değeri buraya yaz
const double LIFT_ZERO = 44;

// L1 ile inerken bu açının altında güç kesilir ve motorlar serbest kalır (COAST) - en alttaki durma noktasına karşı itmesin
const double LIFT_REST = -2;
// sürücü aşağı indirme hızı (0-127)
const int LIFT_DOWN_SPEED = 80;
// bu açının altında aşağı inerken hız LIFT_SLOW_SPEED ile sınırlanır - yere çarpmasın
const double LIFT_SLOW_ZONE = 20;
const int LIFT_SLOW_SPEED = 40;

// hazır yüksekliğe giderken hedefe bu kadar kala durur - gidişin hızıyla hedefi geçmesin
const double LIFT_STOP_MARGIN = 0;
// hazır yüksekliğe bu sürede varamazsa vazgeçer - takılırsa motorlar tam güçle itmeye devam etmesin
const uint32_t LIFT_MOVE_TIMEOUT = 2000;

// lift task'ının kullandığı değişkenler
double liftTarget = 0; // hazır yükseklik hedefi (derece)
int liftManual = 0; // sürücü gücü - 0 değilse elle kontrol
bool liftMoving = false; // hazır yüksekliğe gidiyor mu
bool liftRaising = false; // hazır yüksekliğe yukarı mı gidiyor
bool liftStopped = false; // durduruldu mu - fren komutu sadece bir kez gitsin
uint32_t liftMoveStart = 0; // hazır yüksekliğe gitmenin başladığı an

// sensör takılı ve okunuyor mu
bool liftSensorOk() { return liftRotation.get_angle() != PROS_ERR; }

// DR4B kolunun açısı (derece), en aşağı = 0, yukarı = pozitif
double liftAngle() {
    double angle = liftRotation.get_angle() / 100.0 - LIFT_ZERO; // centidegree -> derece
    // 0/360 geçişinde atlamasın diye -180..180 aralığına al
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

// DR4B'yi hazır yüksekliğe gönderir - liftTask götürür, bu sırada sürüş devam eder
void liftMoveTo(double target) {
    liftTarget = target;
    liftRaising = target > liftAngle();
    liftMoveStart = pros::millis();
    liftMoving = true;
}

// aşağı inerken: alta yaklaşınca yavaşla, en altta yere karşı itme
int liftLimit(int power, double angle) {
    if (power < 0 && angle < LIFT_REST) return 0;
    if (power < 0 && angle < LIFT_SLOW_ZONE) return std::max(power, -LIFT_SLOW_SPEED);
    return power;
}

// DR4B'yi durdurur: LIFT_REST'in altında serbest bırakır, üstünde HOLD ile tutar
void liftStop() {
    if (liftStopped) return;
    bool coast = liftSensorOk() && liftAngle() < LIFT_REST;
    liftMotors.set_brake_mode_all(coast ? pros::E_MOTOR_BRAKE_COAST : pros::E_MOTOR_BRAKE_HOLD);
    liftMotors.brake();
    liftStopped = true;
}

// DR4B'ye güç verir
void liftMove(int power) {
    if (power == 0) {
        liftStop();
    } else {
        liftMotors.move(power);
        liftStopped = false;
    }
}

// DR4B'yi her 10 ms'de bir kontrol eden arka plan görevi
void liftTask() {
    while (true) {
        if (!liftSensorOk()) {
            // sensör yoksa sadece elle kontrol
            liftMoving = false;
            liftMove(liftManual);
        } else if (liftManual != 0) {
            // sürücü hareket ettiriyor - hazır yükseklik iptal
            double angle = liftAngle();
            liftMoving = false;
            liftTarget = angle;
            liftMove(liftLimit(liftManual, angle));
        } else if (liftMoving) {
            // hazır yüksekliğe tam güçle git, varınca dur
            double angle = liftAngle();
            bool reached = liftRaising ? angle >= liftTarget - LIFT_STOP_MARGIN : angle <= liftTarget + LIFT_STOP_MARGIN;
            if (reached || pros::millis() - liftMoveStart >= LIFT_MOVE_TIMEOUT) {
                liftMoving = false;
                liftMove(0);
            } else {
                liftMove(liftLimit(liftRaising ? 127 : -LIFT_DOWN_SPEED, angle));
            }
        } else {
            liftMove(0);
        }
        pros::delay(10);
    }
}

// ===========================================================================
// CLAW
// ===========================================================================
// TODO: intake takılınca portunu yaz - port 8 artık şasi motoru
// pros::Motor intake(PORT);

// claw'u açıp kapatan pnömatik (true = açık, false = kapalı)
pros::adi::DigitalOut clawPiston('H');

bool clawClosed = false;

// claw'u açar / kapatır - ters çalışırsa buradaki ! işaretini kaldır
void setClaw(bool closed) {
    clawClosed = closed;
    clawPiston.set_value(!closed);
}

// ===========================================================================
// PID AYAR TESTLERİ
// ===========================================================================
// true iken sürüşte test tuşları çalışır (kumanda listesi opcontrol'ün üstünde) - ayar bitince false yap
const bool TUNING_MODE = true;

// DR4B test hedefleri (derece) - yukarı ok yükseğe, aşağı ok alçağa götürür
const double LIFT_TEST_HIGH = 45;
const double LIFT_TEST_LOW = 10;

// dönüş PID testi: her basışta sırayla 45, 90 ve 180 derece saat yönünde döner, sonucu yazar
const double TURN_TEST_ANGLES[] = {45, 90, 180};
int turnTestIndex = 0;

void turnTest() {
    double angle = TURN_TEST_ANGLES[turnTestIndex];
    turnTestIndex = (turnTestIndex + 1) % 3; // sonraki basışta diğer açı

    chassis.setPose(0, 0, 0);
    uint32_t start = pros::millis();
    chassis.turnToHeading(angle, 3000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = TURN_MAX_SPEED});
    // dönüş sürerken hedefi en fazla ne kadar geçtiğini izle
    double maxTheta = 0;
    while (chassis.isInMotion()) {
        maxTheta = std::max(maxTheta, (double)chassis.getPose().theta);
        pros::delay(10);
    }
    int time = pros::millis() - start;
    pros::delay(500); // robot tamamen dursun
    double finalError = angle - chassis.getPose().theta; // durduğu yerdeki hata
    double overshoot = std::max(0.0, maxTheta - angle);
    pros::lcd::print(6, "Donus %.0f: gecme %.1f, son %.2f, %d ms", angle, overshoot, finalError, time);
    printf(">>> DONUS %.0f deg: hedefi gecme %.1f deg, durdugu yerde hata %.2f deg, %d ms\n", angle, overshoot, finalError,
           time);
    fflush(stdout);
}

// düz gitme PID testi: her basışta sırayla 12", 24" ve 48" ileri gider, hatayı ve süreyi yazar
const double DRIVE_TEST_DISTANCES[] = {12, 24, 48};
// bu mesafeden uzun hareketlerde hız sınırı - tam hızdan fren robotu sarsıyor
const double DRIVE_LONG_DISTANCE = 30;
const float DRIVE_LONG_SPEED = 80;
int driveTestIndex = 0;

void driveTest() {
    double distance = DRIVE_TEST_DISTANCES[driveTestIndex];
    driveTestIndex = (driveTestIndex + 1) % 3; // sonraki basışta diğer mesafe

    chassis.setPose(0, 0, 0);
    uint32_t start = pros::millis();
    float maxSpeed = distance > DRIVE_LONG_DISTANCE ? DRIVE_LONG_SPEED : 127;
    chassis.moveToPoint(0, distance, 4000, {.maxSpeed = maxSpeed});
    // hareket sürerken hedefi en fazla ne kadar geçtiğini izle
    double maxY = 0;
    while (chassis.isInMotion()) {
        maxY = std::max(maxY, (double)chassis.getPose().y);
        pros::delay(10);
    }
    int time = pros::millis() - start;
    double exitError = distance - chassis.getPose().y; // hareket bittiği andaki hata
    pros::delay(500); // robot tamamen dursun
    double finalError = distance - chassis.getPose().y; // durduğu yerdeki hata
    double overshoot = std::max(0.0, maxY - distance);
    double heading = chassis.getPose().theta;
    pros::lcd::print(7, "Duz %.0f: gecme %.2f, son %.2f in, %d ms", distance, overshoot, finalError, time);
    printf(">>> DUZ %.0f in (hiz %.0f): hedefi gecme %.2f in, bitiste hata %.2f, durdugu yerde hata %.2f in, %d ms, yon sapmasi %.1f deg\n",
           distance, maxSpeed, overshoot, exitError, finalError, time, heading);
    fflush(stdout);
}

// kare testi: 24" kenarlı kare çizip başladığı yere döner - son konumu metreyle de ölç
void squareTest() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 24, 2000);
    chassis.turnToHeading(90, 1500, {.maxSpeed = TURN_MAX_SPEED});
    chassis.moveToPoint(24, 24, 2000);
    chassis.turnToHeading(180, 1500, {.maxSpeed = TURN_MAX_SPEED});
    chassis.moveToPoint(24, 0, 2000);
    chassis.turnToHeading(270, 1500, {.maxSpeed = TURN_MAX_SPEED});
    chassis.moveToPoint(0, 0, 2000);
    chassis.turnToHeading(360, 1500, {.maxSpeed = TURN_MAX_SPEED});
    chassis.waitUntilDone();
    pros::delay(500); // robot tamamen dursun

    lemlib::Pose pose = chassis.getPose();
    double headingError = std::remainder(pose.theta, 360.0);
    pros::lcd::print(5, "Kare: X %.2f Y %.2f yon %.1f", pose.x, pose.y, headingError);
    printf(">>> KARE: X %.2f in, Y %.2f in, yon hatasi %.1f deg\n", pose.x, pose.y, headingError);
    fflush(stdout);
}

// DR4B testi: hazır yüksekliğe gider, durduktan sonra süreyi ve son hatayı yazar
void liftTest(double target) {
    if (!liftSensorOk()) {
        printf(">>> LIFT: rotation sensoru yok\n");
        fflush(stdout);
        return;
    }
    liftManual = 0;
    double startAngle = liftAngle();
    uint32_t start = pros::millis();

    liftMoveTo(target); // liftTask hedefe götürür
    while (liftMoving) pros::delay(10);
    int time = pros::millis() - start;
    pros::delay(500); // DR4B tamamen dursun

    double error = target - liftAngle();
    pros::lcd::print(5, "Lift %.0f->%.0f: %d ms, hata %.1f", startAngle, target, time, error);
    printf(">>> LIFT %.0f -> %.0f: %d ms, son hata %.2f deg\n", startAngle, target, time, error);
    fflush(stdout);
}

// ===========================================================================
// PROGRAM
// ===========================================================================
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate(); // IMU kalibrasyonu (~2 sn) - bu sırada robota dokunma

    liftTarget = liftAngle(); // açılışta olduğu yerde kalsın
    pros::Task lift(liftTask);

    setClaw(true); // claw kapalı başlar

    // konum ve sensör değerlerini ekrana ve terminale yaz
    pros::Task screenTask([] {
        int count = 0;
        while (true) {
            lemlib::Pose pose = chassis.getPose();
            double vDist = vertical.getDistanceTraveled();
            std::vector<double> temps = liftMotors.get_temperature_all(); // °C
            std::vector<std::int32_t> currents = liftMotors.get_current_draw_all(); // mA

            pros::lcd::print(0, "X: %.2f  Y: %.2f  Theta: %.2f", pose.x, pose.y, pose.theta);
            pros::lcd::print(1, "Dikey: %.2f in", vDist);
            if (liftSensorOk()) {
                pros::lcd::print(2, "Lift: %.1f  hedef: %.1f  ham: %.1f", liftAngle(), liftTarget,
                                 liftRotation.get_angle() / 100.0);
            } else {
                pros::lcd::print(2, "Lift: rotation sensoru yok (port 7)");
            }
            pros::lcd::print(3, "Lift sicaklik: %.0f C  %.0f C", temps[0], temps[1]);
            pros::lcd::print(4, "Lift akim: %d mA  %d mA", currents[0], currents[1]);

            // terminal 250 ms'de bir - kablosuzda fazla veri kayboluyor
            if (count++ % 5 == 0) {
                printf("X: %7.2f  Y: %7.2f  Theta: %7.2f  |  Dikey: %7.2f  |  Lift: %6.1f  hedef: %6.1f\n", pose.x,
                       pose.y, pose.theta, vDist, liftAngle(), liftTarget);
                // lift motorları: 11W motor en fazla ~2500 mA çeker, 55°C'de gücü düşmeye başlar
                printf("    Lift motor 11: %2.0f C %5d mA  |  motor 12: %2.0f C %5d mA\n", temps[0], currents[0],
                       temps[1], currents[1]);
                fflush(stdout);
            }
            pros::delay(50);
        }
    });
}

void disabled() {}

void competition_initialize() {}

// otonom adımından sonra konumu terminale yazar
void logPose(const char* step) {
    lemlib::Pose pose = chassis.getPose();
    printf(">>> OTONOM %s: X %.2f, Y %.2f, Theta %.2f\n", step, pose.x, pose.y, pose.theta);
    fflush(stdout);
}

// DR4B hazır yüksekliğe varana kadar bekler (takılırsa LIFT_MOVE_TIMEOUT sonunda devam eder)
void waitLift() {
    while (liftMoving) pros::delay(10);
}

// cup'a yaklaşma hızı - LemLib hedefe 7.5" kala hız sınırını en az 60 yapıyor, 60'ın altı orada hızlanır
// 60 ile hızlanma olmaz, PID son 7.5"'te robotu kendiliğinden yavaşlatır
const float CUP_APPROACH_SPEED = 60;

// claw'un robot merkezinden öndeki mesafesi (inç) - 7.5'ten büyük olmalı, yoksa LemLib son yaklaşmada hızlanır
const float CLAW_OFFSET = 11;
// claw cup'a gelmeden bu kadar önce yavaşlamaya başlar (inç)
const float CUP_SLOW_DISTANCE = 15;
// cup'a son yaklaşma hızı
const float CUP_CREEP_SPEED = 25;

// (x, y) noktasına sabit hızla gider ve orada durur - LemLib'in 7.5" hızlanmasına girmez
void moveConstant(float x, float y, float speed, int timeout, bool forwards = true) {
    chassis.waitUntilDone(); // önceki hareket bitsin, başlangıç noktası doğru olsun

    lemlib::Pose pose = chassis.getPose();
    float dx = x - pose.x;
    float dy = y - pose.y;
    float distance = std::hypot(dx, dy);
    if (distance < 0.1) return;

    // hedefi 8" uzat, tam (x, y)'de çık: robot 7.5" bölgesine hiç girmez
    const float EXTRA = 8;
    chassis.moveToPoint(x + dx / distance * EXTRA, y + dy / distance * EXTRA, timeout,
                        {.forwards = forwards, .maxSpeed = speed, .minSpeed = speed, .earlyExitRange = EXTRA});
    chassis.waitUntilDone();
}

void autonomous() {
    uint32_t autonStart = pros::millis();
     chassis.setPose(0, 0, 0);
     setClaw(true);

  

    liftMoveTo(45);
    waitLift();
    liftMoveTo(20);
    chassis.moveToPoint(0, 5, 2000);
    chassis.moveToPoint(0, 0, 2000);
      chassis.waitUntilDone();
    liftMoveTo(45);
    waitLift();
        
    
     chassis.moveToPoint(0, 19, 2000);
    chassis.waitUntilDone();
    logPose("19 in sonrasi");
    chassis.turnToHeading(-90, 1500, {.maxSpeed = TURN_MAX_SPEED});
    chassis.waitUntilDone();
    logPose("donus sonrasi");
    chassis.moveToPoint(-19, 19, 1500);
    chassis.waitUntilDone();
    logPose("12 in sonrasi");
    
    
    
    
    // DR4B 5 dereceye iner, claw açılır
    
    
     liftMoveTo(15);
    waitLift();
    pros::delay(300);
    setClaw(false);

     // geri geri X = 0'a döner
    chassis.moveToPoint(-13, 19, 1500, {.forwards = false});
    chassis.waitUntilDone();
    logPose("geri donus sonrasi");

    // DR4B en alta iner
    liftMoveTo(0);
    
    chassis.turnToPoint(30, 38, 1500, {.forwards = false, .maxSpeed = TURN_MAX_SPEED});
    chassis.waitUntilDone();
    chassis.moveToPoint(30, 38, 1500 ,  {.forwards = false } );
        chassis.waitUntilDone();
    chassis.turnToPoint(57,16 , 1500, {.forwards = false , .maxSpeed = TURN_MAX_SPEED } );
        chassis.waitUntilDone();
    chassis.moveToPoint(57, 16, 1500 ,{.forwards = false});    
    chassis.waitUntilDone();
chassis.turnToHeading(180, 1500, {.maxSpeed = TURN_MAX_SPEED});
    chassis.waitUntilDone();
    chassis.moveToPoint(57, 4, 1500);
    chassis.waitUntilDone();
    logPose("57, 6 sonrasi");
    setClaw(true);
    pros::delay(300);
    liftMoveTo(3);
    waitLift(); // hedefe erken durd
    chassis.moveToPoint(57, 18, 1500, {.forwards = false});
    chassis.waitUntilDone();
    liftMoveTo(35);
   chassis.turnToHeading(270, 1500, {.maxSpeed = TURN_MAX_SPEED});
   chassis.waitUntilDone();
    chassis.moveToPoint(45, 18, 1500 );
    chassis.waitUntilDone();
    waitLift();
    chassis.moveToPoint(29, 18, 1500);
chassis.waitUntilDone();
    logPose("35, 19 sonrasi");
    pros::delay(300);
        liftMoveTo(20);
    waitLift();
    chassis.waitUntilDone();
    pros::delay(300);
    setClaw(false);
    pros::delay(300);
    chassis.moveToPoint(35, 18, 2000, {.forwards = false});
    liftMoveTo(0);
    chassis.moveToPoint(57, 18, 2000, {.forwards = false});
    chassis.waitUntilDone();
    waitLift();

    
    chassis.moveToPoint(57, 18, 1500 ,{.forwards = false});    
    chassis.waitUntilDone();
chassis.turnToHeading(180, 1500, {.maxSpeed = TURN_MAX_SPEED});
    chassis.waitUntilDone();
    chassis.moveToPoint(57, 4, 1500);
    chassis.waitUntilDone();
    logPose("57, 6 sonrasi");
    setClaw(true);
    pros::delay(300);
    liftMoveTo(3);
    waitLift(); // hedefe erken durd
    chassis.moveToPoint(57, 18, 1500, {.forwards = false});
    chassis.waitUntilDone();
    liftMoveTo(40);
   chassis.turnToHeading(270, 1500, {.maxSpeed = TURN_MAX_SPEED});
   chassis.waitUntilDone();
    chassis.moveToPoint(45, 18, 1500 );
    chassis.waitUntilDone();
    waitLift();
    chassis.moveToPoint(29, 18, 1500);
chassis.waitUntilDone();
    logPose("35, 19 sonrasi");
    pros::delay(300);
        liftMoveTo(25);
    waitLift();
    chassis.waitUntilDone();
    pros::delay(300);
    setClaw(false);
    pros::delay(300);
    chassis.moveToPoint(35, 18, 2000, {.forwards = false});
    liftMoveTo(0);
    chassis.moveToPoint(57, 18, 2000, {.forwards = false});
    chassis.waitUntilDone();
    waitLift();


    chassis.moveToPoint(57, 18, 1500 ,{.forwards = false});    
    chassis.waitUntilDone();
chassis.turnToHeading(180, 1500, {.maxSpeed = TURN_MAX_SPEED});
    chassis.waitUntilDone();
    chassis.moveToPoint(57, 4, 1500);
    chassis.waitUntilDone();
    logPose("57, 6 sonrasi");
    setClaw(true);
    pros::delay(300);
    liftMoveTo(3);
    waitLift(); // hedefe erken durd
    chassis.moveToPoint(57, 18, 1500, {.forwards = false});
    chassis.waitUntilDone();
    liftMoveTo(60);
   chassis.turnToHeading(270, 1500, {.maxSpeed = TURN_MAX_SPEED});
   chassis.waitUntilDone();
    chassis.moveToPoint(45, 18, 1500 );
    chassis.waitUntilDone();
    waitLift();
    chassis.moveToPoint(29, 18, 1500);
chassis.waitUntilDone();
    logPose("35, 19 sonrasi");
    pros::delay(300);
        liftMoveTo(30);
    waitLift();
    chassis.waitUntilDone();
    pros::delay(300);
    setClaw(false);
    pros::delay(300);
    chassis.moveToPoint(35, 18, 2000, {.forwards = false});
    liftMoveTo(0);
    chassis.moveToPoint(57, 18, 2000, {.forwards = false});
    chassis.waitUntilDone();
    waitLift();

    

    
 
}

// Kumanda:
//   sağ joystick Y: ileri-geri, sol joystick X: dönüş (arcade)
//   R1 / L1: DR4B yukarı / aşağı - bırakınca olduğu yerde durur
//   R2: claw aç / kapat
//   sadece TUNING_MODE'da:
//     yukarı ok / aşağı ok: DR4B'yi LIFT_TEST_HIGH / LIFT_TEST_LOW'a götürme testi
//     sol ok: konumu (0, 0, 0) yap - odometri testleri için
void opcontrol() {
    while (true) {
        // arcade sürüş
        int forward = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        chassis.arcade(forward, turn);

        // DR4B - asıl kontrol liftTask'ta
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) liftManual = 127;
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) liftManual = -LIFT_DOWN_SPEED;
        else liftManual = 0;

        // claw
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) setClaw(!clawClosed);

        // PID testleri (test bitene kadar sürüş durur)
        if (TUNING_MODE && controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) liftTest(LIFT_TEST_HIGH);
        if (TUNING_MODE && controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) liftTest(LIFT_TEST_LOW);
        if (TUNING_MODE && controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) chassis.setPose(0, 0, 0);

        pros::delay(10);
    }
}
