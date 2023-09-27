#include <BasicLinearAlgebra.h>
//#include "../../../Arduino/libraries/BasicLinearAlgebra/BasicLinearAlgebra.h"
#include <algorithm>
#include <cmath>
#include <math.h>

using namespace BLA;

struct sensor_data {
    float gpsX;
    float gpsY;
    float courseGPS;
    float distGPS;
    float rz;
};

// Notes
// 6 states
// 2 inputs
// y : 2 x 1
// B : 6 x 2
// u : 2 x 1
// z : 2 x 1
// H : 2 x 6
// mF : 6 x 6
// x : 6 x 1
// S : 2 x 2
// P : 6 x 6
// R : 2 x 2
// K : 6 x 2
// Q : 6 x 6

static float wrap360(float angle) {
    if (angle < 0.) {
        angle += 360.;
    } else if (angle >= 360.) {
        angle -= 360.;
    }
    return angle;
}

static float wrap180(float angle) {
    if (angle > 180.)
        angle -= 360;
    else if (angle < -180.)
        angle += 360;
    return angle;
}

static float courseTo(float dx, float dy) {
    float ans = std::atan2(dx,dy);
    return wrap360(ans * 180. / PI);
}

class KalmanFilter {
    float b1 = 0.8;
    float b2 = 3.;
    float mGpsNoise = 4.;
    float mGpsAngleNoise = 10;
    float mGyroNoise = 0.1;
    float mMotorForce = 0.6;
    float mMotorTorque = 100;

    float dt;

    float mWidth = 0.8; // Width of boat (m)

    Matrix<6,1> x{}; // State space matrix
    Matrix<6,6> P{}; // Initial uncertainty matrix
    Matrix<6,6> Q{}; // Process uncertainty matrix
    Matrix<6,2> B{}; // Input translation matrix
    Matrix<6,6> mF{}; // Transition matrix (model)
    
    Matrix<2,1> u{}; // Input matrix
    Matrix<6,6> I{}; // Identity matrix

    void printState() {
        for (int i = 0; i < 6; i++) {
            Serial.print(x(i));
            Serial.print(" ");
        }
        Serial.println();
    }

    void updateGPS(const sensor_data& sensors) {
        Matrix<4,1> z = {sensors.gpsX,
                         sensors.gpsY,
                         sensors.courseGPS,
                         sensors.rz};
        Matrix<4,6> H = {1., 0., 0., 0., 0., 0.,
                         0., 1., 0., 0., 0., 0.,
                         0., 0., 0., 0., 1., 0.,
                         0., 0., 0., 0., 0., 1.};
        Matrix<4,4> R = {mGpsNoise, 0., 0., 0.,
                         0., mGpsNoise, 0., 0.,
                         0., 0., 1., 0.,
                         0., 0., 0., mGyroNoise};
        R(2, 2) = fmaxf(0., mGpsAngleNoise*(0.5 - sensors.distGPS));
        Matrix<4,1> y = z - H*x;
        Matrix<4,4> S = H*P*~H + R;
        Matrix<6,4> K = P*~H*Inverse(S);
        y(2) = wrap180(y(2)); // Wrap d_theta
        x    = x + K*y;
        P    = (I - K*H)*P;
    }

    void updateNoGPS(float rz) {
        if (!isfinite(rz))
            rz = 0;
        Matrix<1,6> H = {0., 0., 0., 0., 0., 1.};
        Matrix<1,1> R = {mGyroNoise};
        Matrix<1,1> z = {rz};
        Matrix<1,1> y = z - H*x;
        Matrix<1,1> S = H*P*~H + R;
        Matrix<6,1> K = P*~H*Inverse(S);
        x = x + K*y;
        P = (I - K*H)*P;
    }

public:
    KalmanFilter(float dt) : dt(dt) {
        // Init x
        x.Fill(0);

        // Init P
        P.Fill(0);
        P(0,0) = 30.; // x 
        P(1,1) = 30.; // y
        P(2,2) = 5.; // dx
        P(3,3) = 5.; // dy
        P(4,4) = 180.; // theta
        P(5,5) = 5.; // d_theta

        // Init Q
        Q.Fill(0);
        Q(0,0) = 0.5;
        Q(1,1) = 0.5;
        Q(2,2) = 5.0;
        Q(3,3) = 5.0;
        Q(4,4) = 0.5;
        Q(5,5) = 2.0;

        // Init B
        B = {0., 0.,
             0., 0.,
             1., 1.,
             1., 1.,
             0., 0.,
             mMotorTorque*dt, -mMotorTorque*dt};

        // Init f
        mF = {1., 0., dt, 0., 0., 0.,
              0., 1., 0., dt, 0., 0.,
              0., 0., (1.-dt*b1), 0., 0., 0.,
              0., 0., 0., (1.-dt*b1), 0., 0.,
              0., 0., 0., 0., 1., dt,
              0., 0., 0., 0., 0., (1. - dt*b2)};

        I = {1., 0., 0., 0., 0., 0.,
             0., 1., 0., 0., 0., 0.,
             0., 0., 1., 0., 0., 0.,
             0., 0., 0., 1., 0., 0.,
             0., 0., 0., 0., 1., 0.,
             0., 0., 0., 0., 0., 1.};
    }

    void predict(float motor1, float motor2) {
        float theta_r = x(4) * PI / 180;

        Serial.print("Theta_r: ");
        Serial.println(theta_r);
        B(2, 0) = mMotorForce*dt*sin(theta_r);
        B(2, 1) = mMotorForce*dt*sin(theta_r);
        B(3, 0) = mMotorForce*dt*cos(theta_r);
        B(3, 1) = mMotorForce*dt*cos(theta_r);
        Serial.println("Motor output: ");
        Serial.println(mMotorForce*dt*sin(theta_r));
        Serial.println(mMotorForce*dt*cos(theta_r));

        u(0) = motor1;
        u(1) = motor2;
        Serial.print("Motor input: ");
        Serial.print(motor1);
        Serial.print(" ");
        Serial.println(motor2);

        Serial.print("Process uncert: ");
        Serial.println(P(4, 4));
        
        Serial.print("x:");
        for (int i = 0; i < 6; i++) {
            Serial.print(x(i));
            Serial.print(" ");
        }
        Serial.println();

        Serial.print("B:\n");
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 2; j++) {
                Serial.print(B(i, j));
                Serial.print(" ");
            }
            Serial.println();
        }

        // Predict step
        x = mF*x + B*u;
        P = mF*P*~mF + Q;

        Serial.print("x:");
        for (int i = 0; i < 6; i++) {
            Serial.print(x(i));
            Serial.print(" ");
        }
        Serial.println();

    }

    void update(sensor_data& sensors) {
        static float lastX = 0.;
        static float lastY = 0.;
        sensors.rz = -sensors.rz;
        if (sensors.gpsX != lastX ||
            sensors.gpsY != lastY) {
            // New gps data
            float dx = sensors.gpsX - lastX;
            float dy = sensors.gpsX - lastY;
            sensors.distGPS = sqrt(pow(dx, 2.) + pow(dy, 2.));
            sensors.courseGPS = courseTo(dx, dy);

            if (!isfinite(sensors.distGPS) ||
                !isfinite(sensors.courseGPS) ||
                !isfinite(sensors.gpsX) ||
                !isfinite(sensors.gpsY))
                updateNoGPS(sensors.rz);
            else
                updateGPS(sensors);
        } else {
            // Old gps data
            updateNoGPS(sensors.rz);
        }

        x(4) = wrap360(x(4));

        lastX = sensors.gpsX;
        lastY = sensors.gpsY;

        printState();
    }

    void setHome() {
        x(0) = 0.;
        x(1) = 0.;
        x(4) = 0.; // Set to north
        P(0,0) = mGpsNoise;
        P(1,1) = mGpsNoise;
    }

    void copyTo(float array[6]) {
        for (int i = 0; i < 6; i++) {
            array[i] = x(i);
        }
    }
};
