#include <BasicLinearAlgebra.h>
//#include "../../../Arduino/libraries/BasicLinearAlgebra/BasicLinearAlgebra.h"
#include <algorithm>
#include <math.h>

static constexpr float PI = 3.141592653589;

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
// F : 6 x 6
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
    return angle;
}

static float courseTo(float dx, float dy) {
    float ans = std::atan2(dx,dy);
    return wrap360(ans * 180 / PI);
}

class KalmanFilter {
    float mB1 = 2.;
    float mB2 = 3.;
    float mGpsNoise = 2.;
    float mGyroNoise = 0.1;
    float mMotorForce = 0.1;
    float mMotorTorque = 50.;

    float mDt = 0;

    float mWidth = 0.8; // Width of boat (m)

    Matrix<6,1> mX; // State space matrix
    Matrix<6,6> mP; // Initial uncertainty matrix
    Matrix<6,6> mQ; // Process uncertainty matrix
    Matrix<6,2> mB; // Input translation matrix
    Matrix<6,6> mF; // Transition matrix (model)
    
    Matrix<2,1> mU; // Input matrix
    Matrix<6,6> mI; // Identity matrix

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
                         0., 0., mGpsNoise/sensors.distGPS, 0.,
                         0., 0., 0., mGyroNoise};

        Matrix<4,1> y = z - H*mX;
        Matrix<4,4> S = H*mP*~H + R;
        Matrix<6,4> K = mP*~H*Inverse(S);
        y(2) = wrap180(y(2)); // Wrap d_theta
        mX    = mX + K*y;
        mP    = (mI - K*H)*mP;
    }

    void updateNoGPS(float rz) {
        rz = -rz;
        Matrix<1,6> H = {0., 0., 0., 0., 0., 1.};
        Matrix<1,1> R = {mGyroNoise};
        Matrix<1,1> z = {rz};
        Matrix<1,1> y = z - H*mX;
        Matrix<1,1> S = H*mP*~H + R;
        Matrix<6,1> K = mP*~H*Inverse(S);
        mX = mX + K*y;
        mP = (mI - K*H)*mP;
    }

public:
    KalmanFilter(float mDt) : mDt(mDt) {
        // Init mX
        mX.Fill(0);

        // Init mP
        mP.Fill(0);
        mP(0,0) = 30.; // mX
        mP(1,1) = 30.; // y
        mP(2,2) = 5.; // dx
        mP(3,3) = 5.; // dy
        mP(4,4) = 180.; // theta
        mP(5,5) = 5.; // d_theta

        // Init mQ
        mQ.Fill(0);
        mQ(0,0) = 0.5;
        mQ(1,1) = 0.5;
        mQ(2,2) = 5.0;
        mQ(3,3) = 5.0;
        mQ(4,4) = 1.0;
        mQ(5,5) = 2.0;

        // Init mB
        mB = {0., 0.,
             0., 0.,
             1., 1.,
             1., 1.,
             0., 0.,
             mMotorTorque*0.5*mDt*mWidth/2, -mMotorTorque*0.5*mDt*mWidth/2};

        // Init f
        mF = {1., 0., mDt, 0., 0., 0.,
             0., 1., 0., mDt, 0., 0.,
             0., 0., std::max(0., 1.-mDt*mB1), 0., 0., 0.,
             0., 0., 0., std::max(0., 1.-mDt*mB1), 0., 0.,
             0., 0., 0., 0., 1., mDt,
             0., 0., 0., 0., 0., 0.};

        mI = {1., 0., 0., 0., 0., 0.,
             0., 1., 0., 0., 0., 0.,
             0., 0., 1., 0., 0., 0.,
             0., 0., 0., 1., 0., 0.,
             0., 0., 0., 0., 1., 0.,
             0., 0., 0., 0., 0., 1.};
    }

    void predict(float motor1, float motor2) {
        float theta_r = mX(4);
        float delta_theta = 1 - mDt*mB2;

        mF(5, 5) = delta_theta;

        mB(3, 0) = mMotorForce*mDt*sin(theta_r);
        mB(3, 1) = mMotorForce*mDt*sin(theta_r);
        mB(4, 0) = mMotorForce*mDt*cos(theta_r);
        mB(4, 1) = mMotorForce*mDt*cos(theta_r);

        mU(0) = motor1;
        mU(1) = motor2;

        // Predict step
        mX = mF*mX + mB*mU;
        mP = mF*mP*~mF + mQ;
    }

    void update(const sensor_data& sensors) {
        static float lastX = 0;
        static float lastY = 0;
        if (sensors.gpsX != lastX ||
            sensors.gpsY != lastY) {
            // New gps data
            float dx = lastX - gpsX;
            float dy = lastX - gpsX;
            sensors.distGPS = sqrt(pow(dx, 2.) + pow(dy, 2.));
            sensors.courseGPS = courseTo(dx, dy);

            updateGPS(sensors);
        } else {
            // Old gps data
            updateNoGPS(sensors.rz);
        }

        mX(4) = wrap360(mX(4));

        lastX = sensors.gpsX;
        lastY = sensors.gpsY;
    }

    void setHome() {
        x(0) = 0;
        y(0) = 0;
        P(0,0) = mGpsNoise;
        P(1,1) = mGpsNoise;
    }
};
