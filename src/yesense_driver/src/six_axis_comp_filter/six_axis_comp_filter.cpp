#include <math.h>
#include "six_axis_comp_filter.h"
#include <iostream>

#define PI                              3.1415926f
#define HALF_PI                         1.5707963f
#define TWO_PI                          6.2831853f
#define SQRE(x) 		                ((x)*(x))

        //
        // Constructor
        // Description:
        //      Initializes the complementary filter.
        // Parameters:
        // deltaTime - The time delta update period expressed in seconds. This value
        //      should be set to how often the filter is updated with new values,
        //      which should be on a regular interval. The smaller this value is,
        //      the less noise there will be on the comp. filter's output.
        // tau - Max allowable time until gyro drifts too far and comp. filter
        //      shifts its weight to the accelerometer expressed in seconds. This
        //      value is usually based on the drift rate of the gyro. For example,
        //      if the gyro drifts at a rate of 0.5 degrees per second and the
        //      tolerance of max allowable drift is 1 degree, then tau should be set
        //      to 2 seconds as it will take 2 seconds to drift 1 degree. The larger
        //      this value is, the less noise there will be on the comp. filter's
        //      output. In exchange, it will drift further from the correct angular
        //      position.
        // Returns:
        //      None.
        //
CompSixAxis::CompSixAxis(float deltaTime, float tau){
    //deltaTime 保存到类中
    deltaT = deltaTime;
    // 计算alpha
    alpha = tau/(tau + deltaT);


    // 参数初始化
    compAngleX = 0;
    compAngleY = 0;
    compAngleZ = 0;

    accelAngleX = 0;
    accelAngleY = 0;
    accelAngleZ = 0;
    Ax = 0;
    Ay = 0;
    Az = 0;
    //角速度 by gyro 
    Gx = 0;
    Gy = 0;
    Gz = 0;
}
        
        // Complementary Filter Start
        // Description:
        //      Should be called once before CompUpdate can be called at the next
        //      interval. CompAccelUpdate must be called before this function.
        //      This function helps the filter to converge faster. If this function
        //      is not called, the filter will still converge, but it will take
        //      longer initially.
        // Parameters:
        //      None.
        // Returns:
        //      None.
        
void CompSixAxis::CompStart(){

    // Calculate accelerometer angles
    CompAccelCalculate();
    

    //初始化最初的compAngle就是从accelAngle得到
    // Initialize filter to accel angles
    compAngleX = accelAngleX;
    compAngleY = accelAngleY;
    //compAngleZ = accelAngleZ;
}

void CompSixAxis::CompUpdate(){
    // Calculate the accelerometer angles
    CompAccelCalculate();
    //-Gy
    // Omega is the rotational velocity reported by the gyroscope. Though it seems
    // strange, the rotational velocity about the Y axis must be projected back
    // onto the X axis and then its sense of direction must be inverted in order to
    // acquire positive angles about the X axis. This is shown below with -Gy being
    // passed as a parameter.
    compAngleX = CompFilterProcess(compAngleX, accelAngleX, -Gy);

    // In this case, the rotational velocity about the X axis (Gx) is projected back
    // onto the Y axis and its sense of direction is already correct.
    compAngleY = CompFilterProcess(compAngleY, accelAngleY, Gx);
    //?
    //compAngleZ = CompFilterProcess(compAngleY, accelAngleY, Gx);

}

void CompSixAxis::CompAnglesGet(float *XAngle, float *YAngle){

    //如果不需要某个angle，使用这个函数的时候就传入0， 这里就判断下地址是否为0
    // Transfer class's updated comp. filter's angles
    // Check if valid addresses were passed as well.
    if (XAngle){
        *XAngle = compAngleX * RAD_TO_DEG_RATIO;
    }

    if(YAngle){
        *YAngle = compAngleY * RAD_TO_DEG_RATIO;
    }


    // if(ZAngle){
    //     *ZAngle = compAngleZ;
    // }
}

void CompSixAxis::CompAccelUpdate(float accelX, float accelY, float accelZ){
    // Save values to class
    Ax = accelX;
    Ay = accelY;
    Az = accelZ;
}

void CompSixAxis::CompGyroUpdate(float gyroX, float gyroY, float gyroZ){
    // Save values to class
    Gx = gyroX;
    Gy = gyroY;
    Gz = gyroZ;
}

// 根据加速度算欧拉角
// Calculates the angles according to the accelerometer based on the acceleration
// readings
//
void CompSixAxis::CompAccelCalculate(){
    accelAngleX = atan2f(Ax, sqrtf(SQRE(Ay)  + SQRE(Az)));

    accelAngleY = atan2f(Ay, sqrtf(SQRE(Ax) + SQRE(Az)));

    accelAngleX = FormatAccelRange(accelAngleX, Az);
    accelAngleY = FormatAccelRange(accelAngleY, Az); 
    //弧度
    std::cout << "accelAngleX :" << accelAngleX * RAD_TO_DEG_RATIO<< std::endl;
    std::cout << "accelAngleY :" << accelAngleY * RAD_TO_DEG_RATIO << std::endl;
}
// 检查加速度计算的方位角是否在0-2Pi
// Check to see which quadrant of the unit circle the angle lies in
// and format the angle to lie in the range of 0 to 2*PI
//
float CompSixAxis::
FormatAccelRange(float accelAngle, float accelZ){
    if(accelZ < 0.0f){
        accelAngle =  PI - accelAngle;
        
    }else if (accelZ > 0.0f && accelAngle < 0.0f){
        accelAngle = TWO_PI + accelAngle;
    }

    return accelAngle;
}

// 0-2Pi
// Formats the complimentary filter angle for faster convergence of the filter.
//
float CompSixAxis::
FormatFastConverge(float compAngle, float accAngle){
    if(compAngle > accAngle + PI){
        compAngle = compAngle - TWO_PI;
    }else if(accAngle > compAngle  + PI){
        compAngle = compAngle + TWO_PI;
    }

    return compAngle;   
}
//
// Formats the complimentary filter angle to always lie within the range of
// 0 to 2*pi
//

float CompSixAxis::FormatRange0to2PI(float compAngle){
    while(compAngle >= TWO_PI){
        compAngle = compAngle - TWO_PI;
    }

    while (compAngle < 0.0f){
        compAngle = compAngle + TWO_PI;
    }

    return compAngle;
}

// 互补
// Complimentary Filter - This is where the magic happens.
//
float CompSixAxis::CompFilterProcess(float compAngle, float accelAngle, float omega){
    float gyroAngle;
    //format
    //speed up filter convergence
    compAngle =  FormatFastConverge(compAngle, accelAngle);

    //积分
    // Integrate the gyroscope's angular velocity reading to get an angle
    gyroAngle = compAngle + omega * deltaT;
    std::cout << "gyroAngle: " <<  gyroAngle * RAD_TO_DEG_RATIO<< std::endl;

    //互补滤波
    // Weighting is applied to the gyroscope's angular position and
    // accelerometer's angular position and they are put together to form one
    // angle, the complimentary filter angle.
    compAngle = alpha * gyroAngle + (1.0f -  alpha) * accelAngle;

    // 角度调整一致
    // Format the Comp. Angle to lie in the range of 0 to 2*pi
    compAngle = FormatRange0to2PI(compAngle);
    std::cout << "compAngle: " <<  compAngle * RAD_TO_DEG_RATIO<< std::endl;

    return compAngle;
}
