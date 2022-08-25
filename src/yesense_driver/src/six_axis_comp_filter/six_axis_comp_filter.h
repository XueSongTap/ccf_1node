#ifndef SIX_AXIS_COMP_FILTER_H
#define SIX_AXIS_COMP_FILTER_H


//https://github.com/tcleg/Six_Axis_Complementary_Filter/


//*********************************************************************************
// Headers
//*********************************************************************************
#include <stdint.h>

//*********************************************************************************
// Macros and Globals
//*********************************************************************************

//macro用于 角度弧度相互转换
#define DEG_TO_RAD_RATIO    0.0174532f
#define RAD_TO_DEG_RATIO    57.2957795f

//*********************************************************************************
// Class
//*********************************************************************************

//只有这一个类  
class CompSixAxis{
public:

        //构造函数，构造的时候传入deltaTime 和tau两个参数
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
    CompSixAxis(float deltaTime, float tau);

        //
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
        //

    void CompStart();

        //
        // Complementary Filter Update
        // Description:
        //      Must be called on a regular interval specified by deltaT.
        //      Be sure to call CompAccelUpdate and CompGyroUpdate with
        //      new values before calling this function.
        // Parameters:
        //      None.
        // Returns:
        //      None.
        //
    void CompUpdate();


        //
        // Complementary Filter Angles Get
        // Description:
        //      Acquires the angles in radians relative to ground along the
        //      positive X and Y axes.
        // Parameters:
        //      XAngle - Address of a float to store the angle relative to the X
        //          axis into. The number 0 can be passed as a parameter if this
        //          angle is not needed.
        //      YAngle - Address of a float to store the angle relative to the Y
        //          axis into. The number 0 can be passed as a parameter if this
        //          angle is not needed.
        // Returns:
        //      None.
        //
    //输出角度
    void CompAnglesGet(float *XAngle, float*YAngle);

        //
        // Complementary Filter Accelerometer Update
        // Description:
        //      Updates the comp. filter with new accelerometer values.
        // Parameters:
        //      accelX - Acceleration vector along X axis expressed in m/s^2
        //      accelY - Acceleration vector along Y axis expressed in m/s^2
        //      accelZ - Acceleration vector along Z axis expressed in m/s^2
        // Returns:
        //      None.
        //
    //传入加速度
    void CompAccelUpdate(float accelX, float accelY, float accelZ);

        //
        // Complementary Filter Gyroscope Update
        // Description:
        //      Updates the comp. filter with new gyroscope values.
        // Parameters:
        //      gyroX - Angular velocity around X axis expressed in rad/s
        //      gyroY - Angular velocity around Y axis expressed in rad/s
        //      gyroZ - Angular velocity around Z axis expressed in rad/s
        // Returns:
        //      None.
        //
    //传入角速度
    void CompGyroUpdate(float gyroX, float gyroY, float gyroZ);
        //
        // Complementary Filter Degrees to Radians
        // Description:
        //      Converts degrees to radians.
        // Parameters:
        //      degrees - Value in degrees.
        // Returns:
        //      A value in radians.
        //
    float CompDegreesToRadians(float degrees){
        return degrees * DEG_TO_RAD_RATIO;
    };
        //
        // Complementary Filter Radians to Degrees
        // Description:
        //      Converts radians to degrees.
        // Parameters:
        //      radians - Value in radians.
        // Returns:
        //      A value in degrees.
        //
    float CompRadiansToDegrees(float radians){
        return radians * RAD_TO_DEG_RATIO;
    }
private:
        //
        // The time delta between updates. deltaT = 1/(sampling frequency)
        //
    float deltaT;
        //
        // Weighting factor
        //
    float alpha;

        //
        // The most recent accelerometer readings.
        //
    float Ax, Ay, Az;

        //
        // The most recent accelerometer readings.
        //
    float Gx, Gy, Gz;
        //
        // Accelerometer angles in relation to the X and Y axes.
        //
    float accelAngleX, accelAngleY, accelAngleZ;

        //
        // Comp. filter angle output in relation to the X and Y axes.
        //
    float compAngleX, compAngleY, compAngleZ;

        //
        // Extrapolates angles according to accelerometer readings
        //
    void CompAccelCalculate();

        //
        // Check to see which quadrant of the unit circle the angle lies in
        // and format the angle to lie in the range of 0 to 2*PI
        //
    float FormatAccelRange(float accelAngle, float accelZ);
        //
        // Formats the Comp. Angle for faster filter convergence
        //
    float FormatFastConverge(float compAngle, float accAngle);
       //
        // Formats the complimentary filter angle to always lie within the range of
        // 0 to 2*pi
        //
    float FormatRange0to2PI(float compAngle);
        //
        // Complimentary Filter - This is where the magic happens.
        //
    float CompFilterProcess(float compAngle, float accelAngle, float omega);
    float CompFilterProcessZ(float compAngle, float omega);

};

#endif// SIX_AXIS_COMP_FILTER_H