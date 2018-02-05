#include <STC15W4K48S4.h>	//STC15W4K48S4 专用头文件
#include <math.H>
#include "IMU.H"
#define	pi		3.14159265f                           
#define	Kp		2.0f                        
#define	Ki		0.001f  
#define Kd      0.001f
#define	halfT	0.005f           

float idata q0=1,q1=0,q2=0,q3=0;   
float idata exInt=0,eyInt=0,ezInt=0;
float idata exDif=0,eyDif=0,ezDif=0;
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
	float idata norm;
	float idata vx, vy, vz;
	float idata ex, ey, ez;
	static float idata last_ex,last_ey,last_ez;
/*	norm = sqrt(ax*ax + ay*ay + az*az);	//把加速度计的三维向量转成单维向量   
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;*/
	norm=invSqrt(ax*ax + ay*ay + az*az) ;
	ax = ax *norm;
	ay = ay *norm;
	az = az *norm;
		//	下面是把四元数换算成《方向余弦矩阵》中的第三列的三个元素。 
		//	根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素
		//	所以这里的vx vy vz，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的
		//	重力单位向量。
	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3 ;

	ex = (ay*vz - az*vy) ;
	ey = (az*vx - ax*vz) ;
	ez = (ax*vy - ay*vx) ;
	
	exInt = exInt + ex * Ki;
	eyInt = eyInt + ey * Ki;
	ezInt = ezInt + ez * Ki;

	
	exDif = ex-last_ex;
	eyDif = ey-last_ey;
	ezDif = ez-last_ez;
	
	last_ex = ex;
	last_ey = ey;
	last_ez = ez;
	
	gx = gx + Kp*ex + exInt + Kd*exDif;		
	gy = gy + Kp*ey + eyInt + Kd*eyDif;		
	gz = gz + Kp*ez + ezInt + Kd*ezDif;


	q0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;
	q1 = q1 + ( q0*gx + q2*gz - q3*gy) * halfT;
	q2 = q2 + ( q0*gy - q1*gz + q3*gx) * halfT;
	q3 = q3 + ( q0*gz + q1*gy - q2*gx) * halfT;

/*	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
*/
	norm=invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0*norm;
	q1 = q1*norm;
	q2 = q2*norm;
	q3 = q3*norm;
	AngleY = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1)* 572.957795f;
	AngleX = asin(2*(q0*q2 - q1*q3 )) * 572.957795f; 
	//AngleZ = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1)* 572.957795f;

}
/*******************************************************************************
快速计算1/Sqrt(x)
*******************************************************************************/
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
