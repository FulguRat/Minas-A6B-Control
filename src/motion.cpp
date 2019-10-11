#include "motion.h"

// 单位 mm
#define CYLINDER_HEIGHT 485.0 // 476.0 + 9
#define MAX_CYLINDER_LENGTH 44.0 // 总行程的一半
#define FB_LENGTH 433.5 // 433.6 这两个长度要相等，否则下面方程需要改
#define LR_LENGTH 433.5 // 433.3 -> 433.5
#define PI 3.1415926535898
#define MAX_PITCH_ANGLE floor(atan(MAX_CYLINDER_LENGTH*2/FB_LENGTH)*180/PI)
#define MAX_ROLL_ANGLE floor(atan(MAX_CYLINDER_LENGTH*2/LR_LENGTH)*180/PI)
#define MAX_ITERATION 100
#define EPSILON 0.0001

using namespace std;

bool Motion::CalculateMotion(){

	// 反解原理：
	// 以电缸中位状态下上平台的中心(高的一半)作为上平台的静坐标原点，则三电缸上端点坐标为a1 a2 a3，
	// 下平台的中心为下平台的静坐标原点，则三电缸下端点坐标为b1 b2 b3，设电缸自然收缩状态下长度为CYLINDER_HEIGHT
	// 在某一状态T下的三电缸进动h1 h2 h3为上平台在T坐标系下的上端点坐标a1' a2' a3'分别与b1 b2 b3的距离减去CYLINDER_HEIGHT
	// 坐标系取右手系，x轴向前，y轴向左，z轴向上，即pitch是绕y轴的运动，roll是绕x轴的运动，旋转遵守右手螺旋定则。

	if (fabs(mUpdown) > MAX_CYLINDER_LENGTH) {
		cout << "上下位移超过限制" << endl;
		return false;
	}
	if (fabs(mPitch) > MAX_PITCH_ANGLE) {
		cout << "俯仰角度超过限制" << endl;
		return false;
	}
	if (fabs(mRoll) > MAX_ROLL_ANGLE) {
		cout << "横滚角度超过限制" << endl;
		return false;
	}

	// clock_t start = clock();

	// 弧度
	double pitchA = mPitch*PI / 180;
	double rollA = mRoll*PI / 180;

	// 三电缸上端点a1 a2(left) a3(right)的齐次坐标为：
	// 初始值a1(280,0,0) a2(-280,280,0) a3(-280,-280,0)
	Eigen::Vector4d a1(FB_LENGTH / 2, 0, 0, 1);
	Eigen::Vector4d a2(-FB_LENGTH / 2, LR_LENGTH / 2, 0, 1);
	Eigen::Vector4d a3(-FB_LENGTH / 2, -LR_LENGTH / 2, 0, 1);
	// 三电缸下端点b1 b2(left) b3(right)的坐标为:
	Eigen::Vector3d b1(FB_LENGTH / 2, 0, -(CYLINDER_HEIGHT + MAX_CYLINDER_LENGTH));
	Eigen::Vector3d b2(-FB_LENGTH / 2, LR_LENGTH / 2, -(CYLINDER_HEIGHT + MAX_CYLINDER_LENGTH));
	Eigen::Vector3d b3(-FB_LENGTH / 2, -LR_LENGTH / 2, -(CYLINDER_HEIGHT + MAX_CYLINDER_LENGTH));

	// 设定在此运动状态下的T
	Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
	// 绕x轴旋转roll(a)的矩阵为：
	// Rx = 1  0     0
	//      0  cosa  -sina
	//      0  sina  cosa
	// 绕y轴旋转pitch(b)的矩阵为：
	// Ry = cosb   0  sinb
	//      0      1  0
	//      -sinb  0  cosb
	// 绕z轴不旋转，则旋转矩阵为：
	// Ry*Rx = cosb   sina*sinb  sinb*cosa
	//         0      cosa       -sina
	//         -sinb  sina*cosb  cosa*cosb
	// 又在z轴有一个平移分量，则T形如：
	// T = cosb   sina*sinb  sinb*cosa  0
	//     0      cosa       -sina      0
	//     -sinb  sina*cosb  cosa*cosb  z
	//     0      0          0          1
	double sina = sin(rollA);
	double cosa = cos(rollA);
	double sinb = sin(pitchA);
	double cosb = cos(pitchA);
	T(0, 0) = cosb;
	T(0, 1) = sina*sinb;
	T(0, 2) = sinb*cosa;
	T(1, 1) = cosa;
	T(1, 2) = -sina;
	T(2, 0) = -sinb;
	T(2, 1) = sina*cosb;
	T(2, 2) = cosa*cosb;
	T(2, 3) = mUpdown;

	// 则状态T下的a1' a2' a3'坐标为：
	Eigen::Vector4d a1p = T*a1;
	Eigen::Vector4d a2p = T*a2;
	Eigen::Vector4d a3p = T*a3;

	double a1px = a1p.x();
	double a1py = a1p.y();
	double a1pz = a1p.z();
	double a2px = a2p.x();
	double a2py = a2p.y();
	double a2pz = a2p.z();
	double a3px = a3p.x();
	double a3py = a3p.y();
	double a3pz = a3p.z();

	//cout << "状态T下三坐标为：" << endl;
	//cout << "a1p: " << a1px << " " << a1py << " " << a1pz << endl;
	//cout << "a2p: " << a2px << " " << a2py << " " << a2pz << endl;
	//cout << "a3p: " << a3px << " " << a3py << " " << a3pz << endl;

	// 计算电缸需要的进动
	double h1 = sqrt(pow(a1px - b1.x(), 2) + pow(a1py - b1.y(), 2) + pow(a1pz - b1.z(), 2)) - CYLINDER_HEIGHT - MAX_CYLINDER_LENGTH;
	double h2 = sqrt(pow(a2px - b2.x(), 2) + pow(a2py - b2.y(), 2) + pow(a2pz - b2.z(), 2)) - CYLINDER_HEIGHT - MAX_CYLINDER_LENGTH;
	double h3 = sqrt(pow(a3px - b3.x(), 2) + pow(a3py - b3.y(), 2) + pow(a3pz - b3.z(), 2)) - CYLINDER_HEIGHT - MAX_CYLINDER_LENGTH;

	// cout << "电缸进动为：" << endl;
	// cout << "h1: " << h1 << endl;
	// cout << "h2: " << h2 << endl;
	// cout << "h3: " << h3 << endl;
	
	if (fabs(h1) > MAX_CYLINDER_LENGTH) {
		cout << "前缸高度超过限制" << endl;
		return false;
	}
	else if (fabs(h2) > MAX_CYLINDER_LENGTH) {
		cout << "左缸高度超过限制" << endl;
		return false;
	}
	else if(fabs(h3) > MAX_CYLINDER_LENGTH) {
		cout << "右缸高度超过限制" << endl;
		return false;
	}

	// 以上是反解过程，但是由于系统结构设计上的特点，三电缸不是始终完全保持垂直于下平台的状态，
	// 即上平台坐标系会存在一个额外的平移和旋转（但是永远不可能绕z轴旋转），所以需要进行运动学
	// 的正解来验证反解求得的电缸进动是否可以形成我们需要的角度和位移，如果偏差在可接受范围内则求解完成

	// 以下是正解过程，正解的条件是上平台的三个电缸端点的距离分别相等且不会变化（刚体）以及下平台端点坐标已知
	// 条件：
	// |a1b1| = 进动h1+缸长+中位值
	// |a2b2| = 进动h2+缸长+中位值
	// |a3b3| = 进动h3+缸长+中位值
	// |a1a2| = 定值 |a1a3| = 定值 |a2a3| = 定值
	// b1b2b3已知 且根据下平台铰链特点知ya1 = 0, ya2 = -xa2, ya3 = xa3
	// 所以可以列出方程组: 
	// 令 H1 = h1+C(电缸长度)+MAX_CYLINDER_LENGTH;  H2 = h2+C+MAX_CYLINDER_LENGTH; 
	// 令 H3 = h3+C+MAX_CYLINDER_LENGTH;  CT = CYLINDER_HEIGHT + MAX_CYLINDER_LENGTH;
	// (xa1 - Dfb/2)^2 + (za1 + CT)^2 - H1^2 = 0
	// (xa2 + Dfb/2)^2 + (xa2 + Dlr/2)^2 + (za2 + CT)^2 - H2^2 = 0
	// (xa3 + Dfb/2)^2 + (xa3 + Dlr/2)^2 + (za3 + CT)^2 - H3^2 = 0
	// (xa2 - xa1)^2 + xa2^2 + (za2 - za1)^2 - Dfb^2 - Dlr^2/4 = 0
	// (xa3 - xa1)^2 + xa3^2 + (za3 - za1)^2 - Dfb^2 - Dlr^2/4 = 0
	// (xa3 - xa2)^2 + (xa2 + xa3)^2 + (za2 - za3)^2 - Dlr^2 = 0
	// 六个方程 六个未知数 [xa1 za1 xa2 za2 xa3 za3] 
    // 使用牛顿迭代法求解（https://wenku.baidu.com/view/5a8472b8112de2bd960590c69ec3d5bbfc0ada54.html）
	// 计算上面联立的非线性方程组的Jacobi矩阵为F'(x)：
	// 2xa1 - Dfb  2za1 + 2CT  0             0           0             0
	// 0           0           4xa2+Dfb+Dlr  2za2 + 2CT  0             0
	// 0           0           0             0           4xa3+Dfb+Dlr  2za3 + 2CT
	// 2xa1-2xa2   2za1-2za3   4xa2-2xa1     2za2-2za1   0             0
	// 2xa1-2xa3   2za1-2za3   0             0           4xa3-2xa1     2za3-2za1
	// 0           0           4xa2          2za2-2za3   4xa3          2za3-2za2
	// 牛顿迭代法公式为x(k+1) = x(k) - F'(x(k))^-1 * F(x(k))   k=0,1,2...
	// x(0) = [Dfb/2, 0, -Dfb/2, 0, -Dfb/2, 0]
	
	// 声明一些变量
	Eigen::Matrix<double, 6, 1> Fx;
	Eigen::Matrix<double, 6, 6> dFx, dFxi;
	double error = 10;
	double CT = CYLINDER_HEIGHT + MAX_CYLINDER_LENGTH;
	double H1 = h1 + CT;
	double H2 = h2 + CT;
	double H3 = h3 + CT;
	// 初始化x(0)
	double xa1 = FB_LENGTH / 2;
	double za1 = 0;
	double xa2 = -FB_LENGTH / 2;
	double za2 = 0;
	double xa3 = -FB_LENGTH / 2;
	double za3 = 0;

	for (int iter = 0; iter < MAX_ITERATION; ++iter) {
		double xa1n, za1n, xa2n, za2n, xa3n, za3n;
		if (error < EPSILON * 6) break;
		
		Fx << pow((xa1 - FB_LENGTH / 2), 2) + pow((za1 + CT), 2) - pow(H1,2),
			  pow((xa2 + FB_LENGTH / 2), 2) + pow((xa2 + LR_LENGTH / 2), 2) + pow((za2 + CT), 2) - pow(H2,2),
			  pow((xa3 + FB_LENGTH / 2), 2) + pow((xa3 + LR_LENGTH / 2), 2) + pow((za3 + CT), 2) - pow(H3,2),
			  pow((xa1 - xa2), 2) + pow(xa2, 2) + pow((za2 - za1), 2) - pow(FB_LENGTH, 2) - pow(LR_LENGTH, 2) / 4,
			  pow((xa1 - xa3), 2) + pow(xa3, 2) + pow((za3 - za1), 2) - pow(FB_LENGTH, 2) - pow(LR_LENGTH, 2) / 4,
			  pow((xa3 - xa2), 2) + pow((xa2 + xa3), 2) + pow((za2 - za3), 2) - pow(LR_LENGTH, 2);

		dFx << 2 * xa1 - FB_LENGTH, 2 * (za1 + CT),  0,								  0,			   0,								0,
			   0,                   0,				 4 * xa2 + FB_LENGTH + LR_LENGTH, 2 * (za2 + CT),  0,								0,
			   0,					0,				 0,								  0,			   4 * xa3 + FB_LENGTH + LR_LENGTH, 2 * (za3 + CT),
			   2 * (xa1 - xa2),		2 * (za1 - za3), 2 * (2 * xa2 - xa1),			  2 * (za2 - za1), 0,								0,
			   2 * (xa1 - xa3),		2 * (za1 - za3), 0,								  0,               2 * (2 * xa3 - xa1),				2 * (za3 - za1),
			   0,					0,				 4 * xa2,						  2 * (za2 - za3), 4 * xa3,							2 * (za3 - za2);
		
		dFxi = dFx.inverse();
		
		xa1n = xa1 - dFxi.row(0)*Fx;
		za1n = za1 - dFxi.row(1)*Fx;
		xa2n = xa2 - dFxi.row(2)*Fx;
		za2n = za2 - dFxi.row(3)*Fx;
		xa3n = xa3 - dFxi.row(4)*Fx;
		za3n = za3 - dFxi.row(5)*Fx;

		error = fabs(xa1 - xa1n) + fabs(za1 - za1n) + fabs(xa2 - xa2n) + fabs(za2 - za2n) + fabs(xa3 - xa3n) + fabs(za3 - za3n);

		xa1 = xa1n;
		za1 = za1n;
		xa2 = xa2n;
		za2 = za2n;
		xa3 = xa3n;
		za3 = za3n; 
	}

	//cout << xa1 << " " << za1 << " " << xa2 << " " << za2 << " " << xa3 << " " << za3 << " " << error << endl;
	
	// 以上便唯一确定了上平台三个端点的坐标，然后通过三个坐标值唯一确定上平台的俯仰、横滚和纵向运动，
	// 再判断这个运动是不是我们想要的，是不是在可接受范围以内
	double roll_new = atan((za2 - za3) / LR_LENGTH) * 180 / PI;
	double pitch_new = atan(((za2 + za3) / 2 - za1) / FB_LENGTH) * 180 / PI;
	double updown_new = ((za2 + za3) / 2 + za1) / 2;

	// cout << "Pitch new: " << pitch_new << endl; 
	// cout << "Roll new: " << roll_new << endl;
	// cout << "Updown new: " << updown_new << endl;

	if(fabs(roll_new - mRoll)<0.1 && 
		fabs(pitch_new - mPitch)<0.1 && 
		fabs(updown_new - mUpdown)<0.1){
			this->mHFront = h1;
			this->mHLeftBack = h2;
			this->mHRightBack = h3;
	}
	else{
		cout<<"Erorr is Too Much..."<<endl;
		return false;
	}

	// clock_t end = clock();
	// double endtime = (double)(end - start);
	// cout << "Total time: " << endtime << " ms" << endl;	//ms为单位

	return true;
}


void Motion::GetMotion(double &f,double &l, double &r, double &time){
	f = mHFront;
	l = mHLeftBack;
	r = mHRightBack;
	time = mElapsed;
}