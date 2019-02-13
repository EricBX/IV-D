/*
WARNING !

DO NOT MODIFY CODES BELOW!
*/

#ifdef _WIN32
#include <windows.h>
#include <cmath>
#include <string>  
#include <fstream>  
#include <iostream>  
using namespace std;
#endif

#include "driver_cruise.h"
#include "stdio.h"

#define PI 3.141592653589793238462643383279

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_cruise(tModInfo *modInfo)
{
	memset(modInfo, 0, 10 * sizeof(tModInfo));
	modInfo[0].name = "driver_cruise";	// name of the module (short).
	modInfo[0].desc = "user module for CyberCruise";	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId = 0;
	modInfo[0].index = 0;
	return 0;
}

// Module interface initialization.
static int InitFuncPt(int, void *pt)
{
	tUserItf *itf = (tUserItf *)pt;
	itf->userDriverGetParam = userDriverGetParam;
	itf->userDriverSetParam = userDriverSetParam;
	return 0;
}


/*
WARNING!

DO NOT MODIFY CODES ABOVE!
*/

//**********Global variables for vehicle states*********//
static float _midline[200][2];							//
static float _yaw, _yawrate, _speed, _acc, _width, _rpm;//
static int _gearbox;									//
														//******************************************************//


bool parameterSet = false;								//
void PIDParamSetter();									//


														//******************************************************//
typedef struct Circle									//
{														//
	double r;											//
	int sign;											//
}circle;												//
														//******************************************************//


//*************************道路识别***************************
struct kindOfRoad
{
	int sign = -1;//0-直道，1-左弯道，2-右弯道，-1未使用
	int ahead, behind;//该段道路的起始点和终点
	float dcur;//该段道的总角度	//这一段道路转过的角度	正：顺时针	负：逆时针
	float es;//目标速度
	float r;//弯道曲率半径
};
kindOfRoad road[15];
static float _everycur[200];				//路径点曲率角（角度制）			//正：逆时针	负：顺时针	eg:0.1 0.14 0.57
static float _everydcur[200];				//关于0-1向量转过的角度			//正：逆时针	负：顺时针
static float _everyslope[200];				//路径点斜率之角度（在车身坐标系中）（弧度制）	



											//********************PID parameters*************************//
double kp_s;	//kp for speed							     //
double ki_s;	//ki for speed							     //
double kd_s;	//kd for speed							     //
double kp_d;	//kp for direction						     //
double ki_d;	//ki for direction					    	 //
double kd_d;	//kd for direction						     //
				// Direction Control Variables						         //
double D_err;//direction error
double D_err1;//direction error//
double D_errDiff = 0;//direction difference(Differentiation) //
double D_errSum = 0;//sum of direction error(Integration)      //
					// Speed Control Variables								     //
circle c;												     //
double expectedSpeed;//      							     //
double curSpeedErr;//speed error   		                     //
double speedErrSum = 0;//sum of speed error(Integration)       //
int startPoint;											     //
int delta = 20;												 //
int _count = 0;//用以matlab输出
float cur;
float dcur;
//========横向控制专用============
bool steered = 0;//测试用	0=未设置steer	1=设置了steer
float  k1 = 1, k2 = 0, k3 = 0.1;//不同横向控制的权值		k1预瞄点	k2路偏距	k3偏航角
float X0, Y0;//预瞄点

//=====识别公路or土路=====
bool dirt = 0;//0-公路，1-土路
bool distinguish = 0;	//0：未识别	1：识别完成
int count_dirt = 0;



//*******************Other parameters*******************//
const int topGear = 6;									//
double tmp;												//
bool flag = true;											//
double offset = 0;										//
double Tmp = 0;
double Tmp1 = 0;
//******************************************************//

//******************************Helping Functions*******************************//
// Function updateGear:															//
//		Update Gear automatically according to the current speed.				//
//		Implemented as Schmitt trigger.											//
void updateGear(int *cmdGear);													//
double constrain(double lowerBoundary, double upperBoundary, double input);		// Function constrain:															
																				//	Given input, outputs value with boundaries.								
void everyCur(float _midline[200][2]);//获取everycur
void everyDcur(float _midline[200][2]);//获取everydcur
void everySlope(float _midline[200][2]);//获取everyslope
void getRoad(float _everycur[200], float _everydcur[200]);//获取路径类型
float getDistance(float x, float y);//获取与原点距离的工具函数
int sign(float a);//获取符号的工具函数
float getD_eer(float x, float y);//获取D_err//用于预瞄点steer控制
//******************************************************************************//

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm) {
	/* write your own code here */
	for (int i = 0; i< 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;
}

static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear) {
	everyCur(_midline);
	everyDcur(_midline);
	getRoad(_everycur, _everydcur);

	//道路识别
	//cout << "当前道路为：" << road[0].sign << " 前方结束：" << road[0].ahead << " 角度：" << road[0].dcur << endl;
	//cout << "下一道路为：" << road[1].sign << " 距离：" << road[1].behind << " 角度：" << road[1].dcur << endl;

	int aim = 0;
	cout << "当前道路为：";
	switch (road[aim].sign)
	{
	case 0:cout << "直道; "; break;
	case 1:cout << "左弯道; "; break;
	case 2:cout << "右弯道; "; break;
	default:cout << "未识别; "; break;
	}	
	cout<<" 起点：" << road[aim].behind << " 终点：" << road[aim].ahead << " 转过角度：" << road[aim].dcur;
	if (road[aim].sign == 1 || road[aim].sign == 2) { cout << "曲率半径：" << road[aim].r; }
	cout << "期望速度：" << road[aim].es << endl;

	if (parameterSet == false)		// Initialization Part
	{
		PIDParamSetter();
	}
	else
	{
		//识别公路or土路
		if (!distinguish&&_gearbox == 2) {
			if (count_dirt > 145) { dirt = 1; distinguish = 1; }
			else { dirt = 0; distinguish = 1; }
		}
		if (!distinguish&&count_dirt < 500)count_dirt++;
		
		//分土路和公路设计模型
		//土路模型：
		if (dirt) {
			//========================== Speed Control =================================	
				if (road[0].sign == 0) {

					float es = 350;
					if (road[1].sign != -1) es = sqrt(150 * road[1].behind + 0.85*0.85*road[1].es*road[1].es);
					if (road[2].sign != -1)es = min(es, sqrt(150 * road[2].behind + 0.85*0.85*road[2].es*road[2].es));
					if (road[3].sign != -1)es = min(es, sqrt(150 * road[3].behind + 0.85*0.85*road[3].es*road[3].es));

					es = min(es, road[0].es);
					if (abs(_everydcur[50]) > 75 && es > 130 * 0.85)es = 130 * 0.85;
					for (int i = 1; i < 50; i++) {
						if (_everycur[i] > 0.9&&es>sqrt(150 * i + 130 * 0.85*0.85 * 130)) {
							es = sqrt(130 * i + 150 * 0.85*0.85 * 130);
							break;
						}
					}
					expectedSpeed = constrain(60, 350, es);
				}
				if (road[0].sign == 1 || road[0].sign == 2) {

					float es = 350;
					if (road[1].sign != -1) es = sqrt(50 * road[1].behind + road[1].es*road[1].es);
					if (road[2].sign != -1)es = min(es, sqrt(50 * road[2].behind + road[2].es*road[2].es));
					if (road[3].sign != -1)es = min(es, sqrt(50 * road[3].behind + road[3].es*road[3].es));

					es = min(es, road[0].es);
					if (abs(_everydcur[50]) > 75 && es > 130 * 0.85)es = 130 * 0.85;
					for (int i = 1; i < 50; i++) {
						if (_everycur[i] > 0.9&&es>sqrt(50 * i + 130 * 0.85*0.85 * 130)) {
							es = sqrt(50 * i + 130 * 0.85*0.85 * 130);
							break;
						}
					}
					expectedSpeed = constrain(60, 350, es);
				}

			expectedSpeed = constrain(40, 350, expectedSpeed);
			//expectedSpeed *= 1.1;//速度整体提升
			curSpeedErr = expectedSpeed - _speed;
			speedErrSum = 0.1 * speedErrSum + curSpeedErr;

			//========================== Direction Control =================================		
			//set the param of PID controller
			kp_d = 4;
			ki_d = 0;
			kd_d = 0;

			//道路预测模型
			if (road[0].sign == 0)//车在直道		
			{
				if (road[1].sign == -1 || road[1].sign == 0)//长直道
				{

					X0 = _midline[road[0].ahead][0];
					Y0 = _midline[road[0].ahead][1];
					D_err = getD_eer(X0, Y0);
					float Steer_D_err = kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff;
					k1 = 1; k3 = 0.1;
					*cmdSteer = k1*Steer_D_err + k3*_yaw;
					steered = 1;
					cout << "直线行驶\t预瞄点 = （" << X0 << ',' << Y0 << ")" << endl;
				}
				else if (road[1].sign == 1 || road[1].sign == 2)//准备入弯
				{
					if ((road[1].sign + road[2].sign) == 3 && (road[1].ahead - road[1].behind) < 60)//S弯 
					{
						if (road[0].ahead > 30 && abs(road[1].dcur) > 20)//预入弯
						{
							if (dirt) {
								if (road[1].sign == 1)//左转
									X0 = _midline[road[0].ahead][0] + _width * 1 / 8;
								else if (road[1].sign == 2)//右转
									X0 = _midline[road[0].ahead][0] - _width * 1 / 8;

								Y0 = _midline[road[0].ahead][1];
								D_err = getD_eer(X0, Y0);
								float Steer_D_err = ((kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.3;
								k1 = 1; k3 = 0.1;
								*cmdSteer = k1*Steer_D_err + k3*_yaw;
								steered = 1;
								cout << "预入弯\t预瞄点 = （" << X0 << ',' << Y0 << ")" << endl;
							}
							else {
								if (road[1].sign == 1)//左转
									X0 = _midline[road[0].ahead][0] + _width * 1 / 8;
								else if (road[1].sign == 2)//右转
									X0 = _midline[road[0].ahead][0] - _width * 1 / 8;

								Y0 = _midline[road[0].ahead - 5][1];
								D_err = getD_eer(X0, Y0);
								float Steer_D_err = ((kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.1;
								k1 = 1; k3 = 0.1;
								*cmdSteer = k1*Steer_D_err + k3*_yaw;
								steered = 1;
								cout << "预入弯\t预瞄点 = （" << X0 << ',' << Y0 << ")" << endl;
							}
						}
						else {
								D_err = getD_eer(_midline[20][0], _midline[20][1]);
								D_err1 = getD_eer(_midline[40][0], _midline[40][1]);
								double distence = sqrt((_midline[25][0] - _midline[26][0])*(_midline[25][0] - _midline[26][0]) + (_midline[25][1] - _midline[26][1])*(_midline[25][1] - _midline[26][1]));
								*cmdSteer = constrain(-1.0, 1.0, ((kp_d * (D_err + 0.3*D_err1) + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.3);
								cout << "=====正在用简化模型=====" << endl;
						}
					}
					else //未知的大弯 或者 可以预知的小弯 或者 其他任何弯 或者 第一弯太大 也归到此类
					{
						if (road[0].ahead > 30 && abs(road[1].dcur)>20)//预入弯
						{
								if (road[1].sign == 1)//左转
									X0 = _midline[road[0].ahead][0] + _width * 3 / 8;
								else if (road[1].sign == 2)//右转
									X0 = _midline[road[0].ahead][0] - _width * 3 / 8;
								Y0 = _midline[road[0].ahead][1];
								D_err = getD_eer(X0, Y0);
								float Steer_D_err = ((kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.3;
								k1 = 1; k3 = 0.1;
								*cmdSteer = k1*Steer_D_err + k3*_yaw;
								steered = 1;
								cout << "预入弯\t预瞄点 = （" << X0 << ',' << Y0 << ")" << endl;
						}
						else//其他入弯综合处理☆
						{
								X0 = _midline[20][0];
								Y0 = _midline[20][1];
								D_err = getD_eer(X0, Y0);
								D_err1 = getD_eer(_midline[35][0], _midline[35][1]);
								float Steer_D_err = ((kp_d * (D_err + 0.3*D_err1) + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.3;
								k1 = 1; k3 = 0.1;
								*cmdSteer = k1*Steer_D_err + k3*_yaw;//
								steered = 1;
								cout << "直道\t" << "预瞄点 = （" << X0 << ',' << Y0 << ")\n";
						}
					}
				}
			}
			else {
				if (road[0].sign == 1 || road[0].sign == 2)//车在弯道	
				{
					if (road[1].sign == -1 || (road[1].sign == 0 && (road[0].ahead - road[0].behind) > 30) || road[1].sign == road[0].sign || (road[1].sign == 0 && (road[1].ahead - road[1].behind) < 40))//弯内，距离出弯较远				//大弯道——————>走内弯
					{
						float cur = abs(road[0].dcur / (road[0].ahead - road[0].behind));
						
							X0 = _midline[20][0] + min(0.3*cur, 0.11*cur + 0.19)*_width*sin(_everyslope[20]);
							Y0 = _midline[20][1] - min(0.3*cur, 0.11*cur + 0.19)*_width*cos(_everyslope[20]);
							D_err = getD_eer(X0, Y0);
							D_err1 = getD_eer(_midline[40][0], _midline[40][1]);
							float Steer_D_err = constrain(-1.0, 1.0, ((kp_d * (D_err + 0.3*D_err1) + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.3);
							k1 = 1; k3 = 0.1;
							*cmdSteer = k1*Steer_D_err + k3*_yaw*0.1*fabs(_midline[0][0]);
							steered = 1;
							cout << "在过弯\t预瞄点 = （" << X0 << ',' << Y0 << ")\n";

					}
					else if (road[1].sign == 0 && (road[0].ahead - road[0].behind) <= 30 && (road[1].ahead - road[1].behind > 40))//准备出弯
					{
						
							X0 = _midline[20][0] + (0.014*road[0].behind - 0.12)*_width*sin(_everyslope[20]);
							Y0 = _midline[20][1] - (0.014*road[0].behind - 0.12)*_width*cos(_everyslope[20]);
							D_err = getD_eer(X0, Y0);
							D_err1 = getD_eer(_midline[40][0], _midline[40][1]);
							float Steer_D_err = constrain(-1.0, 1.0, ((kp_d * (D_err + 0.3*D_err1) + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.3);
							k1 = 1; k3 = 0;
							*cmdSteer = k1*Steer_D_err + k3*_yaw*0.1*fabs(_midline[0][0]);
							steered = 1;
							cout << "准备出弯\t预瞄点 = （" << X0 << ',' << Y0 << ")\n";
					

					}

					if (((road[0].sign + road[1].sign) == 3 && (road[0].ahead - road[0].behind) < 30)
						|| (road[1].sign == 0 && (road[0].ahead - road[0].behind) <= 30 && (road[1].ahead - road[1].behind <= 20)))//S弯
					{
						
							X0 = _midline[20][0];
							Y0 = _midline[20][1];
							D_err = getD_eer(X0, Y0);
							D_err1 = getD_eer(_midline[40][0], _midline[40][1]);
							float Steer_D_err = constrain(-1.0, 1.0, ((kp_d * (D_err + 0.3*D_err1) + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.3);
							k1 = 1;
							*cmdSteer = k1*Steer_D_err;
							steered = 1;
							cout << "过S弯！！" << "\t预瞄点 = （" << X0 << ',' << Y0 << ")\n";
					}
				}
			}







			if (!steered || road[8].sign != -1 || _speed<50)//如果不在之前的特殊情况里，或者路况复杂，模型不适用，则用以下简化模型
			{
					D_err = getD_eer(_midline[20][0], _midline[20][1]);
					D_err1 = getD_eer(_midline[40][0], _midline[40][1]);
					double distence = sqrt((_midline[25][0] - _midline[26][0])*(_midline[25][0] - _midline[26][0]) + (_midline[25][1] - _midline[26][1])*(_midline[25][1] - _midline[26][1]));
					*cmdSteer = constrain(-1.0, 1.0, ((kp_d * (D_err + 0.3*D_err1) + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.3);
					cout << "=====正在用简化模型=====" << endl;
			}
			else//如果在之前的模型里，进行最后的steer限制
			{
				 *cmdSteer = constrain(-1, 1, *cmdSteer);
			}

			//**********************************************************************************
			if (curSpeedErr > 0)
			{

				if ((abs(*cmdSteer)<0.5 || _speed<40))
				{
					*cmdAcc = constrain(0.0, 1.0, 1 * kp_s * curSpeedErr + ki_s * speedErrSum + offset);
					*cmdBrake = 0;
				}
				else if (abs(*cmdSteer)>0.5&&abs(*cmdSteer)<0.7)
				{
					*cmdAcc = 0.4*constrain(0.0, 1.0, 1 * kp_s * curSpeedErr + ki_s * speedErrSum + offset);
					*cmdBrake = 0;
				}
				else
				{
					if (abs(*cmdSteer)>0.7)
					{
						*cmdAcc = 0.05 + offset;
						*cmdBrake = 0;
					}
					else
					{
						*cmdAcc = 0.11 + offset;
						*cmdBrake = 0;
					}
				}

			}
			else if (curSpeedErr < 0)
			{
				*cmdBrake = constrain(0.0, 1.0, (-kp_s *curSpeedErr / 5 - offset / 3) * 2);
				*cmdAcc = 0;
			}

			updateGear(cmdGear);

			//print some useful info on the terminal
			printf("expectSpeed : %f ,", expectedSpeed);
			cout << "当前道路为：" << road[0].sign << "下一道路为：" << road[1].sign << endl;

			//状态清零======================
			for (int i = 0; i < 15; i++)
			{
				road[i].sign = -1;
			}
			steered = 0;
		}
		//公路模型
		else {
			if (road[0].sign == 0) {
				float es = 350;
				if (road[1].sign != -1) es = sqrt(40 * road[1].behind + road[1].es*road[1].es);
				if (road[2].sign != -1)es = min(es, sqrt(40 * road[2].behind + road[2].es*road[2].es));
				if (road[3].sign != -1)es = min(es, sqrt(40 * road[3].behind + road[3].es*road[3].es));
				if (road[4].sign != -1)es = min(es, sqrt(40 * road[4].behind + road[4].es*road[4].es));
				if (road[5].sign != -1)es = min(es, sqrt(40 * road[5].behind + road[5].es*road[5].es));
				if (road[6].sign != -1)es = min(es, sqrt(40 * road[6].behind + road[6].es*road[6].es));
				if (road[7].sign != -1)es = min(es, sqrt(40 * road[5].behind + road[7].es*road[7].es));

				es = min(es, road[0].es);
				if (abs(_everydcur[50]) > 75 && es > 130)es = 130;
				expectedSpeed = constrain(60, 350, es);
			}
			if (road[0].sign == 1 || road[0].sign == 2) {

				float es = 300;
				if (road[1].sign != -1) es = sqrt(20 * road[1].behind + road[1].es*road[1].es);
				if (road[2].sign != -1)es = min(es, sqrt(20 * road[2].behind + road[2].es*road[2].es));
				if (road[3].sign != -1)es = min(es, sqrt(20 * road[3].behind + road[3].es*road[3].es));
				if (road[4].sign != -1)es = min(es, sqrt(20 * road[4].behind + road[4].es*road[4].es));
				if (road[5].sign != -1)es = min(es, sqrt(20 * road[5].behind + road[5].es*road[5].es));
				if (road[6].sign != -1)es = min(es, sqrt(20 * road[6].behind + road[6].es*road[6].es));
				if (road[7].sign != -1)es = min(es, sqrt(20 * road[7].behind + road[7].es*road[7].es));

				es = min(es, road[0].es);
				if (abs(_everydcur[50]) > 75 && es > 130)es = 130;
				expectedSpeed = constrain(60, 350, es);
			}

			expectedSpeed = constrain(40, 350, expectedSpeed);
			//expectedSpeed *= 1.1;//速度整体提升
			curSpeedErr = expectedSpeed - _speed;
			speedErrSum = 0.1 * speedErrSum + curSpeedErr;

			//========================== Direction Control =================================		
			//set the param of PID controller
			kp_d = 4;
			ki_d = 0;
			kd_d = 0;

			//道路预测模型
			if (road[0].sign == 0)//车在直道	
			{
				if (road[1].sign == -1 || road[1].sign == 0)//长直道
				{
					int target = constrain(20, 100, road[0].ahead / 2);
					X0 = _midline[target][0];
					Y0 = _midline[target][1];
					D_err = getD_eer(X0, Y0);
					float Steer_D_err = ((kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.1;
					k1 = 1; k3 = 0.1;
					*cmdSteer = k1*Steer_D_err + k3*_yaw;
					steered = 1;
					cout << "直线行驶\t预瞄点 = （" << X0 << ',' << Y0 << ")\n";
				}
				else if (road[1].sign == 1 || road[1].sign == 2)//准备入弯
				{

					if ((road[1].sign + road[2].sign) == 3 && (road[1].ahead - road[1].behind) < 30)//S弯 
					{
						X0 = _midline[road[1].ahead][0] * 0.5;
						Y0 = _midline[road[1].ahead][1] + 50;
						if (getDistance(X0, Y0) < 20)
						{
							X0 = _midline[road[1].ahead + 10][0] * 0.5;
							Y0 = _midline[road[1].ahead + 10][1] + 50;
						}
						D_err = getD_eer(X0, Y0);
						float Steer_D_err;
						if (dirt) Steer_D_err = ((kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.3;
						else Steer_D_err = ((kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.1;

						k1 = 1;
						*cmdSteer = k1*Steer_D_err;
						steered = 1;
						cout << "准备过S弯\t" << "预瞄点 = （" << X0 << ',' << Y0 << ")\n";
					}
					else //未知的大弯 或者 可以预知的小弯 或者 其他任何弯 或者 第一弯太大也归到此类
					{
						if ((road[0].ahead > 30 && abs(road[1].dcur) > 20))//预入弯
						{
							if (road[1].sign == 1)//左转
								X0 = _midline[road[0].ahead - 5][0] + (_width*0.5 - 2.5);
							else if (road[1].sign == 2)//右转
								X0 = _midline[road[0].ahead - 5][0] - (_width*0.5 - 2.5);
							Y0 = _midline[road[0].ahead - 5][1];
							D_err = getD_eer(X0, Y0);
							float Steer_D_err;
							if (dirt) Steer_D_err = ((kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.3;
							else Steer_D_err = ((kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.1;

							k1 = 1; k3 = 0.1;
							*cmdSteer = k1*Steer_D_err + k3*_yaw;
							steered = 1;
							cout << "预入弯\t预瞄点 = （" << X0 << ',' << Y0 << ")\troad.r=" << road[1].r << "\troad.dcur=" << road[1].dcur << endl;
						}
						
						else//其他入弯综合处理☆
						{
							road[1].r = fabs(((road[1].ahead - road[1].behind) * 180) / (road[1].dcur*PI));//在这里可以解决getRoad里得不到正确的r的问题
							float angleR = acos(road[1].r / (road[1].r + 0.5*_width));
							int target = (road[1].r + 0.5*_width)*angleR;
							X0 = _midline[target][0];
							Y0 = _midline[target][1];

							D_err = getD_eer(X0, Y0);
							float Steer_D_err;
							if (dirt)  Steer_D_err = ((kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.3;
							else Steer_D_err = ((kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.1;
							k1 = 0.9; k3 = 0.1;
							*cmdSteer = k1*Steer_D_err + k3*_yaw;//
							steered = 1;
							cout << "入弯\t"<< "预瞄点 = （" << X0 << ',' << Y0 << ")\troad.r=" << road[1].r << "\troad.dcur=" << road[1].dcur << endl;
						}
					}
				}
			}
			else if (road[0].sign == 1 || road[0].sign == 2)//车在弯道	
			{
				if (road[1].sign == -1 || ((road[1].sign == 0 && (road[0].ahead - road[0].behind) > 10)) || road[1].sign == road[0].sign
					|| ((road[1].sign + road[0].sign == 3) && (road[0].ahead - road[0].behind) >= 30))//弯内，距离出弯较远				//大弯道——————>走内弯
				{
					road[0].r = fabs(((road[0].ahead - road[0].behind) * 180) / (road[0].dcur*PI));//在这里可以解决getRoad里得不到正确的r的问题
					float angleR = acos(road[0].r / (road[0].r + 0.5*_width));
					int target = (road[0].r + 0.5*_width)*angleR;
					X0 = _midline[target][0];
					Y0 = _midline[target][1];
					D_err = getD_eer(X0, Y0);
					float Steer_D_err;
					Steer_D_err = ((kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.1;

					k1 = 1; k3 = 0.2;//理论上，强化k1 弱化k3
					*cmdSteer = k1*Steer_D_err + k3*_yaw;
					steered = 1;
					cout << "过弯\t预瞄点 = （" << X0 << ',' << Y0 << ")\troad.r=" << road[0].r << " \troad.dcur = " << road[0].dcur <<endl;
				}
				else if (road[1].sign == 0 && (road[0].ahead - road[0].behind) <= 10 && (road[1].ahead - road[1].behind > 20))//准备出弯
				{

					X0 = _midline[20][0] + (0.014*road[0].behind - 0.12)*_width*sin(_everyslope[20]);;
					Y0 = _midline[20][1] - (0.014*road[0].behind - 0.12)*_width*cos(_everyslope[20]);;

					D_err = getD_eer(X0, Y0);
					float Steer_D_err;
					Steer_D_err = ((kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.1;

					k1 = 0.25; k3 = 0.5;//削弱k1 强化k3
					
					*cmdSteer = k1*Steer_D_err + k3*_yaw*0.1*fabs(_midline[0][0]);
					steered = 1;
					cout << "准备出弯\t预瞄点 = （" << X0 << ',' << Y0 << ")\n";
				}
				else if (((road[0].sign + road[1].sign) == 3 && (road[0].ahead - road[0].behind) < 30)
					|| (road[1].sign == 0 && (road[0].ahead - road[0].behind) <= 10 && (road[1].ahead - road[1].behind <= 20)))//S弯
				{
					X0 = _midline[road[0].ahead][0] * 0.5;
					Y0 = _midline[road[0].ahead][1] + 40;
					D_err = getD_eer(X0, Y0);
					float Steer_D_err;
					Steer_D_err = ((kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.1;

					k1 = 1;
					*cmdSteer = k1*Steer_D_err;
					steered = 1;

					cout << "过S弯" << "\t预瞄点 = （" << X0 << ',' << Y0 << ")\n";
					if (getDistance(X0, Y0)<20)
					{
						X0 = _midline[road[0].ahead + 40][0];
						Y0 = _midline[road[0].ahead + 40][1] + 40;
					}
				}

			}

			if (!steered)//如果不在之前的情况里，用以下简化模型
			{
				//float x, y;
				if (_midline[15][0] > 0 && _midline[15][1] < 0)D_err = -1;
				else {
					if (_midline[15][0] < 0 && _midline[15][1] < 0)D_err = 1;
					else D_err = getD_eer(_midline[15][0], _midline[15][1]);
				}
				D_err1 = getD_eer(_midline[35][0], _midline[35][1]);
				double distence = sqrt((_midline[25][0] - _midline[26][0])*(_midline[25][0] - _midline[26][0])
									+ (_midline[25][1] - _midline[26][1])*(_midline[25][1] - _midline[26][1]));
				*cmdSteer = constrain(-1.0, 1.0, ((kp_d * (D_err + 0.3*D_err1) + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.3);
				cout << "=====正在用简化模型=====" << endl;
			}
			else//如果在之前的模型里，进行最后的steer限制
			{
				*cmdSteer = constrain(-1, 1, *cmdSteer);
			}

			if ((0.5*_width - fabs(_midline[0][0]) < _width / 8))//出界处理
			{
				if (road[0].sign == 0)//直道出界
				{
					*cmdSteer = constrain(-1, 1, 0.1*_yaw - 0.1 * 2 * _midline[0][0] / _width);
					//cout << "警告！！直道出界！！" << endl;
				}
				else if (road[0].sign == 1 || road[0].sign == 2)
				{
					if (_midline[0][0] * _everycur[0] > 0)//内弯出界
					{
						*cmdSteer = 0.1*_yaw;
						//cout << "警告！！内弯出界！！" << endl;
					}
					else//外弯出界
					{
						*cmdSteer = constrain(-1, 1, 0.3*_yaw - 0.15 * 2 * _midline[0][0] / _width);
						//cout << "警告！！外弯出界！！" << endl;
					}
				}
			}

			//**********************************************************************************
			if (curSpeedErr > 0)
			{

				if ((abs(*cmdSteer)<0.5 || _speed<40))
				{
					*cmdAcc = constrain(0.0, 1.0, 1 * kp_s * curSpeedErr + ki_s * speedErrSum + offset);
					*cmdBrake = 0;
				}
				else if (abs(*cmdSteer)>0.5&&abs(*cmdSteer)<0.7)
				{
					*cmdAcc = 0.4*constrain(0.0, 1.0, 1 * kp_s * curSpeedErr + ki_s * speedErrSum + offset);
					*cmdBrake = 0;
				}
				else
				{
					if (abs(*cmdSteer)>0.7)
					{
						*cmdAcc = 0.05 + offset;
						*cmdBrake = 0;
					}
					else
					{
						*cmdAcc = 0.11 + offset;
						*cmdBrake = 0;
					}
				}

			}
			else if (curSpeedErr < 0)
			{
				*cmdBrake = constrain(0.0, 1.0, (-kp_s *curSpeedErr / 5 - offset / 3) * 2);
				*cmdAcc = 0;
			}

			updateGear(cmdGear);

			//print some useful info on the terminal
			printf("expectSpeed : %f ,", expectedSpeed);
			//cout << "下dcur：" << road[1].dcur << "下cur：" << road[1].dcur / (road[1].ahead - road[1].behind) << endl;

			//状态清零======================
			for (int i = 0; i < 15; i++)
			{
				road[i].sign = -1;
			}
			steered = 0;
		}
		 
	}
}

void PIDParamSetter()
{

	kp_s = 0.02;
	ki_s = 0;
	kd_s = 0;
	kp_d = 1.35;
	ki_d = 0.151;
	kd_d = 0.10;
	parameterSet = true;

}

void updateGear(int *cmdGear)
{
	if (_gearbox == 1)
	{
		if (_speed >= 60 && topGear >1)
		{
			*cmdGear = 2;
		}
		else
		{
			*cmdGear = 1;
		}
	}
	else if (_gearbox == 2)
	{
		if (_speed <= 45)
		{
			*cmdGear = 1;
		}
		else if (_speed >= 105 && topGear >2)
		{
			*cmdGear = 3;
		}
		else
		{
			*cmdGear = 2;
		}
	}
	else if (_gearbox == 3)
	{
		if (_speed <= 90)
		{
			*cmdGear = 2;
		}
		else if (_speed >= 145 && topGear >3)
		{
			*cmdGear = 4;
		}
		else
		{
			*cmdGear = 3;
		}
	}
	else if (_gearbox == 4)
	{
		if (_speed <= 131)
		{
			*cmdGear = 3;
		}
		else if (_speed >= 187 && topGear >4)
		{
			*cmdGear = 5;
		}
		else
		{
			*cmdGear = 4;
		}
	}
	else if (_gearbox == 5)
	{
		if (_speed <= 173)
		{
			*cmdGear = 4;
		}
		else if (_speed >= 234 && topGear >5)
		{
			*cmdGear = 6;
		}
		else
		{
			*cmdGear = 5;
		}
	}
	else if (_gearbox == 6)
	{
		if (_speed <= 219)
		{
			*cmdGear = 5;
		}
		else
		{
			*cmdGear = 6;
		}
	}
	else
	{
		*cmdGear = 1;
	}
}

double constrain(double lowerBoundary, double upperBoundary, double input)
{
	if (input > upperBoundary)
		return upperBoundary;
	else if (input < lowerBoundary)
		return lowerBoundary;
	else
		return input;
}

circle getR(float x1, float y1, float x2, float y2, float x3, float y3)
{
	double a, b, c, d, e, f;
	double r, x, y;

	a = 2 * (x2 - x1);
	b = 2 * (y2 - y1);
	c = x2*x2 + y2*y2 - x1*x1 - y1*y1;
	d = 2 * (x3 - x2);
	e = 2 * (y3 - y2);
	f = x3*x3 + y3*y3 - x2*x2 - y2*y2;
	x = (b*f - e*c) / (b*d - e*a);
	y = (d*c - a*f) / (b*d - e*a);
	r = sqrt((x - x1)*(x - x1) + (y - y1)*(y - y1));
	x = constrain(-1000.0, 1000.0, x);
	y = constrain(-1000.0, 1000.0, y);
	r = constrain(1.0, 500.0, r);
	int sign = (x>0) ? 1 : -1;
	circle tmp = { r,sign };
	return tmp;
}

// 道路识别
void everyCur(float _midline[200][2]) {
	//everycur
	for (int i = 1; i < 199; i++) {
		float a1 = _midline[i][0] - _midline[i - 1][0];
		float b1 = _midline[i][1] - _midline[i - 1][1];
		float a2 = _midline[i + 1][0] - _midline[i][0];
		float b2 = _midline[i + 1][1] - _midline[i][1];
		float cross = a1*b2 - a2*b1;
		float dot = a1*a2 + b1*b2;
		if (dot == 0) {
			_everycur[i] = cross > 0 ? 90 : -90;
			continue;
		}
		float theta = acos(dot / sqrt((a1*a1 + b1*b1)*(a2*a2 + b2*b2)));
		if (!(theta <= DBL_MAX && theta >= -DBL_MAX))theta = 0;
		_everycur[i] = cross > 0 ? theta / PI * 180 : -theta / PI * 180;
		if (abs(_everycur[i]) < 0.05) _everycur[i] = 0;
	}
	_everycur[0] = _everycur[1];
	_everycur[199] = _everycur[198];
}

void everyDcur(float _midline[200][2])
{
	float a1 = _midline[1][0] - _midline[0][0];
	float b1 = _midline[1][1] - _midline[0][1];
	//everyDcur
	for (int i = 0; i < 199; i++) {
		float a2 = _midline[i + 1][0] - _midline[i][0];
		float b2 = _midline[i + 1][1] - _midline[i][1];
		float cross = a1*b2 - a2*b1;
		float dot = a1*a2 + b1*b2;
		if (dot == 0) {
			_everydcur[i] = cross > 0 ? 90 : -90;
			continue;
		}
		float theta = acos(dot / sqrt((a1*a1 + b1*b1)*(a2*a2 + b2*b2)));
		if (!(theta <= DBL_MAX && theta >= -DBL_MAX))theta = 0;
		_everydcur[i] = cross > 0 ? theta / PI * 180 : -theta / PI * 180;
	}
	_everydcur[199] = _everydcur[198];
}

void everySlope(float _midline[200][2])
{
	//everySlope
	for (int i = 0; i < 199; i++)
	{
		float x1 = _midline[i][0], y1 = _midline[i][1];
		float x2 = _midline[i + 1][0], y2 = _midline[i + 1][1];
		_everyslope[i] = atan((y2 - y1) / x2 - x1);			
	}
	_everyslope[199] = _everyslope[198];

}
int sign(float a) {
	if (a == 0)return 0;
	else {
		return a > 0 ? 1 : -1;
	}
}

float getD_eer(float x, float y)
{
	if (x > 0 && y < 0)return -1;
	else {
		if (x < 0 && y < 0)return 1;
		else return -atan2(x, y);
	}
}

float getDistance(float x, float y)
{
	return sqrt(x*x + y*y);
}

void getRoad(float _everycur[200], float _everydcur[200])
{
	//分土路公路两种识别方式
	if (dirt) {
		int count = 0;
		int a = 0, b = 0;
		float cur = 0;
		for (int i = 2; i < 200; i++) {
			if (sign(_everycur[i]) != sign(_everycur[i - 1])) {
				if (abs(_everydcur[i - 1] - _everydcur[b])>5 || _everycur[i - 2] == 0) {
					a = i;
					road[count].ahead = a;
					road[count].behind = b;
					road[count].dcur = _everydcur[a] - _everydcur[b];
					road[count].r = -1;//土路未用到r
					if (abs(road[count].dcur) < 5) {
						road[count].sign = 0;
						road[count].es = 350;
					}
					else {
						if (road[count].dcur > 0) road[count].sign = 1;
						if (road[count].dcur < 0) road[count].sign = 2;
						cur = abs(road[count].dcur) / ((road[count].ahead - road[count].behind));
						road[count].es = 350;
						if (cur > 0.3) {
							road[count].es = 300;
							if (abs(road[count].dcur) > 35)road[count].es = 200;
							if (cur>0.6)road[count].es = 170;
							if (abs(road[count].dcur) > 45 && cur>0.6)road[count].es = 165;
							if (abs(road[count].dcur) > 75)road[count].es = 155;
							if (abs(road[count].dcur) > 85)road[count].es = 140;
							if (cur>1.2)road[count].es = 140;
							if (abs(road[count].dcur) > 90 && cur>1.5)road[count].es = 135;
							if (cur>1.7)road[count].es = 125;
							if (abs(road[count].dcur) > 110)road[count].es = 110;
							if (abs(road[count].dcur) > 125 && cur>1)road[count].es = 100;
							if (abs(road[count].dcur) > 135 && cur>1)road[count].es = 90;
							if (abs(road[count].dcur) > 150 && cur>1)road[count].es = 90;
							if (cur>2.5)road[count].es = 90;
						}
					}
					b = a;
					count++;
					i++;
				}
			}
		}
		a = 199;
		road[count].ahead = a;
		road[count].behind = b;
		road[count].dcur = _everydcur[a] - _everydcur[b];
		if (abs(road[count].dcur) < 5) {
			road[count].sign = 0;
			road[count].es = 350;
		}
		else {
			if (road[count].dcur > 0) road[count].sign = 1;
			if (road[count].dcur < 0) road[count].sign = 2;
			cur = abs(road[count].dcur) / (road[count].ahead - road[count].behind);
			road[count].es = 350;
			if (cur > 0.3) {
				road[count].es = 300;
				if (abs(road[count].dcur) > 20)road[count].es = 200;
				if (abs(road[count].dcur) > 55 && cur>0.4)road[count].es = 180;
				if (abs(road[count].dcur) > 75)road[count].es = 155;
				if (abs(road[count].dcur) > 85)road[count].es = 140;
				if (cur>1.2)road[count].es = 140;
				if (abs(road[count].dcur) > 95)road[count].es = 140;
				if (cur>1.6)road[count].es = 110;
				if (abs(road[count].dcur) > 110)road[count].es = 110;
				if (abs(road[count].dcur) > 125 && cur>1)road[count].es = 100;
				if (abs(road[count].dcur) > 135 && cur>1)road[count].es = 90;
				if (abs(road[count].dcur) > 150 && cur>1)road[count].es = 90;
				if (cur>2.5)road[count].es = 90;
			}


		}

	}
	else {
		int count = 0;
		int a = 0, b = 0;
		float cur = 0;
		for (int i = 2; i < 200; i++) {
			if (sign(_everycur[i]) != sign(_everycur[i - 1])) {
				if (abs(_everydcur[i - 1] - _everydcur[b])>5 || _everycur[i - 2] == 0) {
					a = i;
					road[count].ahead = a;
					road[count].behind = b;
					road[count].dcur = _everydcur[a] - _everydcur[b];
					//添加road.r的求法
					road[count].r = fabs(((float)(a - b) / road[count].dcur)*((float)180 / PI));

					if (abs(road[count].dcur) < 15) {
						road[count].sign = 0;
						road[count].es = 350;
					}
					else {
						if (road[count].dcur > 0) road[count].sign = 1;
						if (road[count].dcur < 0) road[count].sign = 2;

						cur = abs(road[count].dcur) / (a - b);
						road[count].es = 350;
						
							if (cur > 0.3) {
								road[count].es = 300;
								if (abs(road[count].dcur) > 20)road[count].es = 250;//不能改
								if (abs(road[count].dcur) > 40 && cur>0.5)road[count].es = 185;
								if (abs(road[count].dcur) > 50 && cur>0.5)road[count].es = 180;
								if (abs(road[count].dcur) > 50 && cur>0.5)road[count].es = 170;
								if (abs(road[count].dcur) > 45 && cur>1)road[count].es = 155;
								if (abs(road[count].dcur) > 75)road[count].es = 155;

								if (abs(road[count].dcur) > 85 && cur>1)road[count].es = 140;
								if (cur>1.1)road[count].es = 140;
								if (abs(road[count].dcur) > 95 && cur>1)road[count].es = 140;
								if (cur>1.5)road[count].es = 125;
								if (abs(road[count].dcur) > 105 && cur>1)road[count].es = 110;

								if (abs(road[count].dcur) > 125 && cur>1)road[count].es = 100;
								if (abs(road[count].dcur) > 135 && cur>1)road[count].es = 100;

								if (abs(road[count].dcur) > 150 && cur>1)road[count].es = 100;
								if (cur>2.5)road[count].es = 90;//不能改
							}
					}
					b = a;
					count++;
					i++;
				}
			}
		}
		a = 199;
		road[count].ahead = a;
		road[count].behind = b;
		road[count].dcur = _everydcur[a] - _everydcur[b];
		if (abs(road[count].dcur) < 5) {
			road[count].sign = 0;
			road[count].es = 350;
		}
		else {
			if (road[count].dcur > 0) road[count].sign = 1;
			if (road[count].dcur < 0) road[count].sign = 2;

			cur = abs(road[count].dcur) / (a - b);
			road[count].es = 350;
			
			
				if (cur > 0.3) {
					road[count].es = 300;
					if (abs(road[count].dcur) > 20)road[count].es = 250;//不能改
					if (abs(road[count].dcur) > 40 && cur>0.5)road[count].es = 185;
					if (abs(road[count].dcur) > 50 && cur>0.5)road[count].es = 180;
					if (abs(road[count].dcur) > 50 && cur>0.5)road[count].es = 170;
					if (abs(road[count].dcur) > 45 && cur>1)road[count].es = 155;
					if (abs(road[count].dcur) > 75)road[count].es = 155;

					if (abs(road[count].dcur) > 85 && cur>1)road[count].es = 140;
					if (cur>1.1)road[count].es = 140;
					if (abs(road[count].dcur) > 95 && cur>1)road[count].es = 140;
					if (cur>1.5)road[count].es = 125;
					if (abs(road[count].dcur) > 105 && cur>1)road[count].es = 110;

					if (abs(road[count].dcur) > 125 && cur>1)road[count].es = 100;
					if (abs(road[count].dcur) > 135 && cur>1)road[count].es = 100;

					if (abs(road[count].dcur) > 150 && cur>1)road[count].es = 100;
					if (cur>2.5)road[count].es = 90;//不能改
				}
		}
	}
}




