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


//*************************��·ʶ��***************************
struct kindOfRoad
{
	int sign = -1;//0-ֱ����1-�������2-�������-1δʹ��
	int ahead, behind;//�öε�·����ʼ����յ�
	float dcur;//�öε����ܽǶ�	//��һ�ε�·ת���ĽǶ�	����˳ʱ��	������ʱ��
	float es;//Ŀ���ٶ�
	float r;//������ʰ뾶
};
kindOfRoad road[15];
static float _everycur[200];				//·�������ʽǣ��Ƕ��ƣ�			//������ʱ��	����˳ʱ��	eg:0.1 0.14 0.57
static float _everydcur[200];				//����0-1����ת���ĽǶ�			//������ʱ��	����˳ʱ��
static float _everyslope[200];				//·����б��֮�Ƕȣ��ڳ�������ϵ�У��������ƣ�	



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
int _count = 0;//����matlab���
float cur;
float dcur;
//========�������ר��============
bool steered = 0;//������	0=δ����steer	1=������steer
float  k1 = 1, k2 = 0, k3 = 0.1;//��ͬ������Ƶ�Ȩֵ		k1Ԥ���	k2·ƫ��	k3ƫ����
float X0, Y0;//Ԥ���

//=====ʶ��·or��·=====
bool dirt = 0;//0-��·��1-��·
bool distinguish = 0;	//0��δʶ��	1��ʶ�����
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
void everyCur(float _midline[200][2]);//��ȡeverycur
void everyDcur(float _midline[200][2]);//��ȡeverydcur
void everySlope(float _midline[200][2]);//��ȡeveryslope
void getRoad(float _everycur[200], float _everydcur[200]);//��ȡ·������
float getDistance(float x, float y);//��ȡ��ԭ�����Ĺ��ߺ���
int sign(float a);//��ȡ���ŵĹ��ߺ���
float getD_eer(float x, float y);//��ȡD_err//����Ԥ���steer����
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

	//��·ʶ��
	//cout << "��ǰ��·Ϊ��" << road[0].sign << " ǰ��������" << road[0].ahead << " �Ƕȣ�" << road[0].dcur << endl;
	//cout << "��һ��·Ϊ��" << road[1].sign << " ���룺" << road[1].behind << " �Ƕȣ�" << road[1].dcur << endl;

	int aim = 0;
	cout << "��ǰ��·Ϊ��";
	switch (road[aim].sign)
	{
	case 0:cout << "ֱ��; "; break;
	case 1:cout << "�����; "; break;
	case 2:cout << "�����; "; break;
	default:cout << "δʶ��; "; break;
	}	
	cout<<" ��㣺" << road[aim].behind << " �յ㣺" << road[aim].ahead << " ת���Ƕȣ�" << road[aim].dcur;
	if (road[aim].sign == 1 || road[aim].sign == 2) { cout << "���ʰ뾶��" << road[aim].r; }
	cout << "�����ٶȣ�" << road[aim].es << endl;

	if (parameterSet == false)		// Initialization Part
	{
		PIDParamSetter();
	}
	else
	{
		//ʶ��·or��·
		if (!distinguish&&_gearbox == 2) {
			if (count_dirt > 145) { dirt = 1; distinguish = 1; }
			else { dirt = 0; distinguish = 1; }
		}
		if (!distinguish&&count_dirt < 500)count_dirt++;
		
		//����·�͹�·���ģ��
		//��·ģ�ͣ�
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
			//expectedSpeed *= 1.1;//�ٶ���������
			curSpeedErr = expectedSpeed - _speed;
			speedErrSum = 0.1 * speedErrSum + curSpeedErr;

			//========================== Direction Control =================================		
			//set the param of PID controller
			kp_d = 4;
			ki_d = 0;
			kd_d = 0;

			//��·Ԥ��ģ��
			if (road[0].sign == 0)//����ֱ��		
			{
				if (road[1].sign == -1 || road[1].sign == 0)//��ֱ��
				{

					X0 = _midline[road[0].ahead][0];
					Y0 = _midline[road[0].ahead][1];
					D_err = getD_eer(X0, Y0);
					float Steer_D_err = kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff;
					k1 = 1; k3 = 0.1;
					*cmdSteer = k1*Steer_D_err + k3*_yaw;
					steered = 1;
					cout << "ֱ����ʻ\tԤ��� = ��" << X0 << ',' << Y0 << ")" << endl;
				}
				else if (road[1].sign == 1 || road[1].sign == 2)//׼������
				{
					if ((road[1].sign + road[2].sign) == 3 && (road[1].ahead - road[1].behind) < 60)//S�� 
					{
						if (road[0].ahead > 30 && abs(road[1].dcur) > 20)//Ԥ����
						{
							if (dirt) {
								if (road[1].sign == 1)//��ת
									X0 = _midline[road[0].ahead][0] + _width * 1 / 8;
								else if (road[1].sign == 2)//��ת
									X0 = _midline[road[0].ahead][0] - _width * 1 / 8;

								Y0 = _midline[road[0].ahead][1];
								D_err = getD_eer(X0, Y0);
								float Steer_D_err = ((kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.3;
								k1 = 1; k3 = 0.1;
								*cmdSteer = k1*Steer_D_err + k3*_yaw;
								steered = 1;
								cout << "Ԥ����\tԤ��� = ��" << X0 << ',' << Y0 << ")" << endl;
							}
							else {
								if (road[1].sign == 1)//��ת
									X0 = _midline[road[0].ahead][0] + _width * 1 / 8;
								else if (road[1].sign == 2)//��ת
									X0 = _midline[road[0].ahead][0] - _width * 1 / 8;

								Y0 = _midline[road[0].ahead - 5][1];
								D_err = getD_eer(X0, Y0);
								float Steer_D_err = ((kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.1;
								k1 = 1; k3 = 0.1;
								*cmdSteer = k1*Steer_D_err + k3*_yaw;
								steered = 1;
								cout << "Ԥ����\tԤ��� = ��" << X0 << ',' << Y0 << ")" << endl;
							}
						}
						else {
								D_err = getD_eer(_midline[20][0], _midline[20][1]);
								D_err1 = getD_eer(_midline[40][0], _midline[40][1]);
								double distence = sqrt((_midline[25][0] - _midline[26][0])*(_midline[25][0] - _midline[26][0]) + (_midline[25][1] - _midline[26][1])*(_midline[25][1] - _midline[26][1]));
								*cmdSteer = constrain(-1.0, 1.0, ((kp_d * (D_err + 0.3*D_err1) + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.3);
								cout << "=====�����ü�ģ��=====" << endl;
						}
					}
					else //δ֪�Ĵ��� ���� ����Ԥ֪��С�� ���� �����κ��� ���� ��һ��̫�� Ҳ�鵽����
					{
						if (road[0].ahead > 30 && abs(road[1].dcur)>20)//Ԥ����
						{
								if (road[1].sign == 1)//��ת
									X0 = _midline[road[0].ahead][0] + _width * 3 / 8;
								else if (road[1].sign == 2)//��ת
									X0 = _midline[road[0].ahead][0] - _width * 3 / 8;
								Y0 = _midline[road[0].ahead][1];
								D_err = getD_eer(X0, Y0);
								float Steer_D_err = ((kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.3;
								k1 = 1; k3 = 0.1;
								*cmdSteer = k1*Steer_D_err + k3*_yaw;
								steered = 1;
								cout << "Ԥ����\tԤ��� = ��" << X0 << ',' << Y0 << ")" << endl;
						}
						else//���������ۺϴ����
						{
								X0 = _midline[20][0];
								Y0 = _midline[20][1];
								D_err = getD_eer(X0, Y0);
								D_err1 = getD_eer(_midline[35][0], _midline[35][1]);
								float Steer_D_err = ((kp_d * (D_err + 0.3*D_err1) + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.3;
								k1 = 1; k3 = 0.1;
								*cmdSteer = k1*Steer_D_err + k3*_yaw;//
								steered = 1;
								cout << "ֱ��\t" << "Ԥ��� = ��" << X0 << ',' << Y0 << ")\n";
						}
					}
				}
			}
			else {
				if (road[0].sign == 1 || road[0].sign == 2)//�������	
				{
					if (road[1].sign == -1 || (road[1].sign == 0 && (road[0].ahead - road[0].behind) > 30) || road[1].sign == road[0].sign || (road[1].sign == 0 && (road[1].ahead - road[1].behind) < 40))//���ڣ���������Զ				//�����������������>������
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
							cout << "�ڹ���\tԤ��� = ��" << X0 << ',' << Y0 << ")\n";

					}
					else if (road[1].sign == 0 && (road[0].ahead - road[0].behind) <= 30 && (road[1].ahead - road[1].behind > 40))//׼������
					{
						
							X0 = _midline[20][0] + (0.014*road[0].behind - 0.12)*_width*sin(_everyslope[20]);
							Y0 = _midline[20][1] - (0.014*road[0].behind - 0.12)*_width*cos(_everyslope[20]);
							D_err = getD_eer(X0, Y0);
							D_err1 = getD_eer(_midline[40][0], _midline[40][1]);
							float Steer_D_err = constrain(-1.0, 1.0, ((kp_d * (D_err + 0.3*D_err1) + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.3);
							k1 = 1; k3 = 0;
							*cmdSteer = k1*Steer_D_err + k3*_yaw*0.1*fabs(_midline[0][0]);
							steered = 1;
							cout << "׼������\tԤ��� = ��" << X0 << ',' << Y0 << ")\n";
					

					}

					if (((road[0].sign + road[1].sign) == 3 && (road[0].ahead - road[0].behind) < 30)
						|| (road[1].sign == 0 && (road[0].ahead - road[0].behind) <= 30 && (road[1].ahead - road[1].behind <= 20)))//S��
					{
						
							X0 = _midline[20][0];
							Y0 = _midline[20][1];
							D_err = getD_eer(X0, Y0);
							D_err1 = getD_eer(_midline[40][0], _midline[40][1]);
							float Steer_D_err = constrain(-1.0, 1.0, ((kp_d * (D_err + 0.3*D_err1) + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.3);
							k1 = 1;
							*cmdSteer = k1*Steer_D_err;
							steered = 1;
							cout << "��S�䣡��" << "\tԤ��� = ��" << X0 << ',' << Y0 << ")\n";
					}
				}
			}







			if (!steered || road[8].sign != -1 || _speed<50)//�������֮ǰ��������������·�����ӣ�ģ�Ͳ����ã��������¼�ģ��
			{
					D_err = getD_eer(_midline[20][0], _midline[20][1]);
					D_err1 = getD_eer(_midline[40][0], _midline[40][1]);
					double distence = sqrt((_midline[25][0] - _midline[26][0])*(_midline[25][0] - _midline[26][0]) + (_midline[25][1] - _midline[26][1])*(_midline[25][1] - _midline[26][1]));
					*cmdSteer = constrain(-1.0, 1.0, ((kp_d * (D_err + 0.3*D_err1) + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.3);
					cout << "=====�����ü�ģ��=====" << endl;
			}
			else//�����֮ǰ��ģ�����������steer����
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
			cout << "��ǰ��·Ϊ��" << road[0].sign << "��һ��·Ϊ��" << road[1].sign << endl;

			//״̬����======================
			for (int i = 0; i < 15; i++)
			{
				road[i].sign = -1;
			}
			steered = 0;
		}
		//��·ģ��
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
			//expectedSpeed *= 1.1;//�ٶ���������
			curSpeedErr = expectedSpeed - _speed;
			speedErrSum = 0.1 * speedErrSum + curSpeedErr;

			//========================== Direction Control =================================		
			//set the param of PID controller
			kp_d = 4;
			ki_d = 0;
			kd_d = 0;

			//��·Ԥ��ģ��
			if (road[0].sign == 0)//����ֱ��	
			{
				if (road[1].sign == -1 || road[1].sign == 0)//��ֱ��
				{
					int target = constrain(20, 100, road[0].ahead / 2);
					X0 = _midline[target][0];
					Y0 = _midline[target][1];
					D_err = getD_eer(X0, Y0);
					float Steer_D_err = ((kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.1;
					k1 = 1; k3 = 0.1;
					*cmdSteer = k1*Steer_D_err + k3*_yaw;
					steered = 1;
					cout << "ֱ����ʻ\tԤ��� = ��" << X0 << ',' << Y0 << ")\n";
				}
				else if (road[1].sign == 1 || road[1].sign == 2)//׼������
				{

					if ((road[1].sign + road[2].sign) == 3 && (road[1].ahead - road[1].behind) < 30)//S�� 
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
						cout << "׼����S��\t" << "Ԥ��� = ��" << X0 << ',' << Y0 << ")\n";
					}
					else //δ֪�Ĵ��� ���� ����Ԥ֪��С�� ���� �����κ��� ���� ��һ��̫��Ҳ�鵽����
					{
						if ((road[0].ahead > 30 && abs(road[1].dcur) > 20))//Ԥ����
						{
							if (road[1].sign == 1)//��ת
								X0 = _midline[road[0].ahead - 5][0] + (_width*0.5 - 2.5);
							else if (road[1].sign == 2)//��ת
								X0 = _midline[road[0].ahead - 5][0] - (_width*0.5 - 2.5);
							Y0 = _midline[road[0].ahead - 5][1];
							D_err = getD_eer(X0, Y0);
							float Steer_D_err;
							if (dirt) Steer_D_err = ((kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.3;
							else Steer_D_err = ((kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.1;

							k1 = 1; k3 = 0.1;
							*cmdSteer = k1*Steer_D_err + k3*_yaw;
							steered = 1;
							cout << "Ԥ����\tԤ��� = ��" << X0 << ',' << Y0 << ")\troad.r=" << road[1].r << "\troad.dcur=" << road[1].dcur << endl;
						}
						
						else//���������ۺϴ����
						{
							road[1].r = fabs(((road[1].ahead - road[1].behind) * 180) / (road[1].dcur*PI));//��������Խ��getRoad��ò�����ȷ��r������
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
							cout << "����\t"<< "Ԥ��� = ��" << X0 << ',' << Y0 << ")\troad.r=" << road[1].r << "\troad.dcur=" << road[1].dcur << endl;
						}
					}
				}
			}
			else if (road[0].sign == 1 || road[0].sign == 2)//�������	
			{
				if (road[1].sign == -1 || ((road[1].sign == 0 && (road[0].ahead - road[0].behind) > 10)) || road[1].sign == road[0].sign
					|| ((road[1].sign + road[0].sign == 3) && (road[0].ahead - road[0].behind) >= 30))//���ڣ���������Զ				//�����������������>������
				{
					road[0].r = fabs(((road[0].ahead - road[0].behind) * 180) / (road[0].dcur*PI));//��������Խ��getRoad��ò�����ȷ��r������
					float angleR = acos(road[0].r / (road[0].r + 0.5*_width));
					int target = (road[0].r + 0.5*_width)*angleR;
					X0 = _midline[target][0];
					Y0 = _midline[target][1];
					D_err = getD_eer(X0, Y0);
					float Steer_D_err;
					Steer_D_err = ((kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.1;

					k1 = 1; k3 = 0.2;//�����ϣ�ǿ��k1 ����k3
					*cmdSteer = k1*Steer_D_err + k3*_yaw;
					steered = 1;
					cout << "����\tԤ��� = ��" << X0 << ',' << Y0 << ")\troad.r=" << road[0].r << " \troad.dcur = " << road[0].dcur <<endl;
				}
				else if (road[1].sign == 0 && (road[0].ahead - road[0].behind) <= 10 && (road[1].ahead - road[1].behind > 20))//׼������
				{

					X0 = _midline[20][0] + (0.014*road[0].behind - 0.12)*_width*sin(_everyslope[20]);;
					Y0 = _midline[20][1] - (0.014*road[0].behind - 0.12)*_width*cos(_everyslope[20]);;

					D_err = getD_eer(X0, Y0);
					float Steer_D_err;
					Steer_D_err = ((kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.1;

					k1 = 0.25; k3 = 0.5;//����k1 ǿ��k3
					
					*cmdSteer = k1*Steer_D_err + k3*_yaw*0.1*fabs(_midline[0][0]);
					steered = 1;
					cout << "׼������\tԤ��� = ��" << X0 << ',' << Y0 << ")\n";
				}
				else if (((road[0].sign + road[1].sign) == 3 && (road[0].ahead - road[0].behind) < 30)
					|| (road[1].sign == 0 && (road[0].ahead - road[0].behind) <= 10 && (road[1].ahead - road[1].behind <= 20)))//S��
				{
					X0 = _midline[road[0].ahead][0] * 0.5;
					Y0 = _midline[road[0].ahead][1] + 40;
					D_err = getD_eer(X0, Y0);
					float Steer_D_err;
					Steer_D_err = ((kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff)*(_speed + 110) / 220)*1.1;

					k1 = 1;
					*cmdSteer = k1*Steer_D_err;
					steered = 1;

					cout << "��S��" << "\tԤ��� = ��" << X0 << ',' << Y0 << ")\n";
					if (getDistance(X0, Y0)<20)
					{
						X0 = _midline[road[0].ahead + 40][0];
						Y0 = _midline[road[0].ahead + 40][1] + 40;
					}
				}

			}

			if (!steered)//�������֮ǰ�����������¼�ģ��
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
				cout << "=====�����ü�ģ��=====" << endl;
			}
			else//�����֮ǰ��ģ�����������steer����
			{
				*cmdSteer = constrain(-1, 1, *cmdSteer);
			}

			if ((0.5*_width - fabs(_midline[0][0]) < _width / 8))//���紦��
			{
				if (road[0].sign == 0)//ֱ������
				{
					*cmdSteer = constrain(-1, 1, 0.1*_yaw - 0.1 * 2 * _midline[0][0] / _width);
					//cout << "���棡��ֱ�����磡��" << endl;
				}
				else if (road[0].sign == 1 || road[0].sign == 2)
				{
					if (_midline[0][0] * _everycur[0] > 0)//�������
					{
						*cmdSteer = 0.1*_yaw;
						//cout << "���棡��������磡��" << endl;
					}
					else//�������
					{
						*cmdSteer = constrain(-1, 1, 0.3*_yaw - 0.15 * 2 * _midline[0][0] / _width);
						//cout << "���棡��������磡��" << endl;
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
			//cout << "��dcur��" << road[1].dcur << "��cur��" << road[1].dcur / (road[1].ahead - road[1].behind) << endl;

			//״̬����======================
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

// ��·ʶ��
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
	//����·��·����ʶ��ʽ
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
					road[count].r = -1;//��·δ�õ�r
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
					//���road.r����
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
								if (abs(road[count].dcur) > 20)road[count].es = 250;//���ܸ�
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
								if (cur>2.5)road[count].es = 90;//���ܸ�
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
					if (abs(road[count].dcur) > 20)road[count].es = 250;//���ܸ�
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
					if (cur>2.5)road[count].es = 90;//���ܸ�
				}
		}
	}
}




