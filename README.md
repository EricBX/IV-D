# IV-D
The code is about auto-driving algorithm, written in 2017 with my classmates.

Input:
```
static void userDriverGetParam (
    float midline[200][2], //道路中线XY坐标（米）
    float yaw, //偏航角（弧度）
    float yawrate, //角速度（弧度/秒）
    float speed, //车速（公里/小时）
    float acc, //加速度（米/秒2）
    float width, //道路宽度（米）
    int gearbox, //档位（-1~6）
    float rpm); //转速（RPM）
```
Output:
```
static void userDriverSetParam(
    float * cmdAcc, //油门命令[0.0, 1.0]
    float * cmdBrake, //刹车命令[0.0, 1.0]
    float * cmdSteer, //转向命令[0.0, 1.0]
    float * cmdGear); //变速箱档位{-1,1,2,3,4,5,6}
```
Algorithm:(for each control cycle)
1. get the road and car parameters
2. recognize the road
3. predict the type of the road ahead
4. set command parameters according to the above analysis