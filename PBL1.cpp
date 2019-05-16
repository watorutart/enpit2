#include <kernel.h>
#include "kernel_cfg.h"
#include "app.h"
#include "mbed.h"
#include "app_config.h"
#include "Zumo.h"
#include "Milkcocoa.h"
Serial pc(USBTX, USBRX);
Zumo zumo;
DigitalIn sw1(D12);
DigitalOut led_r(LED1);
DigitalOut led_g(LED2);
DigitalOut led_b(LED3);
//add lines
#define Z_DATA_SIZE 10

void plain();
void slope();
void top();
void cliff();
void goal();
bool searchSlope();
bool searchTop();
void led_blink(DigitalOut);
bool searchBlack();
bool searchCliff();
int accel();
int decel();
void drive(int,int);
void setOffset();
void filtering();

//add lines
void flat_Accel_data();
void slope_Accel_data();
float clc_avg(int, int);
float clc_angle();
void disp_angle();

const int BLACK = 600;
const int CLIFF = 1000;
const float ACCELERATION = 1.001;
const float DECELERATION = 0.95;
const int SPEED_MAX = 120;
const int SPEED_MIN = 30;
const int buf_size = 10;
static int speed = 0;
int x_offset,y_offset,z_offset;
short xFilter[buf_size], yFilter[buf_size],zFilter[buf_size];
int x,y,z;

//add line
int z1_data[Z_DATA_SIZE], z2_data[Z_DATA_SIZE];

void task_main(intptr_t exinf){
	int n;

	sw1.mode(PullUp);
	n = sw1;
	Timer timer1;
	while(sw1 != 0){

	}
	dly_tsk(1000);
	setOffset();
	while(1){
		//speedが100になるまで加速
		if(speed >= 100){
			//speedが100以上ならbreak
			break;
		}
		drive(100,1);
	}
	dly_tsk(500);
	//plainの処理
	plain();

	//slopeの処理
	slope();

	//topの処理
	top();

	//cliffの処理
	cliff();

	//goalまでの処理
	goal();

	//角度の表示
	disp_angle();
}

void plain(){
	while(1){
		//前進
		drive(150,1);
		//坂道検出
		if(searchSlope()){
			//坂道を検出したらbreak
			break;
		}
	}
}

void slope(){
	//LED1を点灯
	led_r = 1;
	while(1){
		//前進
		drive(250,1);
		//トップエリアを検出
		if(searchTop()){
			//トップエリアを検出したらbreak
			break;
		}
	}
}

void top(){
	//停止
	drive(0,0);
	//LED1を消灯
	led_r = 0;
	//LED2を3秒間点滅
	led_blink(led_g);
}

void cliff(){
	while(1){
		//前進
		drive(100,3);
		//黒線検出
		if(searchBlack()){
			//黒線を検出したらLED3を点灯しbreak
			led_b = 1;
			break;
		}
	}

	dly_tsk(320);
	while(1){
		//減速しながら前進
		drive(0,2);

		//崖検出
		if(searchCliff()){
			//崖を検出したら停止してLED3を消灯しbreak
			drive(0,0);
			led_b = 0;
			led_r = 1;
			break;
		}
		if(speed == 0){
			break;
		}
	}
	dly_tsk(500);
}

void goal(){
	//崖を脱出するまで後進
	while(searchCliff()){
		drive(-100,2);
	}
	dly_tsk(200);
	while(1){
		//後進
		drive(-100,2);
		//黒線検出(1本目)
		if(searchBlack()){
			//黒線を検出したらbreak
			break;
		}
	}

	dly_tsk(300);
	while(1){
		//後進
		drive(-150,2);
		//黒線検出
		if(searchBlack()){
			//黒線を検出したら停止してbreak
			drive(0,0);
			break;
		}
	}
}

bool searchSlope(){
	//加速度を取得
	filtering();
	if(x > 2500){
		//x方向の加速度2500以上なら坂道発見
		return true;
	}else{
		return false;
	}
}

bool searchTop(){
	//加速度を取得
	filtering();
	if(x < 2500){
		//x方向の加速度2500以下ならトップ発見
		return true;
	}else{
		return false;
	}
}

void led_blink(DigitalOut led){
	int i;
	for(i = 0 ; i<=2 ; i++){
		led = 1;
		dly_tsk(500);
		led = 0;
		dly_tsk(500);
	}
}

bool searchBlack(){
	unsigned int IR_values[6];

	//IRセンサ値を取得
	zumo.readAnalogIrValue(IR_values);
	if(IR_values[0] > BLACK){
		//値が閾値以上なら黒線発見
		return true;
	}else{
		return false;
	}
}

bool searchCliff(){
	unsigned int IR_values[6];

	//IRセンサ値を取得
	zumo.readAnalogIrValue(IR_values);
	if(IR_values[0] > CLIFF){
		//値が閾値以上なら崖発見
		return true;
	}else{
		return false;
	}
}

int accel(int max,int ac){
	int sp;
	bool pn = 0;
	//目標値が負の場合を記憶
	if(max < 0){
		pn = 1;
		max = -max;
	}
	//speedの現在値に加速度を乗じて返す
	//sp = speed * ACCELERATION;
	sp = speed + ac;
	if(sp < SPEED_MIN){
		//乗じた結果が最低速度以下であればスピードを更新して最低速度を返す
		speed = SPEED_MIN;
		if(pn == 1){
			//maxが負なら負の値を返す
			return -SPEED_MIN;
		}else{
			return SPEED_MIN;
		}
	}else if(sp > max){
		//乗じた結果が目標速度以上であればスピードを更新して最高速度を返す
		speed = max;
		if(pn == 1){
			//maxが負なら負の値を返す
			return -max;
		}else{
			return max;
		}
	}else{
		speed = sp;
		if(pn == 1){
			//maxが負なら負の値を返す
			return -sp;
		}else{
			return sp;
		}
	}
}

int decel(int min,int dc){
	int sp;

	//speedの現在値に加速度を乗じて返す
	//sp = speed * DECELERATION;
	sp = speed - dc;
	if(sp < min){
		//乗じた結果が目標値以下であればスピードを更新して目標値を返す
		speed = min;
		return min;
	}else{
		speed = sp;
		return sp;
	}
}

/*
 * modeによって走行モードを変更
 * 0:通常モード
 * 1:加速モード
 * 2:減速モード
 * 3:急加速モード
 * 4:急減速モード
 */
void drive(int sp,int mode){
	//modeによって通常加速減速を切り替える
	switch(mode){
	case 0:
		//通常モードの処理
		zumo.driveTank(sp,sp);
		speed = sp;
		break;
	case 1:
		//加速モードの処理
		sp = accel(sp,4);
		zumo.driveTank(sp+2,sp);
		break;
	case 2:
		//減速モードの処理
		sp = decel(sp,4);
		zumo.driveTank(sp-2,sp);
		break;
	case 3:
		//急加速モードの処理
		sp = accel(sp,20);
		zumo.driveTank(sp+2,sp);
		break;
	case 4:
		//急減速モードの処理
		sp = decel(sp,20);
		zumo.driveTank(sp-2,sp);
		break;
	}
	dly_tsk(50);
}

void setOffset(){
	short tempx,tempy,tempz;
	int i;

	x_offset=y_offset=z_offset=0;
	dly_tsk(100);


	for (i=0;i<buf_size*20;i++){
		zumo.getAcceleration(&tempx,&tempy,&tempz);
		x_offset+=tempx;
		y_offset+=tempy;
		z_offset+=tempz;
		dly_tsk(50);
	}
	x_offset/=(buf_size*20);
	y_offset/=(buf_size*20);
	z_offset/=(buf_size*20);

	dly_tsk(500);
}

void filtering(){
	short cnt,ring;
	cnt=ring=0;
	int i;
	while(cnt <= 20){
		zumo.getAcceleration(&xFilter[ring], &yFilter[ring], &zFilter[ring]);
		cnt++;
		ring=cnt%buf_size;
		x=y=z=0;
		for (i=0;i<buf_size;i++){
			x+=xFilter[i];
			y+=yFilter[i];
			z+=zFilter[i];
		}
		x/=buf_size;
		y/=buf_size;
		z/=buf_size;
		x-=x_offset;
		y-=y_offset;
		z-=z_offset;

	}
}

//add lines
void flat_Accel_data(){
	//平坦な道時の加速度センサのデータをz1_dataに格納
	for(int i=0; i<Z_DATA_SIZE; i++){
		filtering();
		z1_data[i] = z;
	}
}

void slope_Accel_data(){
	//坂道時の加速度センサの加速度センサのデータをz2_dataに格納
	for(int i=0;i<Z_DATA_SIZE;i++){
		filtering();
		z1_data[i] = z;
	}
}

float clc_avg(int size, int data[]){
	//受け取ったdataの値の平均をとる
	int sum = 0;
	for(int i=0;i<size;i++){
		sum += data[i];
	}
	return (float)sum / size;
}

float clc_angle(){
	//角度を計算し返す
	float z1, z2;
	z1 = z2 = 0;
	z1 = clc_avg(Z_DATA_SIZE, z1_data);
	z2 = clc_avg(Z_DATA_SIZE, z2_data);
	return acos(z2 / z1);
}

void disp_angle(){
	int n = 1;
	while(n != 0){
		n = sw1;
		dly_tsk(100);
	}
	pc.printf("角度は %f 度です。\r\n", clc_angle());
}
