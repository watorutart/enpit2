#include <kernel.h>
#include "kernel_cfg.h"
#include "app.h"

#include "mbed.h"

#include "app_config.h"

#include "Zumo.h"
#include "Milkcocoa.h"

extern void onpush(MQTT::MessageData& md);
Serial pc(USBTX, USBRX);
Zumo zumo;

static int speed = 80;
static int steer = 0;
DigitalOut ledR(LED1);

DigitalIn sw1(D12);

void onpush(MQTT::MessageData& md)
{
	pc.printf("onpush\n\r");
    MQTT::Message &message = md.message;
    DataElement de = DataElement((char*)message.payload);
	int v = de.getInt("z");
}

/*
 * 受け取った100個のデータの平均を返す関数
 */
short clc_avg(short *data){
	int sum = 0;
	for(int i = 0;i<100;i++){
		sum += data[i];
	}
	pc.printf("sum : %d\r\n", sum);
	return sum/100;
}

/*
 * x, y, zの平滑化フィルタを算出する関数
 * 引数:要素数３のshort型配列
 */
void clc_filter(short *filter){
	short data[3][100];
	short x, y, z;

	int i = 0;

	while(i < 100){
		zumo.getMagetism(&x, &y, &z);

		data[0][i] = x;
		data[1][i] = y;
		data[2][i] = z;
		i++;
	}

	filter[0] = clc_avg(data[0]);
	filter[1] = clc_avg(data[1]);
	filter[2] = clc_avg(data[2]);
}

void task_main(intptr_t exinf) {
	int n;
	sw1.mode(PullUp);
	pc.baud(115200);

	short x, y, z;
	short avg[3];

	clc_filter(avg);

	pc.printf("avg: %d %d %d \r\n", avg[0], avg[1], avg[2]);
}
