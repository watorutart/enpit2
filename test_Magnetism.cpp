#include <kernel.h>
#include "kernel_cfg.h"
#include "app.h"

#include "mbed.h"

#include "app_config.h"

#include "Zumo.h"
#include "Milkcocoa.h"

extern void onpush(MQTT::MessageData& md);
void setOffset();
void filtering();
Serial pc(USBTX, USBRX);
Zumo zumo;

static int speed = 80;
static int steer = 0;
DigitalOut ledR(LED1);

DigitalIn sw1(D12);

const int buf_size = 10;
int x_offset,y_offset,z_offset;
short xFilter[buf_size], yFilter[buf_size],zFilter[buf_size];
int x, y, z;

void onpush(MQTT::MessageData& md)
{
	pc.printf("onpush\n\r");
    MQTT::Message &message = md.message;
    DataElement de = DataElement((char*)message.payload);
	int v = de.getInt("z");
}

void task_main(intptr_t exinf) {
	int n;
	Timer timer1;
	int t;
	sw1.mode(PullUp);
	pc.baud(115200);

	setOffset();

	zumo.driveTank(100, 100);

	timer1.reset();
	timer1.start();
	while(1){
		n = sw1;
		if(n == 0){
			zumo.driveTank(0, 0);
			pc.printf("end\r\n");
			return;
		}

		t = timer1.read_ms();
		if(t > 1000){
			zumo.driveTank(0, 0);
			break;
		}

		filtering();
		pc.printf("%d, %d, %d \r\n", x, y, z);

	}

	pc.printf("loop end\r\n");
}

void setOffset(){
	short tempx,tempy,tempz;
	int i;

	x_offset=y_offset=z_offset=0;
	dly_tsk(100);


	for (i=0;i<buf_size*20;i++){
		zumo.getMagetism(&tempx,&tempy,&tempz);
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
		zumo.getMagetism(&xFilter[ring], &yFilter[ring], &zFilter[ring]);
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
