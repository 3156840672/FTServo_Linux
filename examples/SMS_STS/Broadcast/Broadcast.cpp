#include <iostream>
#include "SCServo.h"

SMS_STS sms_sts;

int main(int argc, char **argv)
{
	if(argc<2){
        std::cout<<"argc error!"<<std::endl;
        return 0;
	}
	std::cout<<"serial:"<<argv[1]<<std::endl;
    if(!sms_sts.begin(115200, argv[1])){
        std::cout<<"Failed to init sms/sts motor!"<<std::endl;
        return 0;
    }
	while(1){
		//鑸垫満(骞挎挱)浠ユ渶楂橀€熷害V=60*0.732=43.92rpm锛屽姞閫熷害A=50*8.7deg/s^2锛岃繍琛岃嚦P1=4095浣嶇疆
		sms_sts.WritePosEx(0xfe, 4095, 2400, 50);
		std::cout<<"pos = "<<4095<<std::endl;
		usleep(((4095-0)*1000/(60*50)+(60*50)*10/(50)+50)*1000);//[(P1-P0)/(V*50)]*1000+[(V*50)/(A*100)]*1000 + 50(璇樊)
  
		//鑸垫満(骞挎挱)浠ユ渶楂橀€熷害V=60*0.732=43.92rpm锛屽姞閫熷害A=50*8.7deg/s^2锛岃繍琛岃嚦P0=0浣嶇疆
		sms_sts.WritePosEx(0xfe, 0, 2400, 50);
		std::cout<<"pos = "<<0<<std::endl;
		usleep(((4095-0)*1000/(60*50)+(60*50)*10/(50)+50)*1000);//[(P1-P0)/(V*50)]*1000+[(V*50)/(A*100)]*1000 + 50(璇樊)
	}
	sms_sts.end();
	return 1;
}

