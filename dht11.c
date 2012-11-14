#include "dht11.h"

uint16_t read_cycle(uint16_t cur_tics, uint8_t neg_tic){
	uint16_t cnt_tics;
 	if (cur_tics < MAX_TICS) cnt_tics = 0;
	if (neg_tic){
		while (!GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3)&&(cnt_tics<MAX_TICS)){
			cnt_tics++;
		}
	}
	else {
		while (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3)&&(cnt_tics<MAX_TICS)){
			cnt_tics++;
		}
	}
 	return cnt_tics;
}

uint8_t read_DHT11(uint8_t *buf){
	uint16_t dt[42];
	uint16_t cnt;
	uint8_t i, check_sum, tmp; 
	
	//reset DHT11
	Delay(400);
 	GPIO_LOW(GPIOA,GPIO_Pin_2);
	Delay(18);
 	GPIO_HIGH(GPIOA,GPIO_Pin_2);
	
  //start reading	
 	cnt = 0; 
	for(i=0;i<83 && cnt<MAX_TICS;i++){
		if (i & 1){
			cnt = read_cycle(cnt, 1);
		}
		else {
			cnt = read_cycle(cnt, 0);
			dt[i/2]= cnt;
		}
	}
	
 	//relese line
	GPIO_HIGH(GPIOA,GPIO_Pin_2);
	
	if (cnt>=MAX_TICS) return DHT11_NO_CONN;
	
	//convert data
 	for(i=2;i<42;i++){
		tmp <<= 1;
  	if (dt[i]>20) {
			tmp++;			
 		}
		if (!((i-1)%8) && (i>2)) {
			*buf = tmp;
			buf++;
		}
 	}
	
	//calculating checksum
	buf -= 5;
	check_sum = 0;
 	for(i=0;i<4;i++){
		check_sum += *buf;
		buf++;
	}
	
	if (*buf != check_sum) return DHT11_CS_ERROR;
				
	return DHT11_OK;	
	//return check_sum;
}

