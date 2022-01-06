#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include "debuglog.h"
#include "bb_ctrl_internal.h"
#include "bb_regs.h"
#include "bb_sky_ctrl.h"
#include "log10.h"
#include "cfg_parser.h"
#include "rf_if.h"
#include "systicks.h"

#define SWEEP_NOISE_ORDER 60
#define SWEEP_NOISE_CHG_THD 5
#define CHG_PATTEN_TIME_GAP 64
#define SWEEP_NOISE_SELECT_DIFF_THD 5





void __attribute__ ((section(".h264"))) sky_startSweep(ENUM_RF_BAND band)
{
	
	 if (context.st_bandMcsOpt.e_bandsupport == RF_2G)
    {
        context.rf_info.sweepBand[0] = RF_2G;
		context.rf_info.f2g_freqsize = BB_GetSkySweepFrqNum(RF_2G);
        context.rf_info.bandCnt = 1;
		context.rf_info.rc_avr_sweep_result_size =  context.rf_info.f2g_freqsize;
    }
	 else if (context.st_bandMcsOpt.e_bandsupport == RF_5G)
    {
        context.rf_info.sweepBand[1] = RF_5G;
        context.rf_info.f5g_freqsize = BB_GetSkySweepFrqNum(RF_5G);
        context.rf_info.bandCnt = 1;
		context.rf_info.rc_avr_sweep_result_size = context.rf_info.f5g_freqsize;
    }
    else if (context.st_bandMcsOpt.e_bandsupport == RF_2G_5G)
    {
        context.rf_info.sweepBand[0] = RF_2G;
        context.rf_info.sweepBand[1] = RF_5G;
		context.rf_info.f2g_freqsize = BB_GetSkySweepFrqNum(RF_2G);
        context.rf_info.f5g_freqsize = BB_GetSkySweepFrqNum(RF_5G);
        context.rf_info.bandCnt = 2;
		context.rf_info.rc_avr_sweep_result_size =  context.rf_info.f2g_freqsize+context.rf_info.f5g_freqsize;
    }
    else
    {
        context.rf_info.bandCnt      = 1;
        context.rf_info.sweepBand[0]    = context.st_bandMcsOpt.e_bandsupport;
        context.rf_info.f2g_freqsize = BB_GetRcFrqNum(context.st_bandMcsOpt.e_bandsupport);
		context.rf_info.rc_avr_sweep_result_size = context.rf_info.f2g_freqsize;
    }

    //sweep Band, channel
    context.rf_info.e_bw = BW_10M;
    context.rf_info.curBandIdx = 0;
	context.rf_info.curSweepCh=0;
    context.rf_info.curRowCnt  = 0;
	context.rf_info.currc_statistics_Row=0;
	context.rf_info.sweep_finished=1;
	context.rf_info.fine_sweep_size=8;
	context.rf_info.fine_sweep_id=0;
	context.rf_info.rc_ch_working_patten_len=SKY_PATTEN_SIZE_2G;
	//context.rf_info.rc_avr_sweep_result_size
	reset_table_for_2g();
	//reset_table_for_5g();

    BB_set_SweepFrq(context.rf_info.sweepBand[context.rf_info.curBandIdx], context.rf_info.e_bw, context.rf_info.curSweepCh);
    context.u_bandSwitchParam = (UNION_BandSwitchParm *)CFGBIN_GetNodeAndData((STRU_cfgBin *)SRAM_CONFIGURE_MEMORY_ST_ADDR, RF_SKY_BAND_SWITCH_CFG_ID, NULL);

    DLOG_Critical("%d %d %d %d 0x%x",context.rf_info.f2g_freqsize, context.rf_info.f5g_freqsize, context.rf_info.bandCnt, context.st_bandMcsOpt.e_bandsupport, context.u_bandSwitchParam);
}

void reset_table_for_2g(){
	int i=0,j=0;
	for(i=0;i<context.rf_info.f2g_freqsize;i++){
			context.rf_info.prelist[i].id=i;context.rf_info.prelist[i].value=0;
			context.rf_info.sweep_pwr_avrg_value[i].id=i;context.rf_info.sweep_pwr_avrg_value[i].value=0;
			context.rf_info.sweep_pwr_fluct_value[i].id=i;context.rf_info.sweep_pwr_fluct_value[i].value=0;
			context.rf_info.work_rc_error_value[i].id=i;context.rf_info.work_rc_error_value[i].value=0;
			context.rf_info.work_snr_avrg_value[i].id=i;context.rf_info.work_snr_avrg_value[i].value=0;
			context.rf_info.work_snr_fluct_value[i].id=i;context.rf_info.work_snr_fluct_value[i].value=0;
			context.rf_info.error_ch_record[i]=0xff;
			for(j=0;j<SWEEP_FREQ_BLOCK_ROWS;j++){
				context.rf_info.sweep_pwr_table[j][i].id=i;context.rf_info.sweep_pwr_table[j][i].value=0;
				
				context.rf_info.work_rc_unlock_table[j][i].id=i;context.rf_info.work_rc_unlock_table[j][i].value=0;
				context.rf_info.work_snr_table[j][i].id=i;context.rf_info.work_snr_table[j][i].value=0;
			}
			
		}

}

static int math_multi(int a,int b){
	int i=0;
	int r = 0;
	for(i=0;i<b;i++) r += a;

	return r;
}
static void log_printf_data_int(uint32_t *str,int i){
	#if 1
	DLOG_Critical("[%d] %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",i,
					str[0],str[1],str[2],str[3],str[4],str[5],str[6],str[7],str[8],str[9],
					str[10],str[11],str[12],str[13],str[14],str[15],str[16],str[17],str[18],str[19],
					str[20],str[21],str[22],str[23],str[24],str[25],str[26],str[27],str[28],str[29],
					str[30],str[31],str[32],str[33],str[34],str[35]);
	#endif
}

static void log_printf_40_data(uint32_t *str,int i){
#if 1
DLOG_Critical("[%d] type=[%d] %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
					i,
					str[0],str[1],str[2],str[3],str[4],str[5],str[6],str[7],str[8],str[9],
					str[10],str[11],str[12],str[13],str[14],str[15],str[16],str[17],str[18],str[19],
					str[20],str[21],str[22],str[23],str[24],str[25],str[26],str[27],str[28],str[29],
					str[30],str[31],str[32],str[33],str[34],str[35],str[36]);
#endif
	uint8_t data_buf[37]={0};
	int j=0;
	for(j=0;j<37;j++) data_buf[j]=abs(str[j]);
	BB_Session0SendMsg(DT_NUM_SKY_SWEEP_NOISE, data_buf,37);
}

void  sky_CalcAverage_rc_ch_error_cnt(uint8_t ch)
{
    uint8_t row=0;
	uint8_t value=0;
	context.rf_info.work_rc_error_value[ch].value = 0;
    for (row = 0; row < SWEEP_FREQ_BLOCK_ROWS; row++){
    	value = context.rf_info.work_rc_unlock_table[row][ch].value;
        context.rf_info.work_rc_error_value[ch].value += value;
    }
}

static void reset_working_times_ch_statistics(uint8_t ch)
{
	uint8_t row=0;
	context.rf_info.i32_working_times[ch] = 0;
   
}

void sky_statistics_rc_ch_error(uint8_t locked)
{
	uint8_t ch = context.sky_rc_channel;
    context.rf_info.work_rc_unlock_table[context.rf_info.currc_statistics_Row][ch].value = (locked==1) ? 0 : 1;
	context.rf_info.currc_statistics_Row++;
	if(context.rf_info.currc_statistics_Row >=SWEEP_FREQ_BLOCK_ROWS) 
	{
		context.rf_info.currc_statistics_Row=0;
	}

	sky_CalcAverage_rc_ch_error_cnt(ch);
}

void  sky_CalcAverage_rc_ch_snr(uint8_t ch)
{
    uint8_t row=0,total=0;
	int temp=0;
	context.rf_info.work_snr_avrg_value[ch].value = 0;
	context.rf_info.work_snr_fluct_value[ch].value=0;
    for (row = 0; row < SWEEP_FREQ_BLOCK_ROWS; row++)
    {
    	temp += context.rf_info.work_snr_table[row][ch].value;
		if(context.rf_info.work_snr_table[row][ch].value !=0) total++;
    }
	if(total!=0)
		temp /= total;
		context.rf_info.work_snr_avrg_value[ch].value=temp;

	for (row = 0; row < total-1; row++){
		
		  context.rf_info.work_snr_fluct_value[ch].value +=abs(context.rf_info.work_snr_table[row+1][ch].value-context.rf_info.work_snr_table[row][ch].value);
	   }

}


void sky_statistics_rc_snr(uint8_t locked)
{
	static int resetall=0;
	int i=0;
	uint8_t ch = context.sky_rc_channel;
	uint8_t row = context.sky_rc_record_row;
	if(row==SWEEP_FREQ_BLOCK_ROWS-1){
		for(i=0;i<SWEEP_FREQ_BLOCK_ROWS-1;i++){
			context.rf_info.work_snr_table[i][ch].value =context.rf_info.work_snr_table[i+1][ch].value ;
		}
	}
	if(locked)
    	context.rf_info.work_snr_table[row][ch].value = get_10log10(sky_get_rc_snr()/64);
	else 
		context.rf_info.work_snr_table[row][ch].value = 0;

	sky_CalcAverage_rc_ch_snr(ch);
	context.rf_info.i32_working_times[ch]++; 
	if(row<SWEEP_FREQ_BLOCK_ROWS-1)
	{
		context.sky_rc_record_row++;
	}
}

void sky_SetNextSweepCh(void)
{
	//if need to change patten,need to sweep the selections more 
	if(context.rf_info.lock_sweep)
	{
		context.rf_info.fine_sweep_id ++;
		if(context.rf_info.fine_sweep_id==context.rf_info.fine_sweep_size){
			context.rf_info.fine_sweep_id = 0;
		}
		BB_set_skySweepfrq(context.rf_info.sweepBand[context.rf_info.curBandIdx], context.rf_info.pre_selection_list[context.rf_info.fine_sweep_id].id);
	}
	else
	{
	    context.rf_info.curSweepCh ++;
		uint8_t num = (context.rf_info.curBandIdx==0) ? context.rf_info.f2g_freqsize:context.rf_info.f5g_freqsize;
	    if (context.rf_info.curSweepCh == num)  //last channel
	    {
	        if (context.rf_info.bandCnt == 2)        //switch band
	        {
	            context.rf_info.curBandIdx = (context.rf_info.curBandIdx + 1) % 2;
	        }
	        context.rf_info.curSweepCh = 0;
	    }
	 	BB_set_skySweepfrq(context.rf_info.sweepBand[context.rf_info.curBandIdx], context.rf_info.curSweepCh);
	}
}

int sky_get_rc_current_sweep_ch(){
	
	return context.rf_info.curSweepCh;
}

int sky_get_rc_current_sweep_row(){
	
	return context.rf_info.curRowCnt;
}


int sky_lna_check_sweep_power(int32_t spower_data)
{   
    static uint32_t   swp_lna_switch_cnt = 0;
    static uint32_t   swp_lna_switch_all_cnt = 0;
    static uint8_t    swp_recyles_cnt = 0,swp_watch_window_size = 0;

    if(context.rf_info.sweepBand[context.rf_info.curBandIdx] == RF_2G)
    {
        swp_watch_window_size++;
        if(spower_data > SKY_SWEEP_LNA_SWITCH_THRESHOLD)
        {
            swp_lna_switch_cnt += 1;
        }
        
        //if(swp_lna_switch_cnt > SWEEP_LNA_SWITCH_THRESHOLD_nper100)
       // {
       //     context.swp_bypass = 1; /////////3/100
       // }
        
        if(swp_watch_window_size > 100)
        {
            swp_watch_window_size = 0;
            swp_recyles_cnt++;
            swp_lna_switch_all_cnt += swp_lna_switch_cnt;
            DLOG_Info("lna=%d,swp=%d,%d-%d %d",context.lna_status,context.swp_bypass,swp_lna_switch_cnt,swp_lna_switch_all_cnt,swp_recyles_cnt);
            swp_lna_switch_cnt = 0;
        }

        if(swp_recyles_cnt > 9)
        {
            swp_recyles_cnt = 0;
            
            if(swp_lna_switch_all_cnt >= SWEEP_LNA_SWITCH_THRESHOLD_nper1000)
            {
                context.swp_bypass = 1;//25/1000,error, 
            }
            else if(swp_lna_switch_all_cnt == 0)
            {
                context.swp_bypass = 2;
            }
            
            if(swp_lna_switch_all_cnt > 0)
            {
                if(context.swp_bypass == 2)
                {
                    context.swp_bypass = 0;
                }
            }

            swp_lna_switch_all_cnt= 0;
        }

    }

    //DLOG_Warning("sw-b %d wk-b %d sw-c %d p %d",context.rf_info.e_prevSweepBand,context.e_curBand,ch,spower_data);
}

static uint32_t sweep_pwr_table1[3][50]={0};
static uint32_t sweep_pwr_table2[3][50]={0};

static void sky_print_sweep_pwr_table()
{
	int i=0,j=0; 
	DLOG_Critical("sweep pwr table");
	for(i=0;i<SWEEP_FREQ_BLOCK_ROWS;i++){
		for(j=0;j<context.rf_info.f2g_freqsize;j++){
			if(i<3){
				sweep_pwr_table1[i][j+1]=context.rf_info.sweep_pwr_table[i][j].value;
				
			}
			else if(i<6){
				sweep_pwr_table2[i-3][j+1]=context.rf_info.sweep_pwr_table[i][j].value;
			}
			sweep_pwr_table1[i][0]=0x00;
			sweep_pwr_table2[i-3][0]=0x00;
		}
 	}
}

static void tx_sweep_pwr_table1(){
	int i=0,j=0; 
	uint32_t str[50]={0};
	
	for(i=0;i<3;i++){
		for(j=0;j<context.rf_info.f2g_freqsize+1;j++)str[j]=sweep_pwr_table1[i][j];
		log_printf_40_data(str, i);
 	}
}
static void tx_sweep_pwr_table2(){
	int i=0,j=0; 
	uint32_t str[50]={0};
	for(i=0;i<3;i++){
		for(j=0;j<context.rf_info.f2g_freqsize+1;j++)str[j]=sweep_pwr_table2[i][j];
		log_printf_40_data(str, i+3);
 	}
}

static void sky_print_sweep_pwr_result(){
	int i=0,j=0; 
	uint32_t str[50]={0};

	//DLOG_Critical("sweep pwr avrg");
	for(j=0;j<context.rf_info.f2g_freqsize;j++)str[j+1]=context.rf_info.sweep_pwr_avrg_value[j].value;
	str[0]=0x01;//pwr_avrg
	log_printf_40_data(str, 0);
	
	//DLOG_Critical("sweep pwr fluct");
	for(j=0;j<context.rf_info.f2g_freqsize;j++)str[j+1]=context.rf_info.sweep_pwr_fluct_value[j].value;
	str[0]=0x02;//pwr_fluct
	log_printf_40_data(str, 0);

}

static void sky_print_sweep_snr_result(){
	int i=0,j=0; 
	uint32_t str[50]={0};

	//DLOG_Critical("snr avrg");
	for(j=0;j<context.rf_info.f2g_freqsize;j++)str[j+1]=context.rf_info.work_snr_avrg_value[j].value;
	str[0]=0x05;//snr_avrg
	log_printf_40_data(str, 0);

	
	//DLOG_Critical("snr fluct");
	for(j=0;j<context.rf_info.f2g_freqsize;j++)str[j+1]=context.rf_info.work_snr_fluct_value[j].value;
	str[0]=0x06;//snr_fluct
	log_printf_40_data(str, 0); 

}

static void sky_print_sweep_snr_table()
{
	int i=0,j=0; 
	uint32_t str[50]={0};
	//DLOG_Critical("snr table");
	for(i=0;i<SWEEP_FREQ_BLOCK_ROWS;i++){
		for(j=0;j<context.rf_info.f2g_freqsize;j++)str[j+1]=context.rf_info.work_snr_table[i][j].value;
		str[0]=0x04;//snr_table
		log_printf_40_data(str, i);
	}
	
}

static void sky_print_sweep_error_table()
{
	int i=0,j=0; 
	uint32_t str[50]={0};
	//DLOG_Critical("sweep error result");
	for(j=0;j<context.rf_info.f2g_freqsize;j++)str[j+1]=context.rf_info.work_rc_error_value[j].value;
	str[0]=0x03;//pwr_fluct
	log_printf_40_data(str, 0);	
}

static void sky_print_working_times_statistics()
{
	int j=0; 
	uint32_t str[50]={0};
	DLOG_Critical("sweep working_times result");
	for(j=0;j<context.rf_info.f2g_freqsize;j++)str[j]=context.rf_info.i32_working_times[j];
	log_printf_data_int(str, 0);
}

static void sky_print_now_working_channels(){

	int j=0; 
	uint32_t str[50]={0};
	DLOG_Critical("working_patten and channels");
	for(j=0;j<context.rf_info.rc_ch_working_patten_size;j++)str[context.rf_info.rc_ch_working_patten[j]]=BB_GetRcFrqByCh(context.rf_info.rc_ch_working_patten[j]);
	log_printf_data_int(str, 0);
}
void GetSweepCh_finesweep(uint8_t u8_bandidx, uint8_t u8_ch,signed char data){
	int i=0,j=0; 

	if (u8_bandidx == 0)
	   {
		   context.rf_info.sweep_pwr_table[context.rf_info.fine_sweep_row][u8_ch].value = data;
	   }
   else
	   {
		   context.rf_info.sweep_pwr_table[context.rf_info.fine_sweep_row][u8_ch].value = data;
		   if (u8_ch > 2)
		   {	
			   context.rf_info.sweep_pwr_table[context.rf_info.fine_sweep_row][u8_ch].value = data - 3;
		   }
		   else
		   {
			   context.rf_info.sweep_pwr_table[context.rf_info.fine_sweep_row][u8_ch].value = data + 1;
		   }
	   }
   context.rf_info.fine_sweep_row++;
   if((context.rf_info.fine_sweep_id+1)==context.rf_info.fine_sweep_size)
   {
		if(context.rf_info.fine_sweep_row >= SWEEP_FREQ_BLOCK_ROWS)
		{
		   context.rf_info.sweep_finished=1;
		   context.rf_info.fine_sweep_row=0;
		   DLOG_Critical("swwep finished");
		}
   }

}

void GetSweepCh_normalsweep(uint8_t u8_bandidx, uint8_t u8_ch,signed char data,ENUM_BB_MODE mode)
{
	int i=0,j=0; 
	static int tx_sweep=0;
	static int k=0;
	 if(context.rf_info.curRowCnt==SWEEP_FREQ_BLOCK_ROWS-1)
	   {
		   for(i=0;i<SWEEP_FREQ_BLOCK_ROWS-1;i++)
		   {
			   context.rf_info.sweep_pwr_table[i][u8_ch].value = context.rf_info.sweep_pwr_table[i+1][u8_ch].value;
		   }
	   }
	 if (u8_bandidx == 0)
	   {
		   context.rf_info.sweep_pwr_table[context.rf_info.curRowCnt][u8_ch].value = data;
	   }
     else
	   {
		   context.rf_info.sweep_pwr_table[context.rf_info.curRowCnt][u8_ch].value = data;
		   if (u8_ch > 2)
		   {	
			   context.rf_info.sweep_pwr_table[context.rf_info.curRowCnt][u8_ch].value = data - 3;
		   }
		   else
		   {
			   context.rf_info.sweep_pwr_table[context.rf_info.curRowCnt][u8_ch].value = data + 1;
		   }
	   }
	  
	   int last =0;
	   if(context.rf_info.bandCnt==2) last = (u8_bandidx == 1 && (u8_ch + 1) == context.rf_info.f5g_freqsize);
	   else last = ((u8_ch + 1) == context.rf_info.f2g_freqsize);
	   if (last)
	   {
	   	    if (context.rf_info.curRowCnt < SWEEP_FREQ_BLOCK_ROWS-1) 
			{
				context.rf_info.curRowCnt++;
	   	    }

			context.rf_info.sweep_cycle++;
		  	//next cycle
		   if (context.rf_info.sweep_cycle >= SWEEP_FREQ_BLOCK_ROWS)
		   {
			   context.rf_info.sweep_cycle = 0;
			   context.rf_info.isFull	  = 1;
			   sky_print_sweep_pwr_table();
			   tx_sweep=1;
		   }
		   else
		   {
			   context.rf_info.isFull	  = 0;
		   }
		   if(mode==BB_GRD_MODE) return ;
		   if(tx_sweep)
		   {
			   #if 1
				   k++;
				   if(k==1){
					   tx_sweep_pwr_table1();
				   }else if(k==2){
					   tx_sweep_pwr_table2();
					   
				   }else if(k==3){
					   sky_print_sweep_pwr_result();
					   
				   }else if(k==4){
					   sky_print_sweep_snr_table();
				   }
				   else if(k==5){
					   sky_print_sweep_snr_result();
				   }else if(k==6){
					   sky_print_sweep_error_table();
					   sky_print_working_times_statistics();
					   sky_print_now_working_channels();
					   k=0;
					   tx_sweep = 0;
				   }
			   #endif
			   
		  }
	   }

}

/*
 * return 0: Fail 
 *        1: OK
*/
uint8_t __attribute__ ((section(".h264")))sky_SweepCh(uint8_t u8_bandidx, uint8_t u8_ch)
{
	int i=0,j=0; 
	
	static int tx_sweep=0;
    int32_t power = BB_SweepEnergy();
	signed char data = int2char(power);
    if (data == 0){
        return 0;
    }
    sky_lna_check_sweep_power(data);

	if( context.rf_info.lock_sweep){
		GetSweepCh_finesweep(u8_bandidx,u8_ch,data);
	}
	else{
		GetSweepCh_normalsweep(u8_bandidx,u8_ch,data,BB_SKY_MODE);
	}
	return 1;
}

void __attribute__ ((section(".h264"))) sky_GetSweepNoise(int16_t *ptr_noise_power, uint32_t max)
{
    uint8_t col;
    uint8_t i;
    int16_t value;

    for(col = 0; col <context.rf_info.f2g_freqsize; col++)
    {
        if(col >= max)
        {
            return;
        }
        value = (int16_t)(context.rf_info.sweep_pwr_avrg_value[col].value);
        for(i = 0; i < 8; i++)
        {
            ptr_noise_power[(col * 8) + i] = value;
        }
    }

/*
    for(col = 0; col < context.rf_info.f5g_freqsize; col++)
    {
        if((col + context.rf_info.f2g_freqsize) >= max)
        {
            return;
        }
        value = (int16_t)(context.rf_info.f5g_sweep_pwr_avrg_value[col].value);
        for(i = 0; i < 8; i++)
        {
            ptr_noise_power[(col + context.rf_info.f2g_freqsize) * 8 + i] = value;
        }
    }
  */
  
}




/*
 * check rc status & sweep result if band switch is necessary
*/
#if 0
ENUM_BAND_SWITCH_OPTION __attribute__ ((section(".h264"))) sky_CheckBandSwitch(void)
{
    ENUM_BAND_SWITCH_OPTION e_opt = SKY_EQUAL_2G_5G;

    //check rc lock status to judge if band switch is necessary, and already get the signal block state
    uint8_t rc = sky_CheckRcJudgeBandSwitch();

    //check sweep result
    uint8_t cur_band = (context.e_curBand == RF_5G);


    if (rc == 1)
    {
        //check sweep result if band switch is necessary
        if (cur_band == 0 && st_skySweep.u8_band1BetterCnt >= sizeof(st_skySweep.band_sel))
        {
            e_opt = SKY_MUST_SWITCH_5G;
        }
        else if (cur_band == 1 && st_skySweep.u8_band0BetterCnt >= sizeof(st_skySweep.band_sel))
        {
            e_opt = SKY_MUST_SWITCH_2G;
        }
    }

    DLOG_Info("%d band(0:1)-(%d:%d) result=%d", rc, st_skySweep.u8_band0BetterCnt, st_skySweep.u8_band1BetterCnt, e_opt);

    return e_opt;
}
#endif

void __attribute__ ((section(".h264"))) sky_SweepProcess(void)
{
    //get current sweep energy
    uint8_t ret=0;
	if(context.rf_info.lock_sweep)
	{
		ret = sky_SweepCh(context.rf_info.curBandIdx,  context.rf_info.pre_selection_list[context.rf_info.fine_sweep_id].id);
		 CalcAverageSweepPower(context.rf_info.pre_selection_list[context.rf_info.fine_sweep_id].id);
	}
	else
	{
    	ret = sky_SweepCh(context.rf_info.curBandIdx, context.rf_info.curSweepCh);
		 CalcAverageSweepPower(context.rf_info.curSweepCh);
	}
    //set next sweep energy
    if (ret > 0)
    {
        sky_SetNextSweepCh();
    }
    //compare band
    if(context.rf_info.bandCnt == 1)return;

#if 0
    if (ret > 0 && st_skySweep.u16_cyclelCnt >= st_skySweep.u16_bandCheckCycleCnt)
    {
        uint8_t data = BB_ReadRegMask(PAGE2, GRD_SKY_IT_CH_SYNC, GRD_SKY_BLOCK_MODE_MASK);
        ENUM_RF_SIGNAL_BLOCK e_signal = (data >> 6);

        st_skySweep.u16_cyclelCnt = 0;

        //do band selection
       // sky_CalcAverageSweepPower();

        //dont know the result if not get signal mode(block, unblock)
        if (e_signal == RF_SIGNAL_UNKNOWN_MODE)
        {
            DLOG_Info("UNKNOWN_MODE %x", data);
            return;
        }

        st_skySweep.band_sel[st_skySweep.u8_bandSelCnt] = sky_GetBetterBand(e_signal == RF_SIGNAL_BLOCK_MODE);
        st_skySweep.u8_bandSelCnt ++;

        if (st_skySweep.u8_bandSelCnt == sizeof(st_skySweep.band_sel)/sizeof(st_skySweep.band_sel[0]))
        {
            uint8_t m;
            st_skySweep.u8_bandSelCnt = 0;

            st_skySweep.u8_band0BetterCnt = 0;
            st_skySweep.u8_band1BetterCnt = 0;

            //get band1 better count
            for (m = 0; m < sizeof(st_skySweep.band_sel); m++)
            {
                st_skySweep.u8_band0BetterCnt += (st_skySweep.band_sel[m] == BETTER_2G);
                st_skySweep.u8_band1BetterCnt += (st_skySweep.band_sel[m] == BETTER_5G);
            }
        }
    }
   #endif
}
/*
void __attribute__ ((section(".h264"))) sky_requestRfBandSwitch(ENUM_RF_BAND e_band)
{
    STRU_BandChange change =
    {
        .flag_bandchange = 1,
        .e_toBand        = e_band,
    };
    
    BB_Session0SendMsg(DT_NUM_RF_BAND_CHANGE, (uint8_t *)&change, sizeof(STRU_BandChange));
}
*/
int __attribute__ ((section(".h264"))) sky_GetAverSweepResult(uint8_t ch)
{
	if(context.rf_info.curBandIdx==0){
		return (context.rf_info.sweep_pwr_avrg_value[ch].value);
	}
	else if(context.rf_info.curBandIdx==1){
    	return (context.rf_info.sweep_pwr_avrg_value[ch].value);
	}
	else 
		return 0;
}

int __attribute__ ((section(".h264"))) sky_Get_statistic_rc_error_Result(uint8_t ch)
{
	if(context.rf_info.curBandIdx==0)
	{
		if(ch < context.rf_info.f2g_freqsize)
		return (context.rf_info.work_rc_error_value[ch].value);
	}
	return 0;
}

int sky_get_rc_total_channel()
{
	if(context.rf_info.curBandIdx==0)
		return (context.rf_info.f2g_freqsize);
	else if(context.rf_info.curBandIdx==1)
    	return (context.rf_info.f5g_freqsize);
	else 
		return 0;
}



static uint8_t is_same_patten(){
	int i=0;
	STRU_RF_DATA list[MAX_RC_FRQ_SIZE]={0};
	STRU_RF_DATA listcomp_wk[MAX_RC_FRQ_SIZE]={0};
	STRU_RF_DATA listcomp_nw[MAX_RC_FRQ_SIZE]={0};

	for(i=0;i<context.rf_info.rc_ch_working_patten_size;i++){
			
		list[i].value=context.rf_info.sort_result_list[i].value;
		list[i].id=context.rf_info.sort_result_list[i].id;
	}
	selectionSortBy(listcomp_nw,context.rf_info.rc_ch_working_patten_size,list,0);
	
	for(i=0;i<context.rf_info.rc_ch_working_patten_size;i++){
			
		list[i].value=context.rf_info.prelist[i].value;
		list[i].id=context.rf_info.prelist[i].id;
	}
	selectionSortBy(listcomp_wk,context.rf_info.rc_ch_working_patten_size,list,0);

	for(i=0;i<context.rf_info.rc_ch_working_patten_size;i++){
		if(listcomp_nw[i].id != listcomp_wk[i].id){
			return 0;
		}
	}

	
	return 1;
}

static uint8_t corse_check_sweep_noise(uint8_t mustchg){
	STRU_RF_DATA list[MAX_RC_FRQ_SIZE]={0};
	STRU_RF_DATA listr[MAX_RC_FRQ_SIZE]={0};
	STRU_RF_DATA listcomp[MAX_RC_FRQ_SIZE]={0};
	uint8_t i=0,j=0;
	int sweep_noise=0;
	int sweep_noise_fluct=0;
	
	
	//sort by value,find the low to high list, sort all sweep channel
	for(i=0;i<context.rf_info.rc_avr_sweep_result_size;i++){
		sweep_noise = context.rf_info.sweep_pwr_avrg_value[i].value * SWEEP_NOISE_ORDER;
		sweep_noise_fluct = context.rf_info.sweep_pwr_fluct_value[i].value *(100-SWEEP_NOISE_ORDER);
		list[i].value=(sweep_noise + sweep_noise_fluct)/100;
		list[i].id=context.rf_info.sweep_pwr_avrg_value[i].id;
	}
	selectionSortBy(listr,context.rf_info.rc_avr_sweep_result_size,list,1);
	for(i=0;i<context.rf_info.rc_avr_sweep_result_size;i++){
		context.rf_info.sort_result_list[i].id=listr[i].id;
		context.rf_info.sort_result_list[i].value=listr[i].value;
	}
	if(mustchg){

		for(j=0;j<context.rf_info.fine_sweep_size;j++){
				context.rf_info.pre_selection_list[j].id=listr[j].id;
				context.rf_info.pre_selection_list[j].value=listr[j].value;
				//DLOG_Critical("[%d] %d %d %d",j,listr[j].id,BB_GetRcFrqByCh(listr[j].id),listr[j].value);
			}
		return 1;
	}
	//sort the current working patten
	for(i=0;i<context.rf_info.rc_ch_working_patten_len;i++){
		sweep_noise = context.rf_info.sweep_pwr_avrg_value[context.rf_info.prelist[i].id].value * SWEEP_NOISE_ORDER;
		sweep_noise_fluct = context.rf_info.sweep_pwr_fluct_value[context.rf_info.prelist[i].id].value *(100-SWEEP_NOISE_ORDER);
		list[i].value=( sweep_noise + sweep_noise_fluct)/100;
		list[i].id=context.rf_info.sweep_pwr_avrg_value[context.rf_info.prelist[i].id].id;
	}
	selectionSortBy(listcomp,context.rf_info.rc_ch_working_patten_len,list,1);

	#if 0
	static int cnt = 0;
	cnt++;
	if(cnt>100){
		for(i=0;i<context.rf_info.rc_ch_working_patten_len;i++){
				DLOG_Critical("%d,wkc=%d,%d,%d,swc=%d,%d,%d",
				i,
				listcomp[i].id,
				BB_GetRcFrqByCh(listcomp[i].id),
				listcomp[i].value,
				listr[i].id,
				BB_GetRcFrqByCh(listr[i].id),
				listr[i].value
			);
		}
		cnt =0;
	}
	#endif

	//check the newsweep list ,if same to the working patten list,not need to change patten
	if(is_same_patten()==1){ return 0;}
	//check noise if meet to jump
	for(i=0;i<context.rf_info.rc_ch_working_patten_size;i++)
	{
		if((listcomp[i].value - listr[i].value) > SWEEP_NOISE_CHG_THD)
		{
			for(j=0;j<context.rf_info.fine_sweep_size;j++){
				context.rf_info.pre_selection_list[j].id=listr[j].id;
				context.rf_info.pre_selection_list[j].value=listr[j].value;
				//DLOG_Info("[%d] %d %d %d",j,listr[j].id,BB_GetRcFrqByCh(listr[j].id),listr[j].value);
			}
			
			#if 0
			for(i=0;i<context.rf_info.rc_ch_working_patten_len;i++)
			{
				DLOG_Critical("%d,wkc=%d,%d,%d,swc=%d,%d,%d",
					i,
					listcomp[i].id,
					BB_GetRcFrqByCh(listcomp[i].id),
					listcomp[i].value,
					listr[i].id,
					BB_GetRcFrqByCh(listr[i].id),
					listr[i].value);
			}
			#endif
			return 1;
		}
	}
	
	return 0;
}
static uint8_t check_working_channel_error(){
	uint8_t i=0,j=0;
	uint8_t ch=0;
	uint8_t meet=0;
	for(i=0;i<context.rf_info.rc_ch_working_patten_size;i++){
		ch = context.rf_info.rc_ch_working_patten[i];
		if(context.rf_info.work_rc_error_value[ch].value > 2 ){
			context.rf_info.error_ch_record[j]=ch;
			j++;
			meet = 1;
		}
	}
	return meet;
}

static uint8_t find_ch_in_error_list(uint8_t ch){
	int i=0;
	int have=0;
	for(i=0;i<MAX_RC_FRQ_SIZE;i++){
		if(context.rf_info.error_ch_record[i]==0xff) 	break;
		else
		{
			if(context.rf_info.error_ch_record[i]==ch)
			{
				have=1;
				break;
			}
		}
	}
	return have;
}
static void reset_error_list_record(){
	int i=0;
	for(i=0;i<MAX_RC_FRQ_SIZE;i++){
		if(context.rf_info.error_ch_record[i]==0xff) break;		
		else
		{
			context.rf_info.error_ch_record[i]=0xff;
		}
	}	
}
static uint8_t check_working_channel_snr(){

	return 0;
}

static void find_best_patten()
{
	int i=0;

	STRU_RF_DATA list[MAX_RC_FRQ_SIZE]={0};
	STRU_RF_DATA listr[MAX_RC_FRQ_SIZE]={0};
	STRU_RF_DATA listcomp[MAX_RC_FRQ_SIZE]={0};
	STRU_RF_DATA lastestpatten[MAX_RC_FRQ_SIZE]={0};
	int sweep_noise=0;
	int sweep_noise_fluct=0;
	uint8_t pre_lists_size=0;
	
	//step1 remove  the error channel if the errors in the working patten meets the state to change patten,and to  caculate the condition for selection 
	for(i=0;i<context.rf_info.fine_sweep_size;i++){
		
		if(0==find_ch_in_error_list(context.rf_info.pre_selection_list[i].id)){
			pre_lists_size++;
			sweep_noise = context.rf_info.sweep_pwr_avrg_value[context.rf_info.pre_selection_list[i].id].value * SWEEP_NOISE_ORDER;
			sweep_noise_fluct = context.rf_info.sweep_pwr_fluct_value[context.rf_info.pre_selection_list[i].id].value *(100-SWEEP_NOISE_ORDER);
			list[i].value=( sweep_noise + sweep_noise_fluct)/100;
			list[i].id=context.rf_info.sweep_pwr_avrg_value[context.rf_info.pre_selection_list[i].id].id;
		}
	}
	reset_error_list_record();
	if(pre_lists_size <1) return;
	//step2 sort the list by value and record the sort results
	selectionSortBy(listr,pre_lists_size,list,1);
	for(i=0;i<pre_lists_size;i++){
		context.rf_info.sort_result_list[i].id=listr[i].id;
		context.rf_info.sort_result_list[i].value=listr[i].value;
	}
	
	//step3 decide the len of the patten
	context.rf_info.rc_ch_working_patten_len=(pre_lists_size > SKY_PATTEN_SIZE_2G) ? SKY_PATTEN_SIZE_2G : pre_lists_size;
	DLOG_Critical("patten len = %d",context.rf_info.rc_ch_working_patten_len);
	if(context.rf_info.sort_result_list[context.rf_info.rc_ch_working_patten_len-1].value -context.rf_info.sort_result_list[0].value > SWEEP_NOISE_SELECT_DIFF_THD ){
		context.rf_info.rc_ch_working_patten_len=context.rf_info.rc_ch_working_patten_len-1;
	}
	else {
		int begin  = context.rf_info.rc_ch_working_patten_len;
		for(i=begin-1;i<context.rf_info.fine_sweep_size;i++){
			if(context.rf_info.sort_result_list[i].value==context.rf_info.sort_result_list[i+1].value) {
				context.rf_info.rc_ch_working_patten_len++;
			}
			else break;
		}
	}
	#if 0
	for(i=0;i<context.rf_info.rc_ch_working_patten_len;i++){
		DLOG_Critical("%d:cur_sweep_value[%d][%d]=%d",i,
			context.rf_info.sort_result_list[i].id,
			BB_GetRcFrqByCh(context.rf_info.sort_result_list[i].id),
			context.rf_info.sort_result_list[i].value);
	}
	#endif
	
	//step4 record the id for next compare
	for(i=0;i<context.rf_info.rc_ch_working_patten_len;i++){
		context.rf_info.prelist[i].id = listr[i].id;
		context.rf_info.prelist[i].value = listr[i].value;
	}
	
	//step5 sort by id,find the low to high list for generate the patten codes
	for(i=0;i<context.rf_info.rc_ch_working_patten_len;i++){
		
			list[i].value=listr[i].value;
			list[i].id=listr[i].id;
		}
	selectionSortBy(lastestpatten,context.rf_info.rc_ch_working_patten_len,list,0);
	context.rf_info.rc_patten_nextchg_delay=0;
	for(i=0;i<context.rf_info.rc_ch_patten_need_id_size;i++) 
	{
		context.rcChgPatten.patten[i]=0;
	}
	for(i=0;i<context.rf_info.rc_ch_patten_need_id_size;i++)
	{
		if(lastestpatten[i].value!=0){
			context.rcChgPatten.patten[(lastestpatten[i].id)/8] |=dec2bit_index((lastestpatten[i].id)%8);
		}
	}
	context.rcChgPatten.en_flag=1;
	context.rcChgPatten.valid=1;
	context.rcChgPatten.timeout_cnt=context.sync_cnt+STATUS_CHG_DELAY;
}

static void begin_lock_sweep_noise_for_selection(){
	context.rf_info.sweep_finished=0;
	context.rf_info.fine_sweep_id=0;
	context.rf_info.lock_sweep=1;
}

static void end_lock_sweep_noise_for_selection(){
	context.rf_info.sweep_finished=0;
	context.rf_info.fine_sweep_id=0;
	context.rf_info.lock_sweep=0;
}

void sky_gen_rc_working_patten(void)
{
	int sweep_noise_meet=0;
	int working_error_meet=0;
	int working_snr_meet=0;
	
	if(context.rf_info.rc_patten_set_by_usr==1) return;
	if(context.rcChgPatten.en_flag==1) return;
	if(context.rf_info.rc_avr_sweep_result_size<1) return;
	if(context.dev_state == CHECK_FEC_LOCK) return;
	if(context.dev_state == CHECK_LOCK) return;
	if(context.dev_state == WAIT_VT_LOCK) return;

	if((SysTicks_GetDiff(context.rf_info.rc_patten_nextchg_delay, SysTicks_GetTickCount())) < CHG_PATTEN_TIME_GAP)return;
	
	if(context.rf_info.lock_sweep==0)
	{
		working_error_meet = check_working_channel_error();
		if(working_error_meet)
		{
			sweep_noise_meet = corse_check_sweep_noise(1);
			begin_lock_sweep_noise_for_selection();
			DLOG_Critical("change patten because of the working error meet");
		}
		else
		{
			sweep_noise_meet = corse_check_sweep_noise(0);
			if(sweep_noise_meet)
			{
				begin_lock_sweep_noise_for_selection();
				DLOG_Critical("change patten because of the sweep noise meet");
			}
			else
			{
				working_snr_meet = check_working_channel_snr();
				if(working_snr_meet)
				{
					sweep_noise_meet = corse_check_sweep_noise(1);
					begin_lock_sweep_noise_for_selection();
					DLOG_Critical("change patten because of the working snr meet");
				}
			}
			
		}
	}
	else
	{
		if(context.rf_info.sweep_finished)
		{
			DLOG_Critical("begin select good patten");
			find_best_patten();
			end_lock_sweep_noise_for_selection();
		}
	}
	

}

