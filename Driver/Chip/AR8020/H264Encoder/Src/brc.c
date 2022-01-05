////////////////////////////////////////////////////////////////////////////
// brief: Rate Control algorithm for two video view
// author: Li Hu
// date: 2015-11-8
////////////////////////////////////////////////////////////////////////////
#include "data_type.h"
#include "brc.h"
#include "log10.h"
#include "debuglog.h"
#include "reg_rw.h"
#include "sram_sky.h"

#ifdef ARCAST
#undef 	MINI_DRONE
#else
#define MINI_DRONE
#endif

extern unsigned char IPRatio[2][14][2];
extern const int bridx2br[19];
extern uint32_t  H264_Encoder_GetBufferLevel(unsigned char view) ;

#include "enc_internal.h"


enum REG_ADDR {
    ENABLE_ADDR = 0,
    FRAME_XY_ADDR,
    GOPFPS_ADDR,   
    RCEN_BU_ADDR,  
    RCSET1_ADDR, 
    BR_ADDR,  
    RCSET2_ADDR,
    FEEDBACK_ADDR,
    RC_ACBR_ADDR,
    ZW_BSINFO_ADDR,
    QP_ADDR   ,
    MAD_ADDR  ,
    HBITS_ADDR,
    TBITS_ADDR,
    ABITS_ADDR,
    YMSEL_ADDR,
    YMSEH_ADDR,
    REG_TOTAL,
};

enum {
	VIEW0 = 0,
	VIEW1 = 1,
	VIEW_NUM = 2,
};

unsigned char reg[2][REG_TOTAL] = {
    {(0<<2),        
     (1<<2),        
     (2<<2),        
     (5<<2),        
     (6<<2),        
     (7<<2),        
     (8<<2),        
     (9<<2),        
     (10<<2),       
     (12<<2),       
     (18<<2),       
     (19<<2),       
     (20<<2),       
     (21<<2),       
     (22<<2),       
     (23<<2),       
     (24<<2)},
     
     {0x64 + (0<<2),  
      0x64 + (1<<2),  
      0x64 + (2<<2),  
      0x64 + (5<<2),  
      0x64 + (6<<2),  
      0x64 + (7<<2),  
      0x64 + (8<<2),  
      0x64 + (9<<2),  
      0x64 + (10<<2), 
      0x64 + (12<<2), 
      0x64 + (18<<2), 
      0x64 + (19<<2), 
      0x64 + (20<<2), 
      0x64 + (21<<2), 
      0x64 + (22<<2), 
      0x64 + (23<<2), 
      0x64 + (24<<2)}     
};

RC_SETTING rc_setting[] = 
{
	// MinQP, MaxQP
	{  10, 39, 48, 14000 },				// 0 	: 8  Mbps
	{  10, 48, 51, 1000  },				// 1 	: 0.6Mbps
	{  10, 45, 51, 1400  },				// 2	: 1.2Mbps
	{  10, 42, 48, 3500  },				// 3	: 2.4Mbps
	{  10, 40, 48, 6000  },				// 4	: 3  Mbps
	{  10, 40, 48, 6000  },				// 5	: 3.5Mbps	
	{  10, 40, 48, 6000  },				// 6 	: 4	 Mbps
	{  10, 40, 48, 8000  },				// 7 	: 4.8Mbps
	{  10, 40, 48, 8000  },				// 8	: 5  Mbps
	{  10, 40, 48, 9000  },				// 9	: 6  Mbps
	{  10, 40, 48, 11000 },				// 10	: 7  Mbps
	{  10, 39, 48, 12000 },				// 11	: 7.5Mbps	
	{  10, 39, 48, 15000 },				// 12	: 9  Mbps
	{  10, 39, 48, 16000 }				// 13	: 10 Mbps
};

unsigned char InitQp[][2] =
{
    {30, 28}, // 8Mbps
    {38, 40}, // 600kps
    {37, 38}, // 1.2Mbps
    {36, 36}, // 2.4Mbps
    {35, 33}, // 3Mbps
    {35, 33}, // 3.5Mbps
    {34, 32}, // 4Mbps
    {33, 31}, // 4.8Mbps
    {33, 34}, // 5Mbps
    {32, 30}, // 6Mbps
    {31, 29}, // 7Mbps
    {30, 28}, // 7.5Mbps
    {29, 27}, // 9Mbps
    {28, 26}  // 10Mbps
};



RC_DATA rca[VIEW_NUM];

static const int OMEGA_4p = 0xe;
static const int MINVALUE = 4;




static const int QP2QSTEP_8p[6]={0x0a0,0x0b0,0x0d0,0x0e0,0x100,0x120};
int __attribute__ ((section(".h264"))) QP2Qstep_8p(int QP) {
    int i,Qstep_8p;

    Qstep_8p=QP2QSTEP_8p[QP%6];
    for(i=0;i<(QP/6);i++) Qstep_8p*=2;
    return Qstep_8p;
}

int __attribute__ ((section(".h264"))) Qstep2QP_8p(int Qstep_8p) {
    int tmp,q_per=0,q_rem=0;

    if     (Qstep_8p<QP2Qstep_8p(0 )) return 0;
    else if(Qstep_8p>QP2Qstep_8p(51)) return 51;

    tmp=QP2Qstep_8p(5);
    while(Qstep_8p>tmp){tmp=tmp<<1; q_per+=1;}

    if     (Qstep_8p<=(0x0a8<<q_per)) q_rem=0;
    else if(Qstep_8p<=(0x0c0<<q_per)) q_rem=1;
    else if(Qstep_8p<=(0x0d8<<q_per)) q_rem=2;
    else if(Qstep_8p<=(0x0f0<<q_per)) q_rem=3;
    else if(Qstep_8p<=(0x110<<q_per)) q_rem=4;
    else                              q_rem=5;

    return ((q_per*6)+q_rem);
}


int __attribute__ ((section(".h264"))) my_sqrt32(int x) 
{
    int temp=0,v_bit=15,n=0,b=0x8000;

    if(x<=1)
        return x;
    else if(x<0x10000)
    {
        v_bit=7;
        b=0x80;
    }

    do{
        temp=((n<<1)+b)<<(v_bit--);
        if(x>=temp)
        {
            n+=b;
            x-=temp;
        }
    }while(b>>=1);

    return n;
}
int __attribute__ ((section(".h264"))) my_sqrt64(int64 x) 
{
    int v_bit=31,nn = x>>32;
    int64 temp=0,n=0,b=0x80000000;

    if(x<=1)
        return ((int)x);

    if(nn==0)
        return(my_sqrt32((int)x));

    do{
        temp=((n<<1)+b)<<(v_bit--);
        if(x>=temp)
        {
            n+=b;
            x-=temp;
        }
    }while(b>>=1);

    return ((int)n);
}

int __attribute__ ((section(".h264"))) my_imin(int a, int b) 
{ 
   return((a<b)? a:b); 
}

int __attribute__ ((section(".h264"))) my_iequmin(int a, int b) 
{ 
   return((a<=b)? a:b); 
}

int __attribute__ ((section(".h264"))) my_imax(int a, int b) 
{  
   return((a>b)? a:b); 
}

int __attribute__ ((section(".h264"))) my_iClip3(int low, int high, int x) 
{
  x = (x>low)? x:low;
  x = (x<high)? x:high;
  return x;
}




//==============================================================================
//==============================================================================
//extern int v0_poweron_rc_params_set,v1_poweron_rc_params_set;
int __attribute__ ((section(".h264"))) VEBRC_IRQ_Handler(unsigned int view0_feedback, unsigned int view1_feedback) 
{
    int i,run_case=0;
	RC_DATA *prca;
	unsigned int view_idx;
	int qp;
	unsigned int regVal, fps;

    view_idx = VIEW0;
    my_feedback( view_idx, view0_feedback); //// view0 irq
    if((rca[view_idx].fd_irq_en==1) && (rca[view_idx].rc_enable==1) && (rca[view_idx].enable==1)) {
        run_case=0;
    }
    else {
		view_idx = VIEW1;
        my_feedback(view_idx, view1_feedback ); //// view1 irq
        if((rca[view_idx].fd_irq_en==1) && (rca[view_idx].rc_enable==1) && (rca[view_idx].enable==1)) {
            READ_WORD_ENC(reg[view_idx][FRAME_XY_ADDR],i); //check x/y first
            if( ((i&0xffff)>720) || (((i>>16)&0xffff)>1280) )
                //// only for 1080p input from view1 (encoder used view0)
                //1080P input view1 but encoder used view0,
                //for rc read parameter from view1 / use V1 compute / read enc-data from view0 / write QP to view0
                run_case=2;
            else
                run_case=1;
        }
        else {
            run_case=3;
			//DLOG_Error("Wrong Intr %d %d %d\n", rca[view_idx].fd_irq_en, rca[view_idx].rc_enable, rca[view_idx].enable);
			return 0;
        }
    }	

	switch(run_case) {
        case 0:
			view_idx = VIEW0;
            prca = &rca[view_idx];

           	READ_WORD_ENC(reg[view_idx][MAD_ADDR],prca->mad_tmp); //read mad
            READ_WORD_ENC(reg[view_idx][HBITS_ADDR],prca->hbits_tmp); //read hbits
            //READ_WORD_ENC(reg[VIEW0][TBITS_ADDR],v0_tbits_tmp); //read tbits
            READ_WORD_ENC(reg[view_idx][ABITS_ADDR],prca->fbits_tmp);

            //Fix the HeaderBits and TextureBits assignment according to lyu's statistics === end
            if(prca->fd_last_row==1) {
            	READ_WORD_ENC(reg[view_idx][ABITS_ADDR],prca->fbits_tmp); //read abits
                READ_WORD_ENC(reg[view_idx][YMSEL_ADDR],prca->ymsel_tmp); // read frame's y-mse
                READ_WORD_ENC(reg[view_idx][YMSEH_ADDR],prca->ymseh_tmp);
            }
			break;
        case 1:
			view_idx = VIEW1;
			prca = &rca[view_idx];

            READ_WORD_ENC(reg[view_idx][MAD_ADDR],   prca->mad_tmp); //read mad
            READ_WORD_ENC(reg[view_idx][HBITS_ADDR], prca->hbits_tmp); //read hbits
            //READ_WORD_ENC(reg[view_idx][TBITS_ADDR],v1_tbits_tmp); //read tbits
            //Fix the HeaderBits and TextureBits assignment according to lyu's statistics === begin
            READ_WORD_ENC(reg[view_idx][ABITS_ADDR],prca->fbits_tmp);
            //Fix the HeaderBits and TextureBits assignment according to lyu's statistics === end
            if(prca->fd_last_row==1) {
                READ_WORD_ENC(reg[view_idx][ABITS_ADDR],prca->fbits_tmp); //read abits
                READ_WORD_ENC(reg[view_idx][YMSEL_ADDR],prca->ymsel_tmp); // read frame's y-mse
                READ_WORD_ENC(reg[view_idx][YMSEH_ADDR],prca->ymseh_tmp);
            }
			break;
        case 2: // 1080p used view0
            view_idx = VIEW1;
			prca = &rca[view_idx];

            READ_WORD_ENC(reg[VIEW0][MAD_ADDR],prca->mad_tmp); //read mad
            READ_WORD_ENC(reg[VIEW0][HBITS_ADDR],prca->hbits_tmp); //read hbits
            //READ_WORD_ENC(reg[VIEW0][TBITS_ADDR],v1_tbits_tmp); //read tbits
            //Fix the HeaderBits and TextureBits assignment according to lyu's statistics === begin
            READ_WORD_ENC(reg[VIEW0][ABITS_ADDR],prca->fbits_tmp);
            //Fix the HeaderBits and TextureBits assignment according to lyu's statistics === end
            if(prca->fd_last_row==1) {
            	READ_WORD_ENC(reg[VIEW0][ABITS_ADDR],prca->fbits_tmp); //read abits
                READ_WORD_ENC(reg[VIEW0][YMSEL_ADDR],prca->ymsel_tmp); // read frame's y-mse
                READ_WORD_ENC(reg[VIEW0][YMSEH_ADDR],prca->ymseh_tmp);
           	}
			break;
		default:
			return 0;
			break;
    }


	// Row cnt check
	if( ( (prca->fd_row_cnt == (prca->row_cnt_in_bu - 1)) && (prca->last_row_cnt != prca->FrameHeightInMbs - 1)) || 
		( (prca->fd_row_cnt != (prca->row_cnt_in_bu - 1)) && (prca->fd_row_cnt - prca->last_row_cnt != (prca->row_cnt_in_bu)) ) )
	{
		//DLOG_Info("row_cnt is lost. %d %d %d\n", prca->bu_cnt, prca->frame_cnt, prca->gop_cnt);
		//DLOG_Info("row_cnt is lost. %d %d %d\n", prca->fd_row_cnt, prca->last_row_cnt, (unsigned int )prca);
	}
	// Bu cnt check
	if( ( (prca->bu_cnt == 0) && (prca->last_bu_cnt != prca->TotalNumberofBasicUnit - 1) ) || ( (prca->bu_cnt != 0) && (prca->bu_cnt - prca->last_bu_cnt != 1) ) )
	{
		//DLOG_Info("bu_cnt is lost. %d %d\n", prca->bu_cnt, prca->last_bu_cnt);
	}


	// Counter missing recovery
	if( (prca->fd_last_row == TRUE) && (prca->fd_row_cnt == prca->FrameHeightInMbs-1) ) {
		if( prca->bu_cnt != 0 ) {
			//DLOG_Info("bu cnt is lost. %d\n", prca->bu_cnt);
			prca->bu_cnt = 0;
			{
				if( prca->frame_cnt>=(prca->intra_period-1) ) // change "==" to ">=", lhu, 2017/03/09
		        {
		        	prca->frame_cnt=0;
					//if(prca->gop_cnt<=1000)
		            prca->gop_cnt++;
				}
				else
					prca->frame_cnt++;

				//if( prca->bu_cnt != 0 ) // first bu
				{
					if( prca->frame_cnt == 0 ) // first frame
					{
						prca->type = I_SLICE;				
						my_rc_init_gop_params	( view_idx );
					}
					else {
						prca->type = P_SLICE;
					}
					
					my_rc_init_pict(view_idx, 1);
				}
			}
		}
	}

	prca->last_row_cnt = prca->fd_row_cnt;
	prca->last_bu_cnt  = prca->bu_cnt;

#ifdef MINI_DRONE
	// Take photos
	if( (prca->photography_enable == 1) && (prca->photography_state == TAKE_PHOTO_ING || prca->photography_state == TAKE_PHOTO_INSERT_I_START) )
	{
		take_photography(view_idx);
		return 1;
	}
	else if( prca->photography_state == TAKE_PHOTO_INSERT_I_END && prca->bu_cnt > 0 )
	{
		READ_WORD_ENC(reg[view_idx][GOPFPS_ADDR],regVal);
		fps 	= ( (regVal >> 16) & 0xFF );
		regVal   &= ( ~(0xFF<<24) );
		regVal   |= ( ( fps*2 )  << 24);
		WRITE_WORD_ENC(reg[view_idx][GOPFPS_ADDR], regVal);
		prca->photography_state = TAKE_PHOTO_IDLE;
		//DLOG_Error("G1\n");
	}
#endif

	if(prca->rc_enable==1)
	{	
		if( prca->fd_reset == 1 )	// when insert I frame at p frame, this gop should be end, so here need intra_period update.
		{
		  	READ_WORD_ENC(reg[view_idx][GOPFPS_ADDR],regVal); //read gop and fps
  			regVal=(regVal>>16)&0xffff;
  			prca->intra_period = (regVal>>8)&0xff;
            prca->framerate    = regVal&0xff;
			DLOG_Info("Update GoP = %d\n", prca->intra_period );
		}
		
		if ((prca->gop_cnt!=0) && (prca->fd_iframe==1) && (prca->fd_last_row==1) && (prca->poweron_rc_params_set==1)) {
			prca->poweron_rc_params_set = 0; //lhuemu
		}
		
		my_rc_ac_br(view_idx);
	
		//Fix the HeaderBits and TextureBits assignment according to lyu's statistics === begin
		if (prca->type==I_SLICE) prca->hbits_tmp = prca->hbits_tmp - ((prca->MBPerRow*3)>>1); // decrease 1.5 bit every MB for I SLICE
		else					 prca->hbits_tmp = prca->hbits_tmp - 5; 					// decrease 5 bit every BU for P SLICE
		if( prca->hbits_tmp < 0)
			prca->hbits_tmp = 0;
		
		prca->tbits_tmp = (prca->fbits_tmp - prca->PrevFbits) - prca->hbits_tmp;
		if( prca->tbits_tmp < 0 ) prca->tbits_tmp = 0;
		
		if (prca->bu_cnt==0) prca->PrevFbits = 0;
		else				  prca->PrevFbits = prca->fbits_tmp;
		//Fix the HeaderBits and TextureBits assignment according to lyu's statistics === end
		if(prca->fd_last_row==1) {
			prca->ymse_lastframe = (prca->ymse_frame >> 32)? 0xFFFFFFFF : (prca->ymse_frame & 0xFFFFFFFF);
			prca->ymse_frame = (((long long)((prca->ymseh_tmp>>24)&0x3f)<<32) + prca->ymsel_tmp);
			if ((prca->ymseh_tmp>>2)&0x1) prca->wireless_screen=1; else prca->wireless_screen=0;
			if ((prca->ymseh_tmp>>3)&0x1) prca->changeToIFrame=1; else prca->changeToIFrame=0; // lhu, 2017/03/09
			if ((prca->ymseh_tmp>>4)&0x1) prca->insertOneIFrame=1; else prca->insertOneIFrame=0; // lhu, 2017/03/09

			if( ( !(prca->gop_cnt == 0 && prca->frame_cnt <= 8) ) )  // when last row, the frame cnt has been added.
			{
				prca->previous_total_mse -= prca->previous_frame_mse[prca->previous_frame_mse_idx];
			}
			prca->previous_frame_mse[prca->previous_frame_mse_idx] = ((prca->ymse_frame >> 32) & 0xFFFFFFFF) ? 0xFFFFFFFF : prca->ymse_frame;
			prca->previous_total_mse += prca->previous_frame_mse[prca->previous_frame_mse_idx];
			
			prca->previous_frame_mse_idx++;
			if( prca->previous_frame_mse_idx >= 8)
				prca->previous_frame_mse_idx = 0;
			
			if (prca->fd_iframe==1) {
				prca->ifrm_ymse = prca->ymse_frame; // lhu, 2017/04/13
				//prca->firstpframe_coming = 1;
			}
			else 
			{ // obtain the mininum ymse value for all the P Slice, lhu, 2017/05/26
				//if(prca->firstpframe_coming == 1) {
				//	prca->min_pfrm_ymse = prca->ymse_frame;
				//	prca->firstpframe_coming = 0;
				//}
				//prca->min_pfrm_ymse = my_iequmin(prca->ymse_frame, prca->min_pfrm_ymse);
			
				if (prca->fd_last_p==1 ) {
					prca->min_pfrm_ymse = (prca->previous_total_mse >> 3);
					//DLOG_Error("xx %d\n", (unsigned int)prca->min_pfrm_ymse);
				
					READ_WORD_ENC(reg[view_idx][ZW_BSINFO_ADDR],i);
					if ((i>>1)&0x1) my_ac_RCISliceBitRatio(prca->RCISliceBitRatioMax,view_idx);
					else {READ_WORD_ENC(reg[view_idx][RCSET2_ADDR],i); prca->RCISliceBitRatio = (i>>24)&0xf;}
				}
			}
		}
		qp = my_rc_handle_mb( view_idx ); // update QP once
		//if(prca->fd_last_row==1) //frame last
		//	prca->slice_qp = prca->PAveFrameQP;
		
	
		READ_WORD_ENC(reg[view_idx][QP_ADDR],i);
		WRITE_WORD_ENC(reg[view_idx][QP_ADDR],((qp<<24)+(prca->slice_qp<<16)+(i&0xffff))); //write qp & sqp, also maintain ac_gop and ac_iopratio
		my_hold( view_idx );
	}

	return 1;
}
//==============================================================================
//==============================================================================


//-----------------------------------------------------------------------------
// add by bomb for aof target cycles
//-----------------------------------------------------------------------------
void __attribute__ ((section(".h264"))) update_aof_cycle() 
{
    unsigned int v0_on=0,v0_mbs=0, v1_on=0,v1_mbs=0, tmp=0,i=0;

    READ_WORD((REG_BASE_ADDR+(0x00<<2)),v0_on); //view0
    v0_on=(v0_on>>24)&0x1;
    if(v0_on==1){
        READ_WORD((REG_BASE_ADDR+(0x01<<2)),i);
        v0_mbs=((((i>>16)&0xffff)+15)/16)*(((i&0xffff)+15)/16);
        READ_WORD((REG_BASE_ADDR+(0x02<<2)),i);
        v0_mbs=v0_mbs*((i>>16)&0xff);
    }
    READ_WORD((REG_BASE_ADDR+(0x19<<2)),v1_on); //view1
    v1_on=(v1_on>>24)&0x1;
    if(v1_on==1){
        READ_WORD((REG_BASE_ADDR+(0x1a<<2)),i);
        v1_mbs=((((i>>16)&0xffff)+15)/16)*(((i&0xffff)+15)/16);
        READ_WORD((REG_BASE_ADDR+(0x1b<<2)),i);
        v1_mbs=v1_mbs*((i>>16)&0xff);
    }

    i=v0_mbs+v1_mbs;
    if(i>0){
        i=IN_CLK/i;
        if(i>0xff00) i=0xff00; else if(i<540) i=540;
        if(i>580) i=i-30;

        READ_WORD((REG_BASE_ADDR+(0x0a<<2)),tmp); //write back view0
        tmp=(tmp&0xff0000ff)|((i&0xffff)<<8);
        WRITE_WORD((REG_BASE_ADDR+(0x0a<<2)),tmp);
        READ_WORD((REG_BASE_ADDR+(0x23<<2)),tmp); //write back view1
        tmp=(tmp&0xff0000ff)|((i&0xffff)<<8);
        WRITE_WORD((REG_BASE_ADDR+(0x23<<2)),tmp);
    }
}
void __attribute__ ((section(".h264"))) update_aof() 
{
    if(aof_v0_frame!=rca[0].frame_cnt){
        update_aof_cycle();
        aof_v0_frame=rca[0].frame_cnt;
    }
    else if(aof_v1_frame!=rca[1].frame_cnt){
        update_aof_cycle();
        aof_v1_frame=rca[1].frame_cnt;
    }
    else { 
        ;
    }
}
//-----------------------------------------------------------------------------


//==============================================================================
void __attribute__ ((section(".h264"))) my_initial_all( unsigned int view_idx ) 
{
    int i;
    unsigned int qp;
	
    my_rc_params( view_idx );
	update_aof (); // set aof cycles.
    //if(prca->rc_enable==1) //lhuemu
    {
        qp = my_rc_handle_mb( view_idx ); // update QP initial
        READ_WORD_ENC   (reg[view_idx][QP_ADDR],i);
        WRITE_WORD_ENC  (reg[view_idx][QP_ADDR],((qp<<24)+(rca[view_idx].slice_qp<<16)+(i&0xffff))); //after RC initial..
    }
}

//// read feedback data for restart rc and etc
void __attribute__ ((section(".h264"))) my_feedback(unsigned int view_idx, unsigned int feedback) 
{
    RC_DATA *prca = &rca[view_idx];
  
    //READ_WORD_ENC(reg[VIEW0][FEEDBACK_ADDR],i); //read feedback-data
    prca->aof_inc_qp = (feedback>>16)&0xffff;
    prca->fd_row_cnt = (feedback>>9)&0x7f;
    prca->fd_last_row = (feedback>>8)&0x1;
    //prca->fd_cpu_test = (feedback>>7)&0x1;
    prca->fd_irq_en = (feedback>>4)&0x1;
    //if(prca->v0_re_bitrate==0)
    prca->re_bitrate = (feedback>>3)&0x1;
    //if(prca->v0_fd_reset==0)
    prca->fd_reset = (feedback>>2)&0x1;
    prca->fd_iframe = (feedback>>1)&0x1;
    prca->fd_last_p = feedback&0x1;
    READ_WORD_ENC(reg[view_idx][RCEN_BU_ADDR],feedback); //read rc_en, rc_mode & bu
    prca->rc_enable = (feedback>>24)&0x1;
    READ_WORD_ENC(reg[view_idx][ENABLE_ADDR],feedback); //read view enable
    prca->enable = (feedback>>24)&0x1;

	//if(prca->fd_last_row == TRUE )
	//	DLOG_Error("xx %d %d %d %d\n", prca->bu_cnt, prca->frame_cnt, prca->gop_cnt, prca->fd_row_cnt);
}

//-----------------------------------------------------------------------------------
// \brief
//    Initialize rate control parameters
//-----------------------------------------------------------------------------------
void __attribute__ ((section(".h264"))) my_rc_params( unsigned int view_idx ) 
{
    int i,j,m;
    RC_DATA *prca = &rca[view_idx];   

    prca->mad_tmp   					= 0; // @lhu, initial value
    prca->hbits_tmp 					= 0;
    prca->tbits_tmp 					= 0;
    prca->enable    					= 0;
    prca->gop_cnt   					= 0;
    prca->frame_cnt 					= 0;
    prca->bu_cnt    					= 0;
    prca->fd_reset  					= 0; ////
    prca->aof_inc_qp 					= 0;
    prca->fd_last_row					= 0;
    prca->fd_row_cnt 					= 0;
    prca->fd_last_p  					= 0;
    prca->fd_iframe  					= 0;
    prca->re_bitrate 					= 0;
    prca->prev_ac_br_index 				= 0;
    prca->wireless_screen 				= 0; // lhu, 2017/02/27
    prca->changeToIFrame 				= 0; // lhu, 2017/03/09
    prca->insertOneIFrame				= 0; // lhu, 2017/03/09
    prca->PrevFrmPSNRLow				= 0; // lhu, 2017/03/15
    prca->gop_change_NotResetRC 		= 0; // lhu, 2017/03/07
    prca->nextPFgotoIF 					= 0; // lhu, 2017/03/07
    prca->IFduration 					= 0; // lhu, 2017/03/07
    prca->PrevFbits 					= 0; // lhu, 2017/04/05

	// clear the flag of dropped_frame
	prca->curr_frame_is_dropped			= 0;
	prca->Iframe_causedby_scene_change  = 0;
	prca->frame_bits_cnt			    = 0;
    prca->insert_i_frame                = 0;


#if 0
	if( prca->poweron_rc_params_set == 1)
		prca->close_bs_channel 	= NORMAL;
	else {
		if( prca->close_bs_channel != NORMAL ) {
			if( view_idx == 0)
				Reg_Write32_Mask( (unsigned int) 0xa003008c, 0x00, 0x01);
			else
				Reg_Write32_Mask( (unsigned int) 0xa003004c, 0x00, 0x04);
		}
		prca->close_bs_channel = NORMAL;
	}
	prca->insertIframe_since_buffer = FALSE;
#endif

	prca->photography_enable = 0;
	prca->photography_state  = TAKE_PHOTO_IDLE; 
	prca->photography_frmcnt = 0;

	prca->last_bu_cnt				= prca->TotalNumberofBasicUnit - 1;
	prca->last_row_cnt				= prca->TotalNumberofBasicUnit - 1;
	prca->previous_frame_mse_idx    = 0;
	prca->previous_total_mse		= 255*255;
	for( i=0; i<8; i++) {
		prca->previous_frame_mse[i] = 0xFFFFFF;
	}
	
    READ_WORD_ENC(reg[view_idx][FRAME_XY_ADDR],m); //read frame-x & frame-y
        i=(m>>16)&0xffff;
        j=m&0xffff;
        prca->MBPerRow = (i+15)/16;
        prca->FrameSizeInMbs = prca->MBPerRow*((j+15)/16);
        prca->size = prca->FrameSizeInMbs<<8;
        prca->width = i;
        prca->height = j;

    READ_WORD_ENC(reg[view_idx][GOPFPS_ADDR],i); //read gop and fps
        i=(i>>16)&0xffff;
        prca->intra_period = (i>>8)&0xff;
        prca->framerate = i&0xff;
	DLOG_Critical(" Intra Period = %d\n", prca->intra_period );

    READ_WORD_ENC(reg[view_idx][RCEN_BU_ADDR],i); //read rc_en, rc_mode & bu
        prca->rc_enable = (i>>24)&0x1;
        prca->RCUpdateMode = (i>>16)&0x3;
        prca->BasicUnit = (i&0xffff);
        prca->BasicUnit = (i&0xffff);

    READ_WORD_ENC(reg[view_idx][RCSET1_ADDR],i); //read rcset1
        prca->PMaxQpChange  = i&0x3f;				/* Not used */
        prca->RCMinQP       = (i>>8)&0x3f;
        prca->RCMaxQP       = (i>>16)&0x3f;


    READ_WORD_ENC(reg[view_idx][BR_ADDR],i); //read br
        prca->bit_rate = i;

    READ_WORD_ENC(reg[view_idx][RCSET2_ADDR],i); //read rcset2
        prca->RCISliceBitRatioMax = (i>>8)&0x3f;
        prca->RCIoverPRatio = (i>>16)&0xf;
        prca->RCISliceBitRatio = (i>>24)&0xf;
        unsigned int width_in256 = ((prca->width + 255) & (~(0xFF)));
        prca->dvp_lb_freesize = 24576 - (( (width_in256 << 2)  + (width_in256<< 1) )); // 24576 - picture width * 16*3/2/4
        //DLOG_Info("dvp_lb_freesize %08x\n", prca->dvp_lb_freesize);
}


//===== Auto-config Bit-Rate =====
void __attribute__ ((section(".h264"))) my_rc_ac_br(int view) 
{
    int m,ac_br;
    unsigned char ac_br_index;
	unsigned int i;
	RC_DATA *prca = &rca[view];

    READ_WORD_ENC(reg[view][RC_ACBR_ADDR],m);
    prca->ac_br_index = (m>>26)&0x3f;

    ac_br_index = prca->ac_br_index;
    ac_br = bridx2br[ac_br_index]; // use look-up table for bitrate generation, lhu, 2017/06/12
    prca->bit_rate = ac_br;

    if (ac_br_index != prca->prev_ac_br_index)
    {
         WRITE_WORD_ENC(reg[view][BR_ADDR], ac_br);
		 READ_WORD_ENC(reg[view][RCSET1_ADDR],i); //read rcset1
         prca->PMaxQpChange  = i&0x3f;
         prca->RCMinQP       = (i>>8)&0x3f;
         prca->RCMaxQP       = (i>>16)&0x3f;
    }
    prca->prev_ac_br_index = ac_br_index;

}

#if 0
unsigned short my_divider2psnr(int my_divider) {
    unsigned short my_psnr;
    if      (my_divider>=1000000)                      my_psnr = 60;// 10^6.0=10000
    else if (my_divider>=794328 && my_divider<1000000) my_psnr = 59;// 10^5.9=794328
    else if (my_divider>=630957 && my_divider<794328)  my_psnr = 58;// 10^5.8=630957
    else if (my_divider>=501187 && my_divider<630957)  my_psnr = 57;// 10^5.7=501187
    else if (my_divider>=398107 && my_divider<501187)  my_psnr = 56;// 10^5.6=398107
    else if (my_divider>=316227 && my_divider<398107)  my_psnr = 55;// 10^5.5=316227
    else if (my_divider>=251188 && my_divider<316227)  my_psnr = 54;// 10^5.4=251188
    else if (my_divider>=199526 && my_divider<251188)  my_psnr = 53;// 10^5.3=199526
    else if (my_divider>=158489 && my_divider<199526)  my_psnr = 52;// 10^5.2=158489
    else if (my_divider>=125892 && my_divider<158489)  my_psnr = 51;// 10^5.1=125892
    else if (my_divider>=100000 && my_divider<125892)  my_psnr = 50;// 10^5.0=100000
    else if (my_divider>=79432  && my_divider<100000)  my_psnr = 49;// 10^4.9=79432
    else if (my_divider>=63095  && my_divider<79432 )  my_psnr = 48;// 10^4.8=63095
    else if (my_divider>=50118  && my_divider<63095 )  my_psnr = 47;// 10^4.7=50118
    else if (my_divider>=39810  && my_divider<50118 )  my_psnr = 46;// 10^4.6=39810
    else if (my_divider>=31622  && my_divider<39810 )  my_psnr = 45;// 10^4.5=31622
    else if (my_divider>=25118  && my_divider<31622 )  my_psnr = 44;// 10^4.4=25118
    else if (my_divider>=19952  && my_divider<25118 )  my_psnr = 43;// 10^4.3=19952
    else if (my_divider>=15848  && my_divider<19952 )  my_psnr = 42;// 10^4.2=15848
    else if (my_divider>=12589  && my_divider<15848 )  my_psnr = 41;// 10^4.1=12589
    else if (my_divider>=10000  && my_divider<12589 )  my_psnr = 40;// 10^4.0=10000
    else if (my_divider>=7943   && my_divider<10000 )  my_psnr = 39;// 10^3.9=7943
    else if (my_divider>=6309   && my_divider<7943  )  my_psnr = 38;// 10^3.8=6309
    else if (my_divider>=5011   && my_divider<6309  )  my_psnr = 37;// 10^3.7=5011
    else if (my_divider>=3981   && my_divider<5011  )  my_psnr = 36;// 10^3.6=3981
    else if (my_divider>=3162   && my_divider<3981  )  my_psnr = 35;// 10^3.5=3162
    else if (my_divider>=2511   && my_divider<3162  )  my_psnr = 34;// 10^3.4=2511
    else if (my_divider>=1995   && my_divider<2511  )  my_psnr = 33;// 10^3.3=1995
    else if (my_divider>=1584   && my_divider<1995  )  my_psnr = 32;// 10^3.2=1584
    else if (my_divider>=1258   && my_divider<1584  )  my_psnr = 31;// 10^3.1=1258
    else if (my_divider>=1000   && my_divider<1258  )  my_psnr = 30;// 10^3.0=1000
    else if (my_divider>=794    && my_divider<1000  )  my_psnr = 29;// 10^2.9=794
    else if (my_divider>=630    && my_divider<794   )  my_psnr = 28;// 10^2.8=630
    else if (my_divider>=501    && my_divider<630   )  my_psnr = 27;// 10^2.7=501
    else if (my_divider>=398    && my_divider<501   )  my_psnr = 26;// 10^2.6=398
    else if (my_divider>=316    && my_divider<398   )  my_psnr = 25;// 10^2.5=316
    else if (my_divider>=251    && my_divider<316   )  my_psnr = 24;// 10^2.4=251
    else if (my_divider>=199    && my_divider<251   )  my_psnr = 23;// 10^2.3=199
    else if (my_divider>=158    && my_divider<199   )  my_psnr = 22;// 10^2.2=158
    else if (my_divider>=125    && my_divider<158   )  my_psnr = 21;// 10^2.1=125
    else if (my_divider>=100    && my_divider<125   )  my_psnr = 20;// 10^2.0=100
    else if (my_divider>=79     && my_divider<100   )  my_psnr = 19;// 10^1.9=79
    else if (my_divider>=63     && my_divider<79    )  my_psnr = 18;// 10^1.8=63
    else if (my_divider>=50     && my_divider<63    )  my_psnr = 17;// 10^1.7=50
    else if (my_divider>=39     && my_divider<50    )  my_psnr = 16;// 10^1.6=39
    else if (my_divider>=31     && my_divider<39    )  my_psnr = 15;// 10^1.5=31
    else if (my_divider>=25     && my_divider<31    )  my_psnr = 14;// 10^1.4=25
    else                                               my_psnr = 13;
    
    return my_psnr;
}
#endif


/*===== Criteria for auto-config of RCISliceBitRatio=====
1> Depend on comparsion of I frame's psnr(Ipsnr) and MAX frame's psnr(Ppsnr) in current GOP.
2> Ppsnr-Ipsnr and RCISliceBitRatio_nextGOP and RCISliceBitRatio_currGOP's relation:
    ||                       ||
    \/                       \/
    -5             RCISliceBitRatio_currGOP-4
    ...                      ...
    -2             RCISliceBitRatio_currGOP-1
    -1             RCISliceBitRatio_currGOP
     0             RCISliceBitRatio_currGOP
    +1             RCISliceBitRatio_currGOP+1
    +2             RCISliceBitRatio_currGOP+2
    ...                      ...
    +5             RCISliceBitRatio_currGOP+5
3> Finally use RCISliceBitRatioMax value to clamp final output RCISliceBitRatio value.*/
void __attribute__ ((section(".h264"))) my_ac_RCISliceBitRatio(unsigned char RCISliceBitRatioMax, int view) 
{
    long long whm255square,m;
    int i,iframe_divider,pframe_divider_max;
    unsigned short iframe_psnr, pframe_psnr_max;
    signed short diffpsnr_PI;
	RC_DATA *prca = &rca[view];
    unsigned char RCISliceBitRatio_currGOP;
    signed short RCISliceBitRatio_nextGOP;
    unsigned int IPRatioMax = RCISliceBitRatioMax;
    int delta;

	if( prca->ifrm_ymse == 0)
		prca->ifrm_ymse = 255*255;
	if( prca->min_pfrm_ymse == 0)
		prca->min_pfrm_ymse = 255*255;

    whm255square = (long long)(prca->width*255)*(long long)(prca->height*255);
    iframe_divider = (int)(whm255square/prca->ifrm_ymse);
    pframe_divider_max = (int)(whm255square/prca->min_pfrm_ymse);

    iframe_psnr     = get_10log10(iframe_divider);
    pframe_psnr_max = get_10log10(pframe_divider_max);


    if( prca->realtime_mode )
        IPRatioMax =  IPRatio[1][prca->ac_br_index][0];

    if( (pframe_psnr_max >= iframe_psnr + 10) && (prca->RCISliceBitRatio == IPRatioMax) )
        IPRatioMax <<= 1; 
        

    diffpsnr_PI = pframe_psnr_max - (iframe_psnr + delta);  // jlliu, allow iframe_psnr is better than p frmae 1 db.
    RCISliceBitRatio_currGOP = prca->RCISliceBitRatio;

	if( iframe_psnr > 35 )
        delta = 4;
    else if( iframe_psnr > 30 )
        delta = 2;
    else
        delta = 1;
    
    if (diffpsnr_PI>=1)                         RCISliceBitRatio_nextGOP = RCISliceBitRatio_currGOP+diffpsnr_PI;
    //else if (diffpsnr_PI>=-1 && diffpsnr_PI<=0) RCISliceBitRatio_nextGOP = RCISliceBitRatio_currGOP;
    else if (diffpsnr_PI==0) RCISliceBitRatio_nextGOP = RCISliceBitRatio_currGOP;
    else                                        RCISliceBitRatio_nextGOP = RCISliceBitRatio_currGOP+diffpsnr_PI;
    RCISliceBitRatio_nextGOP = my_imax(RCISliceBitRatio_nextGOP, 1);
    RCISliceBitRatio_nextGOP = my_iequmin(RCISliceBitRatio_nextGOP, IPRatioMax);
    DLOG_Error( "%d %d %d %d\n", pframe_psnr_max, iframe_psnr, prca->RCISliceBitRatio, RCISliceBitRatio_nextGOP);
    
    READ_WORD_ENC(reg[view][ZW_BSINFO_ADDR],i);
    WRITE_WORD_ENC(reg[view][ZW_BSINFO_ADDR],((i&0xffffff03)|((RCISliceBitRatio_nextGOP&0x3f)<<2)));
    prca->RCISliceBitRatio = RCISliceBitRatio_nextGOP;

}

void __attribute__ ((section(".h264"))) my_rc_init_seq( unsigned int view_idx ) 
{
//18  double L1,L2,L3;
  int bpp_p6,qp,i;
  RC_DATA *prca = &rca[view_idx];

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

  prca->type     = I_SLICE;
  prca->qp       = 0;
  prca->slice_qp = 0;
  prca->c1_over  = 0;
  prca->cmadequ0 = 0;// lhumad
  prca->frame_mad   = 0;
  prca->frame_tbits = 0;
  prca->frame_hbits = 0;
  prca->frame_abits = 0;

  //if(prca->RCUpdateMode != RC_MODE_0)
  {
    // if (prca->RCUpdateMode==RC_MODE_1 && prca->intra_period==1) {// make sure it execute only once!!! lhumod
    //   prca->no_frm_base = prca->intra_period*50; //!!!
    //   prca->intra_period = prca->no_frm_base;// make fake for frame_cnt increment, lhumod
    // }
    // else 
	// if (prca->RCUpdateMode==RC_MODE_3) 
	{
		prca->no_frm_base = prca->intra_period*1; // lhugop
    }
  }

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
  switch (prca->RCUpdateMode )
  {
     //case RC_MODE_0: my_v0_updateQP = my_v0_updateQPRC0; break;
     //case RC_MODE_1: my_v0_updateQP = my_v0_updateQPRC1; break;
     case RC_MODE_3: 
	 	my_updateQP = my_updateQPRC3; 
		break;
     default: 
	 	my_updateQP = my_updateQPRC3; break;
  }

  prca->PreviousMAD_8p = (1<<8);
  prca->CurrentMAD_8p  = (1<<8);
  prca->Target        = 0;
  prca->LowerBound    = 0;
  prca->UpperBound1   = MAX_INT;
  prca->UpperBound2   = MAX_INT;
  prca->PAveFrameQP   = 0;
  prca->m_Qc          = 0;
  prca->PAverageQp    = 0;

  for(i=0;i<70;i++)
  {
    prca->BUPFMAD_8p[i] = 0;
    prca->BUCFMAD_8p[i] = 0;
  }

  prca->PrevBitRate = prca->bit_rate; //lhumod
  //compute the total number of MBs in a frame
  if(prca->BasicUnit >= prca->FrameSizeInMbs)
    prca->BasicUnit = prca->FrameSizeInMbs;

  if(prca->BasicUnit < prca->FrameSizeInMbs)
    prca->TotalNumberofBasicUnit = prca->FrameSizeInMbs/prca->BasicUnit;
  else
    prca->TotalNumberofBasicUnit = 1;

  //initialize the parameters of fluid flow traffic model
  prca->CurrentBufferFullness = 0;
//  rca.GOPTargetBufferLevel = 0; //(double)rca.CurrentBufferFullness;

  //initialize the previous window size
  prca->m_windowSize = 0;
  prca->MADm_windowSize = 0;
  prca->NumberofCodedPFrame = 0;
  prca->NumberofGOP = 0;
  //remaining # of bits in GOP
  prca->RemainingBits = 0;

  prca->GAMMAP_1p=1;
  prca->BETAP_1p=1;

  prca->Pm_X1_8p = prca->bit_rate<<8;
  prca->Pm_X2_8p = 0;
  // linear prediction model for P picture
  prca->PMADPictureC1_12p = (1<<12);
  prca->PMADPictureC2_12p = 0;
  prca->MADPictureC1_12p = (1<<12);
  prca->MADPictureC2_12p = 0;

  // Initialize values
  for(i=0;i<20;i++)
  {
    prca->m_rgQp_8p[i] = 0;
    prca->m_rgRp_8p[i] = 0;
    prca->m_rgRp_8prr8[i] = 0;
    prca->rc_tmp0[i] = 0;
    prca->rc_tmp1[i] = 0;
    prca->rc_tmp2[i] = 0;
    prca->rc_tmp3[i] = 0;
    prca->rc_tmp4[i] = 0;

    prca->PictureMAD_8p[i]   = 0;
    prca->ReferenceMAD_8p[i] = 0;
    prca->mad_tmp0[i] = 0;
    prca->mad_tmp0_valid[i] = 1;
    prca->mad_tmp1[i] = 0;
    prca->mad_tmp2[i] = 0;

    prca->rc_rgRejected[i] = FALSE;
    prca->mad_rgRejected[i] = FALSE;
  }

  prca->rc_hold = 0;
  prca->mad_hold = 0;

  prca->PPictureMAD_8p = 0;
  //basic unit layer rate control
  prca->PAveHeaderBits1 = 0;
  prca->PAveHeaderBits3 = 0;
  prca->DDquant = (prca->TotalNumberofBasicUnit>=9? 1:2);

  uint frame_bs = prca->bit_rate/prca->framerate;

  bpp_p6=(frame_bs<<6)/prca->size; //for test
/*if     (bpp_p6<=0x26) qp=35;
  else if(bpp_p6<=0x39) qp=25;
  else if(bpp_p6<=0x59) qp=20;
  else                  qp=10;*/// test for more initial_qp assignment, lhuemu
  if     (bpp_p6<=0x6 ) {if (prca->height>=1080) qp=42; else if(prca->height>=720) qp=40; else qp=38;}
  else if(bpp_p6<=0x16) {if (prca->height>=1080) qp=39; else if(prca->height>=720) qp=37; else qp=35;}
  else if(bpp_p6<=0x26) qp=35;
  else if(bpp_p6<=0x39) qp=25;
  else if(bpp_p6<=0x59) qp=20;
  else                  qp=10;

  prca->MyInitialQp = qp;
  prca->LastIQp		= qp;
}


void __attribute__ ((section(".h264"))) my_rc_init_GOP(unsigned int view_idx, int np) 
{
  RC_DATA *prca = &rca[view_idx];
  Boolean Overum=FALSE;
  int OverBits,denom,i;
  int GOPDquant;
  int gop_bits;
  int v0_RCISliceBitsLow,v0_RCISliceBitsHigh,v0_RCISliceBitsLow2,v0_RCISliceBitsHigh2,v0_RCISliceBitsLow4,v0_RCISliceBitsHigh4;
  int v0_RCISliceBitsLow8,v0_RCISliceBitsHigh8,v0_RCISliceBitsLow9,v0_RCISliceBitsHigh9; // lhuqu1
  unsigned int regVal;
  
  DLOG_Critical("IPRatio = %d, MaxQP = %d, MinQP = %d, BR = %d, GoP = %d, Fps = %d \n", 
  	prca->RCISliceBitRatio, 
  	prca->RCMaxQP, 
  	prca->RCMinQP, 
  	prca->bit_rate,
  	prca->intra_period,
  	prca->framerate
  	);

    //Compute InitialQp for each GOP
    prca->TotalPFrame = np;

	prca->ave_frame_bits = (prca->bit_rate/prca->framerate);
	
    if(prca->gop_cnt==0)
    {
        prca->QPLastGOP   = prca->MyInitialQp;
        prca->PAveFrameQP = prca->MyInitialQp;
        prca->PAverageQp  = prca->MyInitialQp;
        prca->m_Qc        = prca->MyInitialQp;
		prca->LastIQp	  = prca->MyInitialQp;
    }
    else
    {
        //compute the average QP of P frames in the previous GOP
        prca->PAverageQp = (prca->TotalQpforPPicture+(np>>1))/np;// + 0.5);

        if     (np>=22) GOPDquant=2; // GOPDquant=(int)((1.0*(np+1)/15.0) + 0.5);
        else if(np>=7 ) GOPDquant=1; // if(GOPDquant>2)
        else            GOPDquant=0; // GOPDquant=2;

        prca->PAverageQp -= GOPDquant;

        if(prca->PAverageQp > (prca->QPLastPFrame-2))
            prca->PAverageQp--;

        if(prca->RCUpdateMode == RC_MODE_3) {
            //69 gop_bits = rca.v0_no_frm_base * rca.v0_frame_bs;
            gop_bits = (!prca->intra_period? 1 : prca->intra_period ) * ( prca->bit_rate/prca->framerate );
            if ( (prca->IFduration==1 && prca->insertOneIFrame==1) || ( prca->Iframe_causedby_scene_change == 1) || ( prca->insert_i_frame == 1) ){
				if( prca->width > 1280 || prca->height > 720) 
					prca->LastIQp = InitQp[prca->ac_br_index][0];
				else
					prca->LastIQp = InitQp[prca->ac_br_index][1];
            } else {
                // Use the Previous ISliceBitRatio to calculate the initial QP value of current I Slice, lhu, 2017/05/25
                prca->RCISliceTargetBits = gop_bits * prca->RCISliceBitRatio/(prca->RCISliceBitRatio+(prca->intra_period-1));
				prca->TargetForBasicUnit = prca->RCISliceTargetBits / prca->TotalNumberofBasicUnit;
				unsigned int lastIQp_old = prca->LastIQp;
				
                v0_RCISliceBitsLow    = ((prca->RCISliceTargetBits * 230) >> 8);
                v0_RCISliceBitsHigh   = ((prca->RCISliceTargetBits * 282) >> 8);
                v0_RCISliceBitsLow2   = ((prca->RCISliceTargetBits * 205) >> 8);
                v0_RCISliceBitsHigh2  = ((prca->RCISliceTargetBits * 307) >> 8);
                v0_RCISliceBitsLow4   = ((prca->RCISliceTargetBits * 134) >> 8);
                v0_RCISliceBitsHigh4  = ((prca->RCISliceTargetBits * 358) >> 8);
                v0_RCISliceBitsLow8   = ((prca->RCISliceTargetBits *  51) >> 8);
                v0_RCISliceBitsHigh8  = ((prca->RCISliceTargetBits * 461) >> 8);
                v0_RCISliceBitsLow9   = ((prca->RCISliceTargetBits *  26) >> 8);
                v0_RCISliceBitsHigh9  = ((prca->RCISliceTargetBits * 486) >> 8);
                if(prca->RCISliceActualBits  <= v0_RCISliceBitsLow9)                                                              prca->LastIQp  = prca->LastIQp	-8;
                else if((v0_RCISliceBitsLow9  < prca->RCISliceActualBits) && (prca->RCISliceActualBits <= v0_RCISliceBitsLow8))   prca->LastIQp  = prca->LastIQp	-6;
                else if((v0_RCISliceBitsLow8  < prca->RCISliceActualBits) && (prca->RCISliceActualBits <= v0_RCISliceBitsLow4))   prca->LastIQp  = prca->LastIQp	-4;
                else if((v0_RCISliceBitsLow4  < prca->RCISliceActualBits) && (prca->RCISliceActualBits <= v0_RCISliceBitsLow2))   prca->LastIQp  = prca->LastIQp	-2;
                else if((v0_RCISliceBitsLow2  < prca->RCISliceActualBits) && (prca->RCISliceActualBits <= v0_RCISliceBitsLow))    prca->LastIQp  = prca->LastIQp	-1;
                else if((v0_RCISliceBitsLow   < prca->RCISliceActualBits) && (prca->RCISliceActualBits <= v0_RCISliceBitsHigh))   prca->LastIQp  = prca->LastIQp	  ;
                else if((v0_RCISliceBitsHigh  < prca->RCISliceActualBits) && (prca->RCISliceActualBits <= v0_RCISliceBitsHigh2))  prca->LastIQp  = prca->LastIQp	+1;
                else if((v0_RCISliceBitsHigh2 < prca->RCISliceActualBits) && (prca->RCISliceActualBits <= v0_RCISliceBitsHigh4))  prca->LastIQp  = prca->LastIQp	+2;
                else if((v0_RCISliceBitsHigh4 < prca->RCISliceActualBits) && (prca->RCISliceActualBits <= v0_RCISliceBitsHigh8))  prca->LastIQp  = prca->LastIQp	+4;
                else if((v0_RCISliceBitsHigh8 < prca->RCISliceActualBits) && (prca->RCISliceActualBits <= v0_RCISliceBitsHigh9))  prca->LastIQp  = prca->LastIQp	+6;
                else if(prca->RCISliceActualBits > v0_RCISliceBitsHigh9)                                                          prca->LastIQp  = prca->LastIQp	+8;
				
				prca->LastIQp = my_iClip3(prca->RCMinQP,  prca->RCMaxQP,  prca->LastIQp);

				DLOG_Error( "%d %d %d %d %d\n", prca->RCISliceTargetBits, prca->TargetForBasicUnit, prca->RCISliceActualBits, lastIQp_old, prca->LastIQp);
            }

        }

        {
            // QP is constrained by QP of previous QP
            prca->PAverageQp = my_iClip3(prca->QPLastGOP-2, prca->QPLastGOP+2, prca->PAverageQp);
			prca->PAverageQp = my_iClip3(prca->LastIQp - 3, prca->LastIQp  +3, prca->PAverageQp);
        }


        // Also clipped within range.
        prca->PAverageQp = my_iClip3(prca->RCMinQP,  prca->RCMaxQP,  prca->PAverageQp);

        prca->MyInitialQp = prca->PAverageQp;
        prca->Pm_Qp       = prca->PAverageQp;
        prca->PAveFrameQP = prca->PAverageQp; //(13)
        prca->QPLastGOP   = prca->PAverageQp;
    }

    prca->TotalQpforPPicture= 0;//(13)
    prca->TotalIQp 			= 0;

    // bit allocation for RC_MODE_3
    //if(prca->RCUpdateMode == RC_MODE_3) // running this only once !!!
    {
        // calculate allocated bRCUpdateModeits for each type of frame
        // Fix the ISliceBitRatio when decide to insert I Frame and calculate bit target for I/P frame, lhu, 2017/05/25
        if ( (prca->IFduration==1 && prca->insertOneIFrame==1) || (prca->Iframe_causedby_scene_change == 1) || (prca->insert_i_frame == 1))
            prca->RCISliceBitRatio = 4;

        denom = (!prca->intra_period? 1:prca->intra_period) + prca->RCISliceBitRatio - 1;

        // set bit targets for each type of frame
//18      rca.RCPSliceBits = (int)floor(gop_bits/denom + 0.5F);
        prca->RCPSliceBits = gop_bits/denom ;
        prca->RCISliceBits = (prca->intra_period)? (prca->RCISliceBitRatio * prca->RCPSliceBits) : 0;

        prca->NISlice = (prca->intra_period)? (prca->intra_period/prca->intra_period):0; // totoal I-frame number
        prca->NPSlice = prca->intra_period - prca->NISlice;
    }

    // check if the last GOP over uses its budget. If yes, the initial QP of the I frame in
    // the coming  GOP will be increased.
    if(prca->RemainingBits<0)
        Overum=TRUE;
    OverBits=-prca->RemainingBits;

    prca->RemainingBits = 0; // set remainingbits as 0 at beginning of gop, lhu, 2017/02/08
    //initialize the lower bound and the upper bound for the target bits of each frame, HRD consideration
    prca->LowerBound  = prca->RemainingBits + (prca->bit_rate/prca->framerate);
    prca->UpperBound1 = prca->RemainingBits + (prca->bit_rate<<1); //2.048
    prca->UpperBound2  = ((OMEGA_4p*prca->UpperBound1) >> 4); // lhu, 2017/03/13

    //compute the total number of bits for the current GOP
    if (prca->IFduration!=1)
        gop_bits = (1+np)*(prca->bit_rate/prca->framerate);
    else {
        if (prca->changeToIFrame==1)
            gop_bits = ((1+np)*(prca->bit_rate/prca->framerate)*14)/10; // expand whole GOP target by 40%, lhu, 2017/03/07
        else if (prca->insertOneIFrame==1)
            gop_bits = (1+np)*(prca->bit_rate/prca->framerate); // maintain the original GOP target, lhu, 2017/03/09
    }
    prca->RemainingBits+= gop_bits;
    prca->Np = np;

    //  OverDuantQp=(int)(8 * OverBits/gop_bits+0.5);
    prca->GOPOverdue=FALSE;

	DLOG_Critical("I=%d, P=%d", prca->RCISliceBits, prca->RCPSliceBits);

}


void __attribute__ ((section(".h264"))) my_rc_init_pict(unsigned int view_idx, int mult) 
{
  RC_DATA *prca = &rca[view_idx];
  int i,tmp_T;

    //if ( prca->type==P_SLICE ) //g1|| (rca.RCUpdateMode==RC_MODE_1 &&(rca.gop_cnt!=0 || rca.frame_cnt!=0)) ) // (rca.number !=0)
    if ( prca->type==P_SLICE || ((prca->type==I_SLICE) && (!(prca->gop_cnt==0 && prca->frame_cnt==0))) ) // lhuitune
    {
      //// for CBR ...
      if(prca->PrevBitRate	!=prca->bit_rate)
        prca->RemainingBits += (prca->bit_rate - prca->PrevBitRate)*prca->Np/prca->framerate;
      /*if(prca->re_bitrate == 1)
      {
        prca->re_bitrate = 0;
        prca->RemainingBits += (prca->new_bitrate - prca->bit_rate)*prca->Np/prca->framerate;
        prca->bit_rate = prca->new_bitrate;
      }*/


      {
        if(prca->NumberofCodedPFrame>0)
        {
          for(i=0;i<prca->TotalNumberofBasicUnit;i++)
             prca->BUPFMAD_8p[i] = prca->BUCFMAD_8p[i];
        }

        if(prca->gop_cnt==0) //(rca.NumberofGOP==1)
        {
          if(prca->frame_cnt==2) //(rca.NumberofPPicture==1)
          {
            prca->TargetBufferLevel = prca->CurrentBufferFullness;
//18            rca.DeltaP = (rca.CurrentBufferFullness - rca.GOPTargetBufferLevel)/(rca.TotalPFrame-1);
            prca->DeltaP = prca->CurrentBufferFullness/(prca->TotalPFrame-1);
            prca->TargetBufferLevel -= prca->DeltaP;
          }
          else if(prca->frame_cnt>2) //(rca.NumberofPPicture>1)
            prca->TargetBufferLevel -= prca->DeltaP;
        }
        else if(prca->gop_cnt>0) //(rca.NumberofGOP>1)
        {
          if(prca->frame_cnt==1) //(rca.NumberofPPicture==0)
          {
            prca->TargetBufferLevel = prca->CurrentBufferFullness;
//18            rca.DeltaP = (rca.CurrentBufferFullness - rca.GOPTargetBufferLevel) / rca.TotalPFrame;
            prca->DeltaP = prca->CurrentBufferFullness/prca->TotalPFrame;
            prca->TargetBufferLevel -= prca->DeltaP;
          }
          else if(prca->frame_cnt>1) //(rca.NumberofPPicture>0)
            prca->TargetBufferLevel -= prca->DeltaP;
        }
      }
    }

    // Compute the target bit for each frame
    if(prca->type==P_SLICE || ((prca->gop_cnt!=0 || prca->frame_cnt!=0) ))
    {
        // frame layer rate control
        if( prca->NumberofCodedPFrame > 0 )
        {
            //if( prca->RCUpdateMode == RC_MODE_3 )
            if( 1 )
            {
                int bitrate = (prca->type==P_SLICE)? prca->RCPSliceBits: prca->RCISliceBits;
                int denom   = prca->NISlice*prca->RCISliceBits + prca->NPSlice*prca->RCPSliceBits;

                // target due to remaining bits
                prca->Target = ((long long)bitrate*(long long)prca->RemainingBits) / denom;

                // target given original taget rate and buffer considerations
//18            tmp_T = imax(0, (int)floor((double)bitrate - ((rca.CurrentBufferFullness-rca.TargetBufferLevel)/rca.GAMMAP) + 0.5) );
//s             tmp_T = imax(0, bitrate-((rca.CurrentBufferFullness-rca.TargetBufferLevel)/rca.GAMMAP_1p));
                tmp_T = my_imax(0, (bitrate-((prca->CurrentBufferFullness-prca->TargetBufferLevel)>>1)));

                if(prca->type == I_SLICE) {
                    //prca->Target = prca->Target/(prca->RCIoverPRatio); //lhulhu
                }
            }
            else
            {
//18              rca.Target = (int) floor( rca.RemainingBits / rca.Np + 0.5);
                prca->Target = prca->RemainingBits/prca->Np;
                tmp_T=my_imax(0, ((prca->bit_rate/prca->framerate) - ((prca->CurrentBufferFullness-prca->TargetBufferLevel)>>1)));
//s              rca.Target = ((rca.Target-tmp_T)/rca.BETAP) + tmp_T;
                prca->Target = (prca->Target+tmp_T)>>1;
            }
        }
      // basic unit layer rate control
      else
      {
        if(((prca->gop_cnt==0)&&(prca->NumberofCodedPFrame>0)) || (prca->gop_cnt>0))
        {
//18          rca.Target = (int)(floor(rca.RemainingBits/rca.Np + 0.5));
          prca->Target = prca->RemainingBits/prca->Np;
          tmp_T = my_imax(0, ((prca->bit_rate/prca->framerate) - ((prca->CurrentBufferFullness-prca->TargetBufferLevel)>>1)));

//s          rca.Target = ((rca.Target-tmp_T)*rca.BETAP) + tmp_T;
          prca->Target = ((prca->Target+tmp_T)>>1);
        }
      }
      prca->Target = mult * prca->Target;

      // HRD consideration
      if(1) {
        if ( prca->IFduration!=1 )
          prca->Target = my_iClip3(prca->LowerBound, prca->UpperBound2, prca->Target);
        else {
          if ( prca->changeToIFrame == 1 )
            prca->Target = (prca->Target*14)/10; // expand P frame target by 40%, lhu, 2017/03/07
        }
      }
    }

    // frame layer rate control
    prca->NumberofHeaderBits  = 0;
    prca->NumberofTextureBits = 0;
    prca->TotalFrameMAD = 0;// lhumod
    // basic unit layer rate control
    if(prca->BasicUnit < prca->FrameSizeInMbs)
    {
      prca->TotalFrameQP = 0;
      prca->NumberofBasicUnitHeaderBits  = 0;
      prca->NumberofBasicUnitTextureBits = 0;
      prca->TotalMADBasicUnit = 0;
    }
    prca->PrevBitRate = prca->bit_rate; // lhumod
    prca->PrevRCMinQP = prca->RCMinQP; // lhupsnr

	prca->TargetForBasicUnit = prca->Target / prca->TotalNumberofBasicUnit;
	if( prca->TargetForBasicUnit < 0 )
		prca->TargetForBasicUnit = 0;
		

	//DLOG_Error("%d %d %d\n", prca->RemainingBits, prca->Target, prca->fbits_tmp);

}


void __attribute__ ((section(".h264"))) my_rc_update_pict(unsigned int view_idx, int nbits) // after frame running once
{
  int delta_bits;
  RC_DATA *prca = &rca[view_idx];
/////////////////////////////////////////////////////  my_rc_update_pict_frame( );

  //if(prca->RCUpdateMode==RC_MODE_3){
  if( 1 ){
    if(prca->type==I_SLICE && (prca->gop_cnt!=0 || prca->frame_cnt!=0)) //(rca.number != 0)
      prca->NISlice--;
    if(prca->type==P_SLICE)
    {
      my_updatePparams( view_idx );
      prca->NPSlice--;
    }
  }
/////////////////////////////////////////////////////
  if ( prca->type==I_SLICE ) { // lhugop, save bits number for I_SLICE every gop
    prca->RCISliceActualBits = nbits;
  }

  delta_bits=nbits - (prca->bit_rate/prca->framerate);
  // remaining # of bits in GOP
  prca->RemainingBits -= nbits;
  prca->CurrentBufferFullness += delta_bits;

  // update the lower bound and the upper bound for the target bits of each frame, HRD consideration
  prca->LowerBound  -= delta_bits;
  prca->UpperBound1 -= delta_bits;
  prca->UpperBound2  = ((OMEGA_4p*prca->UpperBound1) >> 4);

  // update the parameters of quadratic R-D model
  if( prca->type==P_SLICE )
  {
    my_updateRCModel( view_idx );
    //if(prca->RCUpdateMode == RC_MODE_3)
    //prca->PreviousWholeFrameMAD_8p = prca->frame_mad; // my_ComputeFrameMAD( ) * (1<<8);
//21      rca.PreviousWholeFrameMAD = my_ComputeFrameMAD( ); ////!!!!
  }
}

void __attribute__ ((section(".h264"))) my_updatePparams( unsigned int view_idx ) 
{
  RC_DATA *prca = &rca[view_idx];
  
  prca->Np--;
  if(prca->NumberofCodedPFrame<=1000)
    prca->NumberofCodedPFrame++;
}



void __attribute__ ((section(".h264"))) my_updateRCModel ( unsigned int view_idx ) 
{
  int n_windowSize;
  int i,n_realSize;
  RC_DATA *prca = &rca[view_idx];
  
  int m_Nc = prca->NumberofCodedPFrame;
  Boolean MADModelFlag = FALSE;
//1  static Boolean m_rgRejected[RC_MODEL_HISTORY];
  int error_0p[RC_MODEL_HISTORY];
  unsigned int std_0p=0, threshold_0p;

  if(prca->bu_cnt==0)
    prca->codedbu_cnt = prca->TotalNumberofBasicUnit;
  else
    prca->codedbu_cnt = prca->bu_cnt;

  if( prca->type==P_SLICE || (((prca->type==I_SLICE)) && (!(prca->gop_cnt==0&&prca->frame_cnt==0))) ) //lhuitune
  {
    {
        //compute the MAD of the current bu
        prca->CurrentMAD_8p = prca->TotalMADBasicUnit/prca->BasicUnit;
        prca->TotalMADBasicUnit=0;

        // compute the average number of header bits
        prca->PAveHeaderBits1=(prca->PAveHeaderBits1*(prca->codedbu_cnt-1) + prca->NumberofBasicUnitHeaderBits)/prca->codedbu_cnt;
        if(prca->PAveHeaderBits3 == 0)
            prca->PAveHeaderBits2 = prca->PAveHeaderBits1;
        else
        {
            prca->PAveHeaderBits2 = (prca->PAveHeaderBits1*prca->codedbu_cnt +
                prca->PAveHeaderBits3*(prca->TotalNumberofBasicUnit-prca->codedbu_cnt))/prca->TotalNumberofBasicUnit;
        }

//s        *(pp_BUCFMAD_8p+prca->codedbu_cnt-1) = prca->CurrentMAD_8p;
        prca->BUCFMAD_8p[prca->codedbu_cnt-1] = prca->CurrentMAD_8p;

        if(prca->codedbu_cnt >= prca->TotalNumberofBasicUnit)
            m_Nc = prca->NumberofCodedPFrame * prca->TotalNumberofBasicUnit;
        else
            m_Nc = prca->NumberofCodedPFrame * prca->TotalNumberofBasicUnit + prca->codedbu_cnt;
    }

    if(m_Nc > 1)
      MADModelFlag=TRUE;

    // hold to over
    prca->rc_hold = 1;

    prca->m_rgQp_8p[0] = QP2Qstep_8p(prca->m_Qc); //*1.0/prc->CurrentMAD;

    if(prca->BasicUnit >= prca->FrameSizeInMbs) {//frame layer rate control
        if(prca->CurrentMAD_8p==0) {// added by lhumad
            prca->cmadequ0 = 1;
            prca->m_rgRp_8p[0] = (long long)prca->NumberofTextureBits<<16;
        }
        else {
            prca->cmadequ0 = 0;
            prca->m_rgRp_8p[0] = ((long long)prca->NumberofTextureBits<<16)/prca->CurrentMAD_8p;
        }
    }
    else {//basic unit layer rate control
        if(prca->CurrentMAD_8p==0) {// added by lhumad
            prca->cmadequ0 = 1;
            prca->m_rgRp_8p[0] = (long long)prca->NumberofBasicUnitTextureBits<<16;
        }
        else {
            prca->cmadequ0 = 0;
            //prca->Pm_rgRp[0] = prca->NumberofBasicUnitTextureBits*1.0/prca->CurrentMAD;
            prca->m_rgRp_8p[0] = ((long long)prca->NumberofBasicUnitTextureBits<<16)/prca->CurrentMAD_8p;
        }
    }

    prca->rc_tmp0[0] = (prca->m_rgQp_8p[0]>>4)*(prca->m_rgRp_8p[0]>>4);
    prca->rc_tmp1[0] = (1<<24)/(prca->m_rgQp_8p[0]>>4);
    prca->rc_tmp4[0] = (prca->m_rgQp_8p[0]>>4)*(prca->m_rgQp_8p[0]>>4);
    prca->rc_tmp2[0] = (1<<28)/prca->rc_tmp4[0];
    prca->m_rgRp_8prr8[0] = prca->m_rgRp_8p[0]>>8;
    prca->rc_tmp3[0] = (prca->m_rgQp_8p[0]>>8)*prca->m_rgRp_8prr8[0];;
    prca->m_X1_8p = prca->Pm_X1_8p;
    prca->m_X2_8p = prca->Pm_X2_8p;

    //compute the size of window
    //n_windowSize = (prca->CurrentMAD>prca->PreviousMAD)? (int)(prca->PreviousMAD/prca->CurrentMAD * (RC_MODEL_HISTORY-1))
    //    :(int)(prca->CurrentMAD/prca->PreviousMAD * (RC_MODEL_HISTORY-1));
    n_windowSize = (prca->CurrentMAD_8p>prca->PreviousMAD_8p)? ((prca->PreviousMAD_8p*20)/prca->CurrentMAD_8p):
        ((prca->CurrentMAD_8p*20)/prca->PreviousMAD_8p);

    n_windowSize=my_iClip3(1, m_Nc, n_windowSize);
    n_windowSize=my_imin(n_windowSize,prca->m_windowSize+1); // m_windowSize:: previous_windowsize
    n_windowSize=my_imin(n_windowSize,20);

    //update the previous window size
    prca->m_windowSize = n_windowSize;
    n_realSize = n_windowSize;

    // initial RD model estimator
    my_RCModelEstimator(view_idx, n_windowSize, n_windowSize, prca->rc_rgRejected);

    n_windowSize = prca->m_windowSize;
    // remove outlier

    for(i=0; i<n_windowSize; i++)
    {
//a     error_4p[i] = prca->m_X1_8p/prca->m_rgQp_8p[i] + (prca->m_X2_8p)/((prca->m_rgQp_8p[i]>>4)*(prca->m_rgQp_8p[i]>>4)) - (prca->m_rgRp_8p[i]>>8);
        error_0p[i] = prca->m_X1_8p/prca->m_rgQp_8p[i] + (prca->m_X2_8p/prca->rc_tmp4[i]) - prca->m_rgRp_8prr8[i];
        std_0p += error_0p[i]*error_0p[i];
    }

    threshold_0p = (n_windowSize==2)? 0:my_sqrt32(std_0p/n_windowSize);

    for(i=1;i<n_windowSize;i++)
    {
      if(abs(error_0p[i]) > threshold_0p)
      {
        prca->rc_rgRejected[i] = TRUE;
        n_realSize--;
      }
    }
    // always include the last data point
//1    prca->rc_rgRejected[0] = FALSE;

    // second RD model estimator
    my_RCModelEstimator(view_idx, n_realSize, n_windowSize, prca->rc_rgRejected);

    if( MADModelFlag )
      my_updateMADModel( view_idx );
    else if(prca->type==P_SLICE)
      prca->PPictureMAD_8p = prca->CurrentMAD_8p;
  }
}


void __attribute__ ((section(".h264"))) my_RCModelEstimator (unsigned int view_idx, int n_realSize, int n_windowSize, char *rc_rgRejected) 
{
  int i;
  Boolean estimateX2 = FALSE;
  unsigned int  a00_20p=0,a01_20p=0,a11_20p=0,b0_0p=0,b1_0p=0;
  long long  MatrixValue_20p;
  int sum_rc_tmp0=0;
  RC_DATA *prca = &rca[view_idx];

    // default RD model estimation results
    prca->m_X1_8p = 0;
    prca->m_X2_8p = 0;

    for(i=0;i<n_windowSize;i++) // if all non-rejected Q are the same, take 1st order model
    {
        if(!rc_rgRejected[i])
        {
            if((prca->m_rgQp_8p[i]!=prca->m_rgQp_8p[0]))
            {
                estimateX2 = TRUE;
                break;
            }
            sum_rc_tmp0 += prca->rc_tmp0[i]; // ((prca->m_rgQp_8p[i]>>4) * (prca->m_rgRp_8p[i]>>4));
        }
    }
    if(estimateX2==FALSE)
        prca->m_X1_8p = sum_rc_tmp0/n_realSize;


  // take 2nd order model to estimate X1 and X2
  if(estimateX2)
  {
    a00_20p = n_realSize<<20;
    for (i = 0; i < n_windowSize; i++)
    {
      if (!rc_rgRejected[i])
      {
        a01_20p += prca->rc_tmp1[i];
        a11_20p += prca->rc_tmp2[i];
        b0_0p   += prca->rc_tmp3[i];
        b1_0p   += prca->m_rgRp_8prr8[i];
      }
    }
    MatrixValue_20p = (((long long)a00_20p*(long long)a11_20p)-((long long)a01_20p*(long long)a01_20p)+(1<<19))>>20;
    if(MatrixValue_20p > 1)
    {
      prca->m_X1_8p = (((long long)b0_0p*(long long)a11_20p - (long long)b1_0p*(long long)a01_20p)<<8)/MatrixValue_20p;
      prca->m_X2_8p = (((long long)b1_0p*(long long)a00_20p - (long long)b0_0p*(long long)a01_20p)<<8)/MatrixValue_20p;
    }
    else
    {
      prca->m_X1_8p = (b0_0p<<8)/(a00_20p>>20);
      prca->m_X2_8p = 0;
    }
  }

  if( prca->type==P_SLICE || (((prca->type==I_SLICE)) && (!(prca->gop_cnt==0&&prca->frame_cnt==0))) ) //lhuitune
  {
    prca->Pm_X1_8p = prca->m_X1_8p;
    prca->Pm_X2_8p = prca->m_X2_8p;
  }
}


void __attribute__ ((section(".h264"))) my_updateMADModel( unsigned int view_idx ) 
{
  RC_DATA *prca = &rca[view_idx];
  int    n_windowSize;
  int    i, n_realSize;
  int    m_Nc = prca->NumberofCodedPFrame;
  static int error_8p[RC_MODEL_HISTORY];
  long long std_16p=0;
  int threshold_8p;
  int MADPictureC2_12prr4;

  if(prca->NumberofCodedPFrame>0)
  {
    //frame layer rate control
    if(prca->BasicUnit >= prca->FrameSizeInMbs)
      m_Nc = prca->NumberofCodedPFrame;
    else // basic unit layer rate control
      m_Nc=prca->NumberofCodedPFrame*prca->TotalNumberofBasicUnit+prca->codedbu_cnt; //prca->CodedBasicUnit;

    // hold to over
    prca->mad_hold=1;

    prca->PPictureMAD_8p = prca->CurrentMAD_8p;
    prca->PictureMAD_8p[0]  = prca->PPictureMAD_8p;

    if(prca->BasicUnit >= prca->FrameSizeInMbs)
        prca->ReferenceMAD_8p[0]=prca->PictureMAD_8p[1];
    else
        prca->ReferenceMAD_8p[0]=prca->BUPFMAD_8p[prca->codedbu_cnt-1];
//s        prca->ReferenceMAD_8p[0] = *(pp_BUPFMAD_8p+prca->codedbu_cnt-1);

    if(prca->ReferenceMAD_8p[0] == 0)
    {
        prca->mad_tmp0_valid[0] = 0;
        prca->mad_tmp0[0] = 0;
    }
    else
    {
        prca->mad_tmp0_valid[0] = 1;
        prca->mad_tmp0[0] = (prca->PictureMAD_8p[0]<<12)/prca->ReferenceMAD_8p[0];
    }
    prca->mad_tmp1[0] = (prca->ReferenceMAD_8p[0]>>4)*(prca->ReferenceMAD_8p[0]>>4);
    prca->mad_tmp2[0] = (prca->PictureMAD_8p[0]>>4)*(prca->ReferenceMAD_8p[0]>>4);


    prca->MADPictureC1_12p = prca->PMADPictureC1_12p;
    prca->MADPictureC2_12p = prca->PMADPictureC2_12p;

    //compute the size of window
    //n_windowSize = (prca->CurrentMAD>prca->PreviousMAD)? (int)((float)(RC_MODEL_HISTORY-1) * prca->PreviousMAD/prca->CurrentMAD)
    //    :(int)((float)(RC_MODEL_HISTORY-1) * prca->CurrentMAD/prca->PreviousMAD);
    n_windowSize = (prca->CurrentMAD_8p>prca->PreviousMAD_8p)? ((20*prca->PreviousMAD_8p)/prca->CurrentMAD_8p)
        :((20*prca->CurrentMAD_8p)/prca->PreviousMAD_8p);

    n_windowSize = my_iClip3(1, (m_Nc-1), n_windowSize);
    n_windowSize = my_imin(n_windowSize, my_imin(20, prca->MADm_windowSize+1));

    //update the previous window size
    prca->MADm_windowSize=n_windowSize;


    //update the MAD for the previous frame
    if( prca->type==P_SLICE || (((prca->type==I_SLICE)) && (!(prca->gop_cnt==0&&prca->frame_cnt==0))) ) {//lhuitune
      if (prca->CurrentMAD_8p==0) 
	  	prca->PreviousMAD_8p=1;// lhumad, make fake for dividing by zero when PreviousMAD equal to 0
      else                         
	  	prca->PreviousMAD_8p = prca->CurrentMAD_8p;
    }

    // initial MAD model estimator
    my_MADModelEstimator (view_idx, n_windowSize, n_windowSize, prca->mad_rgRejected);

    MADPictureC2_12prr4 = prca->MADPictureC2_12p>>4;
    // remove outlier
    for (i = 0; i < n_windowSize; i++)
    {
      //error[i] = prca->MADPictureC1 * prca->ReferenceMAD[i] + prca->MADPictureC2 - prca->PictureMAD[i];
      error_8p[i] = ((prca->MADPictureC1_12p*prca->ReferenceMAD_8p[i])>>12) + MADPictureC2_12prr4 - prca->PictureMAD_8p[i];
      std_16p += error_8p[i]*error_8p[i];
    }

    threshold_8p = (n_windowSize==2)? 0:my_sqrt64(std_16p/n_windowSize);

    n_realSize = n_windowSize;
    for(i=1; i<n_windowSize; i++)
    {
      if(abs(error_8p[i]) > threshold_8p)
      {
        prca->mad_rgRejected[i] = TRUE;
        n_realSize--;
      }
    }

    // second MAD model estimator
    my_MADModelEstimator(view_idx, n_realSize, n_windowSize, prca->mad_rgRejected);
  }
}


void __attribute__ ((section(".h264"))) my_MADModelEstimator(unsigned int view_idx, int n_realSize, int n_windowSize, char *mad_rgRejected) 
{
  RC_DATA *prca = &rca[view_idx];
  int     i;
  long long MatrixValue_20p; // change 4p to 20p, lhu, 2017/02/23
  Boolean estimateX2=FALSE;
  unsigned int a00_20p=0,a01_20p=0,a11_20p=0,b0_8p=0,b1_8p=0; // change 8p to 20p, lhu, 2017/02/23

    // default MAD model estimation results
    prca->MADPictureC1_12p = 0;
    prca->MADPictureC2_12p = 0;
    prca->c1_over = 0;

    for(i=0;i<n_windowSize;i++) // if all non-rejected MAD are the same, take 1st order model
    {
        if(!mad_rgRejected[i])
        {
            if(prca->PictureMAD_8p[i]!=prca->PictureMAD_8p[0])
            {
                estimateX2 = TRUE;
                    break;
            }
            prca->MADPictureC1_12p += prca->mad_tmp0[i]; // ((prca->PictureMAD_8p[i]<<12) / prca->ReferenceMAD_8p[i]) /n_realSize;
            if(prca->mad_tmp0_valid[i] == 0)
                prca->c1_over = 1;
        }
    }
    if(estimateX2==FALSE)
        prca->MADPictureC1_12p = prca->MADPictureC1_12p/n_realSize;

    // take 2nd order model to estimate X1 and X2
    if(estimateX2)
    {
        a00_20p = n_realSize<<20; // change 8 to 20, lhu, 2017/02/23
        for(i=0;i<n_windowSize;i++)
        {
            if(!mad_rgRejected[i])
            {
                a01_20p += (prca->ReferenceMAD_8p[i]<<12); // change 8p to 20p, lhu, 2017/02/23
                a11_20p += (prca->mad_tmp1[i]<<12); // change 8p to 20p, lhu, 2017/02/23
                b0_8p  += prca->PictureMAD_8p[i];
                b1_8p  += prca->mad_tmp2[i]; // (prca->PictureMAD_8p[i]>>4)*(prca->ReferenceMAD_8p[i]>>4);
            }
        }
        // solve the equation of AX = B
        MatrixValue_20p = ((long long)a00_20p*(long long)a11_20p - (long long)a01_20p*(long long)a01_20p + (1<<19))>>20; // change 4p to 20p, lhu, 2017/02/23

        //if(MatrixValue_4p != 0)  //if(fabs(MatrixValue) > 0.000001)
        if(abs(MatrixValue_20p) > 1)  // change 4p to 20p, lhu, 2017/02/23
        {
            prca->MADPictureC2_12p = (((long long)b0_8p*(long long)a11_20p - (long long)b1_8p*(long long)a01_20p)<<4)/MatrixValue_20p;
            prca->MADPictureC1_12p = (((long long)b1_8p*(long long)a00_20p - (long long)b0_8p*(long long)a01_20p)<<4)/MatrixValue_20p;
        }
        else
        {
            if (a01_20p==0) {// lhumad, make fake for dividing by zero when a01_20p equal to 0
                prca->MADPictureC1_12p = ((long long)b0_8p)<<4;
                prca->cmadequ0 = 1;
            }
            else {
                prca->MADPictureC1_12p = (((long long)b0_8p)<<24)/(long long)a01_20p; // lhu, 2017/02/23
                prca->cmadequ0 = 0;
            }
            prca->MADPictureC2_12p = 0;
        }
        prca->c1_over = 0;
    }
    if( prca->type==P_SLICE || (((prca->type==I_SLICE)) && (!(prca->gop_cnt==0&&prca->frame_cnt==0))) ) //lhuitune
    {
        prca->PMADPictureC1_12p = prca->MADPictureC1_12p;
        prca->PMADPictureC2_12p = prca->MADPictureC2_12p;
    }
}


void __attribute__ ((section(".h264"))) my_hold( unsigned int view_idx ) 
{
    RC_DATA *prca = &rca[view_idx];
	
    int i;
    if(prca->rc_hold==1)
    {
        for(i=(RC_MODEL_HISTORY-2); i>0; i--)
        {// update the history
            prca->m_rgQp_8p[i] = prca->m_rgQp_8p[i-1];
            prca->m_rgRp_8p[i] = prca->m_rgRp_8p[i-1];
            prca->rc_tmp0[i] = prca->rc_tmp0[i-1];
            prca->rc_tmp1[i] = prca->rc_tmp1[i-1];
            prca->rc_tmp2[i] = prca->rc_tmp2[i-1];
            prca->rc_tmp3[i] = prca->rc_tmp3[i-1];
            prca->rc_tmp4[i] = prca->rc_tmp4[i-1];
            prca->m_rgRp_8prr8[i] = prca->m_rgRp_8prr8[i-1];
        }
        for(i=0; i<(RC_MODEL_HISTORY-1); i++)
            prca->rc_rgRejected[i] = FALSE;

        prca->rc_hold=0;
    }

    if(prca->mad_hold==1)
    {
        for(i=(RC_MODEL_HISTORY-2);i>0;i--)
        {// update the history
            prca->PictureMAD_8p[i] = prca->PictureMAD_8p[i-1];
            prca->ReferenceMAD_8p[i] = prca->ReferenceMAD_8p[i-1];
            prca->mad_tmp0[i] = prca->mad_tmp0[i-1];
            prca->mad_tmp0_valid[i] = prca->mad_tmp0_valid[i-1];
            prca->mad_tmp1[i] = prca->mad_tmp1[i-1];
            prca->mad_tmp2[i] = prca->mad_tmp2[i-1];
        }
        for(i=0; i<(RC_MODEL_HISTORY-1); i++)
            prca->mad_rgRejected[i] = FALSE;

        prca->mad_hold=0;
    }
}


//////////////////////////////////////////////////////////////////////////////////////
// \brief
//    compute a  quantization parameter for each frame
//////////////////////////////////////////////////////////////////////////////////////
int __attribute__ ((section(".h264"))) my_updateQPRC3( unsigned int view_idx ) 
{
  RC_DATA *prca = &rca[view_idx];
  int m_Bits;
  int SumofBasicUnit;
  int MaxQpChange, m_Qp, m_Hp;
  int last_qp;
  last_qp = prca->m_Qc;

  {
    if(prca->gop_cnt==0 && prca->frame_cnt==0) // (prca->number == 0)
    {
      prca->m_Qc 		= prca->MyInitialQp;
	  prca->TotalIQp 	+= prca->m_Qc;
      return prca->m_Qc;
    }
    //else if( prca->type == P_SLICE )
    else if( prca->type == I_SLICE )
    {
    	if( prca->bu_cnt == 0 )
    	{
    		prca->TotalIQp = prca->LastIQp;
    		return prca->LastIQp;
    	}
		else {
			unsigned int totalbits = prca->NumberofBasicUnitHeaderBits + prca->NumberofBasicUnitTextureBits;
			if( totalbits > ( ( prca->TargetForBasicUnit * 307) >> 8 )  )
			{
				prca->m_Qc = prca->m_Qc + 2;
			}
			else if( totalbits > ( ( prca->TargetForBasicUnit * 282) >> 8 ) )
			{
				prca->m_Qc = prca->m_Qc + 1;
			}
			else if( totalbits < ( ( prca->TargetForBasicUnit * 204) >> 8 ) )
			{
				prca->m_Qc = prca->m_Qc - 2;
			}
			else if( totalbits < ( ( prca->TargetForBasicUnit * 230) >> 8 ) )
			{
				prca->m_Qc = prca->m_Qc - 1;
			}
				
			prca->Target -= (prca->NumberofBasicUnitHeaderBits + prca->NumberofBasicUnitTextureBits);
          	prca->NumberofBasicUnitHeaderBits  = 0;
          	prca->NumberofBasicUnitTextureBits = 0;

			if( prca->Target < 0 )
				prca->Target = 0;

			prca->TargetForBasicUnit = ( (prca->TotalNumberofBasicUnit - prca->bu_cnt) == 0 ) ? 0 :  prca->Target / (prca->TotalNumberofBasicUnit - prca->bu_cnt);

			prca->m_Qc = my_iClip3( prca->LastIQp - 3, 	prca->LastIQp + 3, 	prca->m_Qc);
			prca->m_Qc = my_iClip3( prca->RCMinQP, 		prca->RCMaxQP, 		prca->m_Qc);

     		prca->TotalIQp += prca->m_Qc;
			
			return prca->m_Qc;
		}
			
    }
    else if( prca->type == P_SLICE ) //lhuitune
    {
      if(prca->gop_cnt==0 && prca->frame_cnt==1) // ((prca->NumberofGOP==1)&&(prca->NumberofPPicture==0)) // gop==0; frameP==0
      {
          return my_updateFirstP( view_idx  );
      }
      else
      {
        prca->m_X1_8p = prca->Pm_X1_8p;
        prca->m_X2_8p = prca->Pm_X2_8p;
        prca->MADPictureC1_12p=prca->PMADPictureC1_12p;
        prca->MADPictureC2_12p=prca->PMADPictureC2_12p;

        m_Qp=prca->Pm_Qp;

        SumofBasicUnit=prca->TotalNumberofBasicUnit;

        if(prca->bu_cnt==0) //(prca->NumberofBasicUnit==SumofBasicUnit)
          return my_updateFirstBU( view_idx );
        else
        {
          unsigned int totalbits = prca->NumberofBasicUnitHeaderBits + prca->NumberofBasicUnitTextureBits;
          /*compute the number of remaining bits*/
          prca->Target -= (prca->NumberofBasicUnitHeaderBits + prca->NumberofBasicUnitTextureBits);
          prca->NumberofBasicUnitHeaderBits  = 0;
          prca->NumberofBasicUnitTextureBits = 0;

          if(prca->Target<0)
            return my_updateNegativeTarget(view_idx, m_Qp );
          else
          {
            /*predict the MAD of current picture*/
            my_predictCurrPicMAD( view_idx );

            /*compute the total number of bits for the current basic unit*/
            my_updateModelQPBU( view_idx, m_Qp );

			if( totalbits > ( ( prca->TargetForBasicUnit * 282) >> 8 ) )
			{
				if( prca->m_Qc < last_qp  )
					prca->m_Qc = last_qp;
			}
			else  if( totalbits < ( ( prca->TargetForBasicUnit * 230) >> 8 ) )
			{
				if( prca->m_Qc > last_qp )
					prca->m_Qc = last_qp;
          	}

			prca->TargetForBasicUnit = ( (prca->TotalNumberofBasicUnit - prca->bu_cnt) == 0 ) ? 0 :  prca->Target / (prca->TotalNumberofBasicUnit - prca->bu_cnt);
			
            prca->TotalFrameQP +=prca->m_Qc;
            prca->Pm_Qp=prca->m_Qc;
            if((prca->bu_cnt==(prca->TotalNumberofBasicUnit-1)) && prca->type==P_SLICE) // lhu, 2017/03/23
            //if((prca->bu_cnt==(prca->TotalNumberofBasicUnit-1)) && (prca->type==P_SLICE || prca->type==I_SLICE) ) //lhuitune
              my_updateLastBU( view_idx );

            return prca->m_Qc;
          }
        }
      }
    }
  }
  return prca->m_Qc;
}


void __attribute__ ((section(".h264"))) my_updateQPNonPicAFF( unsigned int view_idx ) 
{
    RC_DATA *prca = &rca[view_idx];
    prca->TotalQpforPPicture +=prca->m_Qc;
    prca->Pm_Qp=prca->m_Qc;
}


int __attribute__ ((section(".h264"))) my_updateFirstP( unsigned int view_idx ) 
{
  RC_DATA *prca = &rca[view_idx];
  //top field of the first P frame
  prca->m_Qc=prca->MyInitialQp;
  prca->NumberofBasicUnitHeaderBits=0;
  prca->NumberofBasicUnitTextureBits=0;
  //bottom field of the first P frame
  if(prca->bu_cnt==(prca->TotalNumberofBasicUnit-1)) //(prca->NumberofBasicUnit==0)
  {
    prca->TotalQpforPPicture +=prca->m_Qc;
    prca->PAveFrameQP=prca->m_Qc;
    prca->PAveHeaderBits3=prca->PAveHeaderBits2;
  }
  prca->Pm_Qp = prca->m_Qc;
  prca->TotalFrameQP += prca->m_Qc;
  return prca->m_Qc;
}


int __attribute__ ((section(".h264"))) my_updateNegativeTarget( unsigned int view_idx, int m_Qp ) 
{
  RC_DATA *prca = &rca[view_idx];
  int PAverageQP;

  if(prca->GOPOverdue==TRUE)
    prca->m_Qc=m_Qp+2;
  else
    prca->m_Qc=m_Qp+prca->DDquant;//2

  prca->m_Qc = my_imin(prca->m_Qc, prca->RCMaxQP);  // clipping
  if(prca->BasicUnit >= prca->MBPerRow) {
    if (prca->wireless_screen!=1) { // added by lhu, 2017/02/27
      if (prca->type == P_SLICE) prca->m_Qc = my_imin(prca->PAveFrameQP+6, prca->m_Qc); // change +6 to +10, lhu, 2017/01/26
      else                        prca->m_Qc = my_imin(prca->PAveFrameQP+5, prca->m_Qc); // lower QP change range for I slice, lhu, 2017/02/07
    } else {
      if (prca->type == P_SLICE) prca->m_Qc = my_imin(prca->PAveFrameQP+3, prca->m_Qc); // change +6 to +3, lhu, 2017/04/25
      else                        prca->m_Qc = my_imin(prca->PAveFrameQP+2, prca->m_Qc); // change +6 to +2 for I slice, lhu, 2017/04/25
    }
  } else
    prca->m_Qc = my_imin(prca->m_Qc, prca->PAveFrameQP+3);

  prca->TotalFrameQP +=prca->m_Qc;
  if(prca->bu_cnt==(prca->TotalNumberofBasicUnit-1)) //(prca->NumberofBasicUnit==0)
  {
//18    PAverageQP=(int)((double)prca->TotalFrameQP/(double)prca->TotalNumberofBasicUnit+0.5);
    PAverageQP=(prca->TotalFrameQP+(prca->TotalNumberofBasicUnit>>1))/prca->TotalNumberofBasicUnit;
    if(prca->frame_cnt==(prca->intra_period-1)) //(prca->NumberofPPicture == (prca->intra_period - 2))
      prca->QPLastPFrame = PAverageQP;
    if (prca->type == P_SLICE) // not increase TotalQpforPPicture for I_SLICE, lhuitune
    {
      prca->TotalQpforPPicture +=PAverageQP;
	  //DLOG_Error("%d\n", prca->TotalQpforPPicture);
    }
	
	if( PAverageQP < prca->PAveFrameQP )
		prca->PAveFrameQP = prca->PAveFrameQP;
	else
    	prca->PAveFrameQP=PAverageQP;
	
    prca->PAveHeaderBits3=prca->PAveHeaderBits2;
  }
  if(prca->GOPOverdue==TRUE)
    prca->Pm_Qp=prca->PAveFrameQP;
  else
    prca->Pm_Qp=prca->m_Qc;

  return prca->m_Qc;
}


int __attribute__ ((section(".h264"))) my_updateFirstBU( unsigned int view_idx ) 
{
  RC_DATA *prca = &rca[view_idx];
  //	if(prca->frame_cnt==1)  prca->PAveFrameQP = prca->QPLastPFrame; // first P frame's initial QP value equals to LastPFrame's average QP, lhu, 2017/03/23
  //	else                    prca->PAveFrameQP = prca->PAveFrameQP;
  if(prca->Target<=0)
  {
    prca->m_Qc = prca->PAveFrameQP + 2;
    if(prca->m_Qc > prca->RCMaxQP)
      prca->m_Qc = prca->RCMaxQP;

    prca->GOPOverdue=TRUE;
  }
  else
  {
    prca->m_Qc=prca->PAveFrameQP;
  }
  prca->TotalFrameQP +=prca->m_Qc;
  prca->Pm_Qp = prca->PAveFrameQP;

  return prca->m_Qc;
}


void __attribute__ ((section(".h264"))) my_updateLastBU( unsigned int view_idx ) 
{
  RC_DATA *prca = &rca[view_idx];
  int PAverageQP;

//18  PAverageQP=(int)((double)prca->TotalFrameQP/(double)prca->TotalNumberofBasicUnit+0.5);
  PAverageQP=(prca->TotalFrameQP+(prca->TotalNumberofBasicUnit>>1))/prca->TotalNumberofBasicUnit;
  if(prca->frame_cnt==(prca->intra_period-1)) // (prca->NumberofPPicture == (prca->intra_period - 2))  last P_FRAME in gop
    prca->QPLastPFrame = PAverageQP;
  if (prca->type == P_SLICE) { // not increase TotalQpforPPicture for I_SLICE, lhuitune
    prca->TotalQpforPPicture +=PAverageQP;
	//DLOG_Error("%d\n", prca->TotalQpforPPicture);
  }
  prca->PAveFrameQP=PAverageQP;
  prca->PAveHeaderBits3=prca->PAveHeaderBits2;
}


void __attribute__ ((section(".h264"))) my_updateModelQPFrame( unsigned int view_idx, int m_Bits ) 
{
  RC_DATA *prca = &rca[view_idx];
  long long dtmp_8p, qstep_tmp;
  int tmp_4p=0;
  int m_Qstep_8p;

  //dtmp_8p = (prca->CurrentMAD_8p>>6)*(prca->m_X1_8p>>6)*(prca->CurrentMAD_8p>>6)*(prca->m_X1_8p>>6) + \
  //    4*(prca->m_X2_8p>>4)*(prca->CurrentMAD_8p>>4)*m_Bits;
  dtmp_8p = ((long long)prca->CurrentMAD_8p>>6)*((long long)prca->CurrentMAD_8p>>6)*((long long)prca->m_X1_8p>>6)*((long long)prca->m_X1_8p>>6) + \
      4*((long long)prca->m_X2_8p>>4)*((long long)prca->CurrentMAD_8p>>4)*m_Bits;

  if(dtmp_8p>0)
      tmp_4p = my_sqrt64(dtmp_8p);

  if((prca->m_X2_8p==0) || (dtmp_8p<0) || ((tmp_4p-((prca->m_X1_8p>>6)*(prca->CurrentMAD_8p>>6)))<=0))
  {
    //m_Qstep = (float)((prca->m_X1*prca->CurrentMAD) / (double) m_Bits);
    m_Qstep_8p = ((prca->m_X1_8p>>4)*(prca->CurrentMAD_8p>>4)) / m_Bits;
  }
  else // 2nd order mode
  {
    //m_Qstep = (float)((2*prca->m_X2_8p*prca->CurrentMAD_8p)/(sqrt(dtmp)*(1<<16) - prca->m_X1_8p*prca->CurrentMAD_8p));
    qstep_tmp = (2*((long long)prca->m_X2_8p)*((long long)prca->CurrentMAD_8p)) / ((tmp_4p<<4) - (prca->m_X1_8p>>4)*(prca->CurrentMAD_8p>>4));
    m_Qstep_8p = qstep_tmp;
  }

  prca->m_Qc = Qstep2QP_8p(m_Qstep_8p);
}


void __attribute__ ((section(".h264"))) my_predictCurrPicMAD( unsigned int view_idx ) 
{
    int i,CurrentBUMAD_8p,MADPictureC1_12prr4,MADPictureC2_12prr4;
	RC_DATA *prca = &rca[view_idx];

    MADPictureC1_12prr4 = prca->MADPictureC1_12p>>4;
    MADPictureC2_12prr4 = prca->MADPictureC2_12p>>4;

    //prca->CurrentMAD=prca->MADPictureC1*prca->BUPFMAD[prca->bu_cnt]+prca->MADPictureC2;
    prca->CurrentMAD_8p=(MADPictureC1_12prr4*(prca->BUPFMAD_8p[prca->bu_cnt]>>8)) + MADPictureC2_12prr4;
    prca->TotalBUMAD_12p=0;

    for(i=prca->TotalNumberofBasicUnit-1; i>=prca->bu_cnt; i--)
    {
        //CurrentBUMAD = prca->MADPictureC1*prca->BUPFMAD[i]+prca->MADPictureC2;
        CurrentBUMAD_8p = (MADPictureC1_12prr4*(prca->BUPFMAD_8p[i]>>8)) + MADPictureC2_12prr4;
        prca->TotalBUMAD_12p += (CurrentBUMAD_8p*CurrentBUMAD_8p)>>4;
    }
}


void __attribute__ ((section(".h264"))) my_updateModelQPBU( unsigned int view_idx, int m_Qp ) 
{
  RC_DATA *prca = &rca[view_idx];
  int m_Bits;
  long long dtmp_8p,qstep_tmp;
  int tmp_4p=0;
  int m_Qstep_8p;

  //compute the total number of bits for the current basic unit
  //m_Bits =(int)(prca->Target * prca->CurrentMAD * prca->CurrentMAD / prca->TotalBUMAD);
  if((prca->TotalBUMAD_12p>>8) == 0)
    m_Bits = prca->Target;
  else
    m_Bits =(prca->Target*(prca->CurrentMAD_8p>>6)*(prca->CurrentMAD_8p>>6)) / (prca->TotalBUMAD_12p>>8);

  //compute the number of texture bits
  m_Bits -=prca->PAveHeaderBits2;

  m_Bits=my_imax(m_Bits,((prca->bit_rate/prca->framerate)/(MINVALUE*prca->TotalNumberofBasicUnit)));

  //dtmp = prca->CurrentMAD*prca->CurrentMAD*prca->m_X1*prca->m_X1 + 4*prca->m_X2*prca->CurrentMAD*m_Bits;
  dtmp_8p = ((long long)prca->CurrentMAD_8p>>6)*((long long)prca->CurrentMAD_8p>>6)*((long long)prca->m_X1_8p>>6)*((long long)prca->m_X1_8p>>6) + \
      4*((long long)prca->m_X2_8p>>4)*((long long)prca->CurrentMAD_8p>>4)*m_Bits;

  if(dtmp_8p>0)
    tmp_4p = my_sqrt64(dtmp_8p);

  //if((prca->m_X2==0) || (dtmp<0) || ((sqrt(dtmp)-(prca->m_X1*prca->CurrentMAD))<=0))  // fall back 1st order mode
  if((prca->m_X2_8p==0) || (dtmp_8p<0) || ((tmp_4p-((prca->m_X1_8p>>6)*(prca->CurrentMAD_8p>>6)))<=0))
  {
    //m_Qstep = (float)((prca->m_X1*prca->CurrentMAD) / (double) m_Bits);
    m_Qstep_8p = ((prca->m_X1_8p>>4)*(prca->CurrentMAD_8p>>4)) / m_Bits;
  }
  else // 2nd order mode
  {
      //m_Qstep = (float)((2*prca->m_X2_8p*prca->CurrentMAD_8p)/(sqrt(dtmp)*(1<<16) - prca->m_X1_8p*prca->CurrentMAD_8p));
      qstep_tmp = (2*((long long)prca->m_X2_8p)*((long long)prca->CurrentMAD_8p)) / ((tmp_4p<<4) - (prca->m_X1_8p>>4)*(prca->CurrentMAD_8p>>4));
      m_Qstep_8p = qstep_tmp;
  }

  prca->m_Qc = Qstep2QP_8p(m_Qstep_8p);
  //use the Qc by R-D model when non-wireless-screen application, lhu, 2017/02/27
  if (prca->wireless_screen==1) // added by lhu, 2017/02/27
    prca->m_Qc = my_imin(m_Qp+prca->DDquant,  prca->m_Qc); // control variation
  
  if(prca->BasicUnit >= prca->MBPerRow) {
    if (prca->wireless_screen!=1) { // added by lhu, 2017/02/27
      if (prca->type == P_SLICE) prca->m_Qc = my_imin(prca->PAveFrameQP+6, prca->m_Qc); // change +6 to +10, lhu, 2017/01/24
      else                        prca->m_Qc = my_imin(prca->PAveFrameQP+5, prca->m_Qc); // lower QP change range for I slice, lhu, 2017/02/07
    } else {
      if (prca->type == P_SLICE) prca->m_Qc = my_imin(prca->PAveFrameQP+3, prca->m_Qc); // change +6 to +3, lhu, 2017/04/25
      else {
			// Expand QP change range when decide to insert I Frame, lhu, 2017/05/26
			if (prca->IFduration==1 && prca->insertOneIFrame==1) prca->m_Qc = my_imin(prca->PAveFrameQP+6, prca->m_Qc);
			else												 prca->m_Qc = my_imin(prca->PAveFrameQP+2, prca->m_Qc); // change +6 to +2 for I slice, lhu, 2017/04/25
	  }
    }
  } else
    prca->m_Qc = my_imin(prca->PAveFrameQP+3, prca->m_Qc);

  /*if(prca->c1_over==1)
    //prca->m_Qc = my_imin(m_Qp-prca->DDquant, prca->RCMaxQP); // clipping
    prca->m_Qc = my_imin(m_Qp+prca->DDquant, prca->RCMaxQP-10); // not letting QP decrease when MAD equal 0, 2017/02/21
  else*/
    prca->m_Qc = my_iClip3(m_Qp-prca->DDquant, prca->RCMaxQP, prca->m_Qc); // clipping

  if(prca->BasicUnit >= prca->MBPerRow) {
    if (prca->wireless_screen!=1) { // added by lhu, 2017/04/18
      if (prca->type == P_SLICE) prca->m_Qc = my_imax(prca->PAveFrameQP-6, prca->m_Qc); // lhu, 2017/04/18
      else                        prca->m_Qc = my_imax(prca->PAveFrameQP-5, prca->m_Qc); // lhu, 2017/04/18
    } else {
      if (prca->type == P_SLICE) prca->m_Qc = my_imax(prca->PAveFrameQP-3, prca->m_Qc); // lhu, 2017/04/25
      else {
        // Expand QP change range when decide to insert I Frame, lhu, 2017/05/26
        if (prca->IFduration==1 && prca->insertOneIFrame==1) prca->m_Qc = my_imax(prca->PAveFrameQP-6, prca->m_Qc);
      	else                                                 prca->m_Qc = my_imax(prca->PAveFrameQP-2, prca->m_Qc); // lhu, 2017/04/25
      }
    }
  } else
    prca->m_Qc = my_imax(prca->PAveFrameQP-3, prca->m_Qc);

  prca->m_Qc = my_imax(prca->RCMinQP, prca->m_Qc);
}


void __attribute__ ((section(".h264"))) my_rc_update_bu_stats( unsigned int view_idx ) 
{
    RC_DATA *prca = &rca[view_idx];
    
    prca->NumberofHeaderBits  = prca->frame_hbits;
    prca->NumberofTextureBits = prca->frame_tbits;
    // basic unit layer rate control
    if(prca->BasicUnit < prca->FrameSizeInMbs) {
        prca->NumberofBasicUnitHeaderBits  = prca->hbits_tmp;  // add slice_header
        prca->NumberofBasicUnitTextureBits = prca->tbits_tmp;
    }
}
void __attribute__ ((section(".h264"))) my_rc_update_frame_stats( unsigned int view_idx ) 
{
	RC_DATA *prca = &rca[view_idx];
    prca->frame_mad   += prca->mad_tmp;
    prca->frame_tbits += prca->tbits_tmp;
    prca->frame_hbits += prca->hbits_tmp;
    prca->frame_abits = prca->frame_tbits+prca->frame_hbits;
    if(prca->bu_cnt==0) { //after calculate frame's status reset related status to zero, lhu, 2017/03/06
        prca->frame_mad   = 0;
        prca->frame_tbits = 0;
        prca->frame_hbits = 0;
        prca->frame_abits = 0;
    }
}



void __attribute__ ((section(".h264"))) my_rc_init_gop_params( unsigned int view_idx ) 
{
	RC_DATA *prca = &rca[view_idx];

    {
        if(prca->frame_cnt==0 && prca->bu_cnt==0) {
            if ((prca->IFduration==1 && prca->insertOneIFrame==1) || (prca->Iframe_causedby_scene_change == 1 || prca->insert_i_frame == 1) )
                prca->intra_period = prca->PrevIntraPeriod; // use previous intra_period to calculate GOP TargetBits, lhu, 2017/03/13
            my_rc_init_GOP( view_idx, prca->intra_period - 1 );
        }
    }
}


int __attribute__ ((section(".h264"))) my_rc_handle_mb( unsigned int view_idx ) 
{    
    RC_DATA *prca = &rca[view_idx];
	unsigned int regVal;
    //// first update, update MB_state
    if(prca->gop_cnt!=0 || prca->frame_cnt!=0 || prca->bu_cnt!=0)
    {
        my_rc_update_bu_stats( view_idx ); 
        prca->TotalMADBasicUnit = prca->mad_tmp;
        prca->TotalFrameMAD    += prca->mad_tmp;// lhumod
    }


    if((prca->gop_cnt>0 || prca->frame_cnt>0) && prca->bu_cnt==0) {
        //prca->frame_mad = prca->TotalFrameMAD/prca->FrameSizeInMbs;// lhumod, calculate the average MB's mad value of previous encoded frame.
        my_rc_update_pict( view_idx, prca->fbits_tmp );  // should put it to the frame-last
    }

    //// initial sequence (only once)
    if( prca->bu_cnt == 0 ) // first bu
    {
        if( prca->frame_cnt == 0 ) // first frame
        {
            prca->type = I_SLICE;
            if( prca->gop_cnt == 0 ) // first gop, sequence need set parameters , @jlliu
            {
                my_rc_params    ( view_idx );
                my_rc_init_seq  ( view_idx ); //// initial seq params
            }
            
            my_rc_init_gop_params   ( view_idx );
        }
        else {
            prca->type = P_SLICE;
        }
        
        my_rc_init_pict(view_idx, 1);
        prca->qp        = my_updateQP( view_idx );
        prca->slice_qp  = prca->qp;
    }

    
    // frame layer rate control //// BU-Level
    if (prca->BasicUnit < prca->FrameSizeInMbs)
    {
        // each I or B frame has only one QP
        if(prca->gop_cnt==0 && prca->frame_cnt==0) //lhuitune
        {
            prca->qp = prca->MyInitialQp;
        }
        else 
        {
            // compute the quantization parameter for each basic unit of P frame
            if(prca->bu_cnt!=0)
            {
              my_updateRCModel( view_idx );
              prca->qp = my_updateQP( view_idx );
            }
        }
    }

    prca->qp = my_iClip3(prca->RCMinQP, prca->RCMaxQP, prca->qp); // -rca.bitdepth_luma_qp_scale

    my_rc_update_frame_stats( view_idx ); // computer frame parameters
    

#ifdef ARCAST
	// Added by jlliu
    if( prca->curr_frame_is_dropped == 1 ) // Set max QP for scene change.
        prca->qp = 51; 

    // Drop scene change frame, since scen change will cause big quality drop. by jlliu
	drop_frame_by_scene_change(view_idx);
    // Insert I frame, by jlliu
    insert_i_frame_by_mad( view_idx );
#endif

    if(prca->BasicUnit < prca->FrameSizeInMbs) // bu-level counter
    {
        if( prca->bu_cnt == (prca->TotalNumberofBasicUnit-1) )
        {
            prca->bu_cnt=0;
            if(prca->frame_cnt >= (prca->intra_period-1)) // change "==" to ">=", lhu, 2017/03/09
            {
                prca->frame_cnt=0;
                //if(prca->gop_cnt<=1000)
                prca->gop_cnt++;
				prca->curr_frame_is_dropped = 0;
            }
    	    else if( prca->curr_frame_is_dropped == 1 || prca->insert_i_frame == 1)
    	    {
    	    	prca->frame_cnt = 0;
    	    	prca->gop_cnt++;
    	    	prca->curr_frame_is_dropped = 0;
    	    }
            else
                prca->frame_cnt++;
			
#ifdef MINI_DRONE
			if( prca->photography_enable == 1 && prca->photography_state == TAKE_PHOTO_IDLE  ) {
				prca->photography_state = TAKE_PHOTO_INSERT_I_START;
				READ_WORD_ENC(reg[view_idx][GOPFPS_ADDR], regVal);
				regVal &= (~(0xFF<<24));
				regVal |= (1<<24);
				WRITE_WORD_ENC(reg[view_idx][GOPFPS_ADDR], regVal);
				//DLOG_Error("I0\n");
			}
#endif
        }
        else
            prca->bu_cnt++;
    }
    else // frame-level counter
    {
        if(prca->frame_cnt==(prca->intra_period-1))
        {
            prca->frame_cnt=0;
            //if(prca->gop_cnt<=1000)
            prca->gop_cnt++;
            prca->curr_frame_is_dropped = 0;
        }
        else if( prca->curr_frame_is_dropped == 1 || prca->insert_i_frame == 1 ) // clear
        {
        	prca->frame_cnt = 0;
        	prca->gop_cnt++;
        	prca->curr_frame_is_dropped = 0;
        }
        else
            prca->frame_cnt++;
    }

    // Adjust Max QP when the bitrate is bigger than the UpperBound
	adaptive_increase_max_qp(view_idx);

    return prca->qp;
}


#ifdef MINI_DRONE
void __attribute__ ((section(".h264"))) take_photography( unsigned int view_idx  )
{
	RC_DATA 		*prca;
	unsigned int	 data, fps, qp;
	prca = &rca[view_idx];

	// NOTE: 
	// ToDo: Be careful the buffer level.

	//if( prca-> )
	// TODO, adjust qp adaptively.
	qp = 20 - prca->photography_frmcnt;
	qp = (qp > 20)? 20 : (qp < 10)? 10 : qp;
	READ_WORD_ENC(reg[view_idx][QP_ADDR],data);
	WRITE_WORD_ENC(reg[view_idx][QP_ADDR],((qp << 24) + (prca->slice_qp<<16)+(data&0xffff))); //write qp & sqp, also maintain ac_gop and ac_iopratio
	
	if(prca->bu_cnt == (prca->TotalNumberofBasicUnit-1))
	{
		prca->bu_cnt = 0;
		prca->photography_frmcnt ++;
					
		if( (prca->photography_frmcnt > 5)	) {
			prca->photography_enable = 0;
			prca->photography_state  = TAKE_PHOTO_INSERT_I_END; 
			prca->photography_frmcnt = 0;

			prca->frame_cnt 		 	= 0;
			// Set Gop = 1 to force intra.
			READ_WORD_ENC(reg[view_idx][GOPFPS_ADDR],data);
			data &= (~(0xFF<<24));
			data |= (1<<24);
			WRITE_WORD_ENC(reg[view_idx][GOPFPS_ADDR], data);
			//DLOG_Error("I1\n");
        }
		//DLOG_Error("TP %d\n", prca->photography_frmcnt);
    }
	else
		prca->bu_cnt++;

	// Insert User Data
	if( prca->fd_last_row == TRUE ) {

	    Reg_Write32(SRAM_SKY_MASTER_ID_ADDR, SRAM_SKY_MASTER_ID_VALUE);
	    Reg_Write32(SRAM_SKY_MASTER_ID_MASK_ADDR, SRAM_SKY_MASTER_ID_MASK_VALUE);
		// Should check level
		if( view_idx == 0)
		{
			Reg_Write32_Mask( (unsigned int) SRAM_VIEW0_ENABLE_ADDR, SRAM_VIEW0_ENABLE, SRAM_VIEW0_ENABLE);
			WRITE_WORD( (unsigned int) VIDEO_BYPASS_CHANNEL_0_DEST_ADDR, 0x00000000);
            WRITE_WORD( (unsigned int) VIDEO_BYPASS_CHANNEL_0_DEST_ADDR, 0x7c010000);
            WRITE_WORD( (unsigned int) VIDEO_BYPASS_CHANNEL_0_DEST_ADDR, (0xd5000000 | (prca->fd_iframe << 16) | (qp << 8) | (prca->photography_frmcnt << 0) ));
			Reg_Write32_Mask( (unsigned int) SRAM_VIEW0_ENABLE_ADDR, 0x00, SRAM_VIEW0_ENABLE);
		}
		#if 0
		else
		{
			Reg_Write32_Mask( (unsigned int) 0xa003004c, 0x04, 0x04);		
            WRITE_WORD( (unsigned int) 0xb1800000, 0x00000000);
            WRITE_WORD( (unsigned int) 0xb1800000, 0x7c010000);
            WRITE_WORD( (unsigned int) 0xb1800000, (0xd5c50000 | (qp << 8) | (prca->photography_frmcnt << 0) ));
			Reg_Write32_Mask( (unsigned int) 0xa003004c, 0x00, 0x04);
		}
		#endif 
	}

	if( prca->photography_state == TAKE_PHOTO_INSERT_I_START  && prca->bu_cnt > 0 )
	{
		READ_WORD_ENC(reg[view_idx][GOPFPS_ADDR],data);
		fps 	= ( (data >> 16) & 0xFF );
		data   &= ( ~(0xFF<<24) );
		data   |= ( ( fps*2 )  << 24);
		WRITE_WORD_ENC(reg[view_idx][GOPFPS_ADDR], data);
		prca->photography_state = TAKE_PHOTO_ING;
		//DLOG_Error("G0\n");
	}
}
#endif

void drop_frame_by_scene_change(unsigned int view_idx)
{
	unsigned int frame_bits_cnt;
	unsigned int i, value;
	RC_DATA *prca = &rca[view_idx];  
    unsigned int ymse, ymse_lastframe;

    long long whm255square;
    int pframe_divider;
	int last_pframe_divider;
    unsigned short last_pframe_psnr, pframe_psnr;
	
	if( prca->Iframe_causedby_scene_change	== 1 )
	{
		if( prca->bu_cnt == 0 ) //
		//if( prca->bu_cnt == prca->TotalNumberofBasicUnit - 1 ) // in order to check the total frame bits
		{
    	    prca->gop_change_NotResetRC = 1;

			prca->Iframe_causedby_scene_change = 0;
			READ_WORD_ENC(reg[view_idx][GOPFPS_ADDR],i);
    		WRITE_WORD_ENC(reg[view_idx][GOPFPS_ADDR],((prca->PrevIntraPeriod<<24)+(i&0x00ffffff)));
			READ_WORD_ENC(reg[view_idx][GOPFPS_ADDR],i);
			prca->intra_period = (i>>24)&0xff;

			READ_WORD_ENC(reg[view_idx][GOPFPS_ADDR],i);
            DLOG_Error("drop back %d %d\n", prca->PrevIntraPeriod, (i>>24) );
		}
	}

	ymse = ( (prca->ymse_frame >> 32)? 0xFFFFFFFF : (prca->ymse_frame & 0xFFFFFFFF) );
	if( prca->ymse_frame == 0)
		prca->ymse_frame = 255*255;	
			
	whm255square 		= (long long)(prca->width*255)*(long long)(prca->height*255);
	pframe_divider      = (int)(whm255square/prca->ymse_frame);		
	pframe_psnr 		= get_10log10(pframe_divider);
	prca->pframe_psnr   = pframe_psnr;

	/* Check if  */
    /* Here we only consider the typical normal case: Each scene longs a while time.*/
    /* We insert the following frame as I Frame, and want it keep it's QP not changes. */
    if( 
        (prca->drop_scene_change_frame_en )&&	          // enable this function
        (prca->type		== P_SLICE )       &&             // Only P frame think about this issue. 
        (prca->bu_cnt	>  0 )             &&			  // since the bu_cnt of first bu is 1.
         1//(prca->frame_cnt < prca->intra_period - 1)
    )
    {

    	if( !prca->curr_frame_is_dropped )
    	{
            if( (prca->frame_bits_cnt >= MAX_FRAME_BITS_CNT) )
            {
            	unsigned char bitsize_element = 
                    ( ((prca->fbits_tmp > prca->target_buff_levelx2) && (prca->bu_cnt  < (prca->TotalNumberofBasicUnit >> 1))) ||     // half picture && all bits > 2xaverage bits
                    //((prca->fbits_tmp > prca->target_buff_levelx3) && (prca->bu_cnt  < ((prca->TotalNumberofBasicUnit >> 1) + (prca->TotalNumberofBasicUnit >> 2))))  // 3/4 picture && all bits > 3xaverage bits
                    ((prca->fbits_tmp > prca->target_buff_levelx3) && (prca->bu_cnt  < ((prca->TotalNumberofBasicUnit ) )))   ||  // 3/4 picture && all bits > 3xaverage bits
                    0 ) ? 1 : 0;
				unsigned char  mad_psnr_element = 
					(((prca->frame_mad >  (prca->previous_frame_mad[2] << 3)) || (prca->pframe_psnr < prca->last_pframe_psnr - 8) ) && (prca->bu_cnt == prca->TotalNumberofBasicUnit - 1 )) ?
					1 : 0;
				
    		    if( (prca->fbits_tmp > prca->RCPSliceBits ) &&
                    ( bitsize_element  || mad_psnr_element )
                ) // the total bits have exceeded too much, i have to adjust it to be changes scene.
    		    {
                    if( (prca->frame_cnt < prca->intra_period - 1) )
                    {
    		    	    prca->gop_change_NotResetRC = 1;
    		    	
    		    	    READ_WORD_ENC(reg[view_idx][GOPFPS_ADDR],i);
    		    	    prca->PrevIntraPeriod = (i>>24)&0xff;	// save previous GOP before writing new GOP to it.
    		    	    WRITE_WORD_ENC(reg[view_idx][GOPFPS_ADDR],((1<<24)+(i&0x00ffffff)));
                    }

    		    	prca->curr_frame_is_dropped = 1;
    		        prca->frame_bits_cnt = 0;

					if( bitsize_element )
						DLOG_Error("d0, %d,%d,%d\n", prca->fbits_tmp, prca->target_buff_levelx2, prca->target_buff_levelx3);
					else if( mad_psnr_element )
						DLOG_Error("d1, %d, %d, %d, %d", prca->frame_mad, prca->previous_frame_mad[2], pframe_psnr, prca->last_pframe_psnr);
					
                    {
                        // insert drop flag, switch to cpu write
                        // close encoder channel
                        #ifdef DROP_SCENE_CHANGE
						if( view_idx == 1) {
							READ_WORD( (unsigned int) 0xa003004c, value);
							value |= 0x04;
                        	WRITE_WORD( (unsigned int) 0xa003004c, value);
                        
                        	WRITE_WORD( (unsigned int) 0xb1800000, 0x00000000);
                        	WRITE_WORD( (unsigned int) 0xb1800000, 0x7a010000);
                        	WRITE_WORD( (unsigned int) 0xb1800000, 0xd5c5b5a5);
						}
						else {
						}
                        #endif
                    }
    		    }
            }
    	}

    	// calculate the prca->target_buff_levelx2
		if( prca->bu_cnt == prca->TotalNumberofBasicUnit - 1 )
    	{
            //DLOG_INFO("xxd %d,%d,%d,%d,%d\n", prca->fbits_tmp, prca->target_buff_levelx2, prca->target_buff_levelx3, prca->RCPSliceBits, prca->frame_bits_cnt);
    		if( prca->curr_frame_is_dropped == 1 )
    		{
    			prca->Iframe_causedby_scene_change	= 1;

                // change back to encoder write from cpu write
                if( view_idx == 1 ) {
            	    DLOG_Error("VDB_OPEN:bu=%d\n", prca->bu_cnt );
					READ_WORD( (unsigned int) 0xa003004c, value);
					value &= (~0x04);
        	        WRITE_WORD( (unsigned int) 0xa003004c, value);
            	    DLOG_Error("VDB_OPEN:bu=%d\n", prca->bu_cnt );
                }
    		}
            else {
    
    		    for( frame_bits_cnt = MAX_FRAME_BITS_CNT - 1; frame_bits_cnt > 0; frame_bits_cnt -- )
    		    {
    		    	prca->frame_bits[frame_bits_cnt] = prca->frame_bits[frame_bits_cnt - 1];
    		    }
    		    prca->frame_bits[0] = (prca->fbits_tmp > prca->RCPSliceBits) ? prca->fbits_tmp : prca->RCPSliceBits;
    		    prca->frame_bits_cnt ++;
    		    prca->frame_bits_cnt = (prca->frame_bits_cnt > MAX_FRAME_BITS_CNT) ? MAX_FRAME_BITS_CNT:  prca->frame_bits_cnt;
    		    if( prca->frame_bits_cnt == MAX_FRAME_BITS_CNT )
    		    {
    		    	prca->target_buff_levelx2 = 0;
    		    	prca->target_buff_levelx3 = 0;
    		    	for( frame_bits_cnt = 0; frame_bits_cnt < MAX_FRAME_BITS_CNT; frame_bits_cnt++ )
    		    	{
    		    		prca->target_buff_levelx2 += prca->frame_bits[frame_bits_cnt];
    		    	}
    		    	prca->target_buff_levelx3 = (prca->target_buff_levelx2 >> (LOG_MAX_FRAME_BITS_CNT-1)) + (prca->target_buff_levelx2 >> (LOG_MAX_FRAME_BITS_CNT));
    		    	prca->target_buff_levelx2 >>= (LOG_MAX_FRAME_BITS_CNT-1);
                    // prca->target_buff_levelx3 = (prca->target_buff_levelx2 >> (LOG_MAX_FRAME_BITS_CNT - 1));     // x2
                    // prca->target_buff_levelx2 = (prca->target_buff_levelx3  - ( prca->target_buff_levelx2 >> (LOG_MAX_FRAME_BITS_CNT + 1)));
                    //DLOG_Info("%d\n", prca->target_buff_levelx2);
			    }

            }
            //DLOG_Info( "%d %d %d\n", prca->frame_cnt, prca->target_buff_levelx2, prca->intra_period);
            //
		} // if( prca->bu_cnt == prca->TotalNumberofBasicUnit - 1 )
	} // 
}

void insert_i_frame_by_mad( unsigned int view_idx )
{
    RC_DATA *prca = &rca[view_idx];
    unsigned int max_mad, min_mad;
    unsigned int i;
    unsigned int ymse, ymse_lastframe;
    

    // Insert I frame, if the MAD varies too much. 
    if( prca->bu_cnt == prca->TotalNumberofBasicUnit - 1 ) 
    {
        // recovery
        if( prca->insert_i_frame == 1 )
        {
            if( // 1 ||   
            	(prca->fbits_tmp >= (prca->ave_frame_bits>>1))  ||
                //(prca->fbits_tmp >= (((prca->ave_frame_bits<<1) + prca->ave_frame_bits) >> 1))  ||
                0 //(prca->actual_bitrate >= ( rc_settings[prca->is_1080p][prca->ac_br_index].InitBR << 10)) ||
                  //(ymse > mse_50db[prca->is_1080p])
            ) // 1.5 times
            {
                prca->insert_i_frame = 0;

    	        prca->gop_change_NotResetRC = 1;
			    READ_WORD_ENC(reg[view_idx][GOPFPS_ADDR],i);
    		    WRITE_WORD_ENC(reg[view_idx][GOPFPS_ADDR],((prca->PrevIntraPeriod<<24)+(i&0x00ffffff)));
				READ_WORD_ENC(reg[view_idx][GOPFPS_ADDR],i);
				prca->intra_period = (i>>24)&0xff;
                DLOG_Error("I back\n");
            }
            else {
    	        prca->gop_change_NotResetRC = 1;
                DLOG_Error("I going\n");
            }
        }

        if( prca->frame_cnt == 0 ) {
            prca->iframe_bits = prca->fbits_tmp;
        }
		//DLOG_Info("%d %d %d", prca->frame_cnt, prca->fd_iframe, prca->fbits_tmp);

        if( 
            (prca->type == P_SLICE) && 
            (prca->curr_frame_is_dropped == 0) && 
            (prca->insert_i_frame == 0) &&
            (prca->frame_cnt < prca->intra_period - 1)
          )
        {
            //DLOG_Info("%d, %d, %d, %d, %d\n", prca->fbits_tmp, prca->iframe_bits, prca->actual_bitrate, (((prca->iframe_bits << 8) - (prca->iframe_bits << 5) - (prca->iframe_bits << 4) -(prca->iframe_bits <<1) ) >> 8),  ( rc_settings[prca->is_1080p][prca->ac_br_index].UpperBR << 10) );
            if( (prca->frame_mad >  (prca->previous_frame_mad[2] << 2)) ||
				
                //( (prca->fbits_tmp > (prca->iframe_bits << 1)) && (prca->actual_bitrate >= ( rc_settings[prca->is_1080p][prca->ac_br_index].InitBR << 10)) )       ||
                //( (prca->fbits_tmp > (( (prca->iframe_bits << 1) + (prca->iframe_bits << 0)) >> 2) ) &&  (prca->actual_bitrate < ( rc_settings[prca->is_1080p][prca->ac_br_index].UpperBR << 10))) ||
                //  ( ((prca->fbits_tmp > ( ((prca->iframe_bits<<1) + prca->iframe_bits) >> 1) ) && (prca->RCISliceBitRatio > 2) && (prca->iframe_bits > ((prca->RCISliceBits*22)>>5)) ) ) ||
                //  ( ((prca->fbits_tmp > ( ((prca->iframe_bits<<2))) && (prca->RCISliceBitRatio <= 2)) && (prca->iframe_bits > ((prca->RCISliceBits*22)>>5)) )) ||              
                
                //	( ((prca->fbits_tmp > ( ((prca->iframe_bits<<1) + prca->iframe_bits) >> 1) ) && (prca->RCISliceBitRatio > 2) && (prca->iframe_bits > ((prca->RCISliceBits*22)>>5)) ) ) ||
                //	( ((prca->fbits_tmp > ( ((prca->iframe_bits<<2))) && (prca->RCISliceBitRatio <= 2)) && (prca->iframe_bits > ((prca->RCISliceBits*22)>>5)) )) ||
                (prca->pframe_psnr < prca->last_pframe_psnr - 5) ||
                
                //( (prca->fbits_tmp > (( (prca->iframe_bits << 8) - (prca->iframe_bits << 5) - (prca->iframe_bits << 4) -(prca->iframe_bits <<1) ) >> 8) ) &&  (prca->actual_bitrate < ( rc_settings[prca->is_1080p][prca->ac_br_index].UpperBR << 10))) ||
                //( (prca->fbits_tmp > (( (prca->iframe_bits << 8) - (prca->iframe_bits << 5) - (prca->iframe_bits << 4) -(prca->iframe_bits <<1) ) >> 8) )) ||
                //( (prca->fbits_tmp > (prca->RCPSliceBits << 3)) )       ||     // SCENE: miao_biao_shan_shuo
                //( (prca->fbits_tmp > (prca->RCISliceBits     )) )       ||
                //( (ymse > mse_30db[prca->is_1080p]) )                   ||
                0//( (ymse_lastframe <= 0x7FFFFFFF && (ymse >= (( (ymse_lastframe << 4) + ( ymse_lastframe << 2 ) + 7) >> 3) ) ) )                            // 4 db

            )
            {
    	        prca->gop_change_NotResetRC = 1;
                prca->insert_i_frame = 1;
    	        READ_WORD_ENC(reg[view_idx][GOPFPS_ADDR],i);
    	        prca->PrevIntraPeriod = (i>>24)&0xff;	// save previous GOP before writing new GOP to it.
    	        WRITE_WORD_ENC(reg[view_idx][GOPFPS_ADDR],((1<<24)+(i&0x00ffffff)));
    
                prca->frame_bits_cnt = 0;       // if insert I frame, i want the target_buff_level to be recalculated.
                //DLOG_Info("Insert I: %d %d\n", ymse, ymse_lastframe);
                //DLOG_Error("Insert I: %d %d %d %d\n", prca->fbits_tmp, prca->iframe_bits, prca->frame_mad, prca->previous_frame_mad[2]);
                DLOG_Error("Insert I: %d %d %d %d\n", prca->last_pframe_psnr, prca->pframe_psnr, prca->frame_mad, prca->previous_frame_mad[2]);
            }

        }

        if( prca->type == P_SLICE && prca->insert_i_frame == 0)
        {   //  insert I frame basing on MAD
            if( prca->previous_frame_mad[1] > prca->frame_mad  )
            {
                max_mad = prca->previous_frame_mad[1];
                min_mad = prca->frame_mad;
            }
            else {
                max_mad = prca->frame_mad;
                min_mad = prca->previous_frame_mad[1];
            }
            if( prca->previous_frame_mad[0] > max_mad )
            {
                prca->previous_frame_mad[2] = max_mad;
            }
            else
            {
                if( prca->previous_frame_mad[0] < min_mad )
                {
                    prca->previous_frame_mad[2] = min_mad;
                }
                else
                {
                    prca->previous_frame_mad[2] = prca->previous_frame_mad[0];
                }
            }
            prca->previous_frame_mad[0] = prca->previous_frame_mad[1];
            prca->previous_frame_mad[1] = prca->frame_mad;


			//whm255square = (long long)(prca->width*255)*(long long)(prca->height*255);
            //ymse = ( (prca->ymse_frame >> 32)? 0xFFFFFFFF : (prca->ymse_frame & 0xFFFFFFFF) );
		    //ymse_lastframe = prca->ymse_lastframe;
			
        	//if( ymse_lastframe == 0)
			//	ymse_lastframe = 255*255;
			//last_pframe_divider = (int)(whm255square/prca->ymse_lastframe);
			//last_pframe_psnr 	= get_10log10(last_pframe_divider);
			prca->last_pframe_psnr 	= prca->pframe_psnr;
        }
        else
        {
            prca->previous_frame_mad[2] = (1<<24);  // initialize 
            prca->previous_frame_mad[1] = (1<<24);  // initialize 
            prca->previous_frame_mad[0] = (1<<10);
			prca->last_pframe_psnr = 10;
        }

    }   // bu_cnt == TotalBuNumber-1
}

#define MAX_FRAME_RATE 120
static unsigned int bits[MAX_FRAME_RATE];    // Assume the max frame rate is 120 fps
void adaptive_increase_max_qp( unsigned int view_idx )
{
    RC_DATA *prca;
    unsigned int fps;
    int total_bits;
    unsigned int ymse;
    unsigned int upper_bitrate;
    prca = &rca[view_idx];
    fps = prca->framerate;
    if( fps > MAX_FRAME_RATE )
    {
        // More than 120 fps, then close this function
        return;
    }

    ymse = ( (prca->ymse_frame >> 32) > 0) ? 0xFFFFFFFF : (prca->ymse_frame & 0xFFFFFFFF);
    
    if( prca->fd_last_row && prca->bu_cnt == 1)  // since bu counter of last bu is 1.
    {
        //DLOG_INFO("%d %d %d\n", prca->fbits_tmp, bits[prca->passed_frame_index], prca->actual_bitrate);
    
        if( prca->passed_frame_cnt < fps )
        {
            prca->actual_bitrate += prca->fbits_tmp;
            bits[prca->passed_frame_cnt] = prca->fbits_tmp;
            prca->passed_frame_cnt ++;
        }
        else {
    
            prca->actual_bitrate = prca->actual_bitrate + prca->fbits_tmp - bits[prca->passed_frame_index];
            //DLOG_INFO("%d, %d\n", prca->passed_frame_index, prca->actual_bitrate );
            upper_bitrate        = (rc_setting[prca->ac_br_index].MaxBR << 10);
            //if( prca->actual_bitrate > ( rc_settings[prca->is_1080p][prca->ac_br_index].UpperBR<<10 ) || ymse < mse_30db[prca->is_1080p] )
            //if( prca->actual_bitrate > ( rc_settings[prca->is_1080p][prca->ac_br_index].UpperBR<<10 ) )
            if( prca->actual_bitrate > (upper_bitrate) )
            {
                if( prca->RCMaxQP < rc_setting[prca->ac_br_index].UpperQP )
                {
					signed int qpdelta = ((prca->actual_bitrate - upper_bitrate + (1<<18))>>19) + 1;
					qpdelta = ( qpdelta <= 0 )? 0 : (qpdelta > 10 ) ? 10 : qpdelta;
                    prca->RCMaxQP += qpdelta;
                    DLOG_Error("I%d %d %d\n", prca->RCMaxQP, prca->actual_bitrate,rc_setting[prca->ac_br_index].MaxBR );
                }
            }
            else  
            {
                if( (prca->RCMaxQP > rc_setting[prca->ac_br_index].MaxQP) )
                {
                    if( prca->iframe_bits > prca->RCISliceBits && (prca->RCMaxQP <= prca->MyInitialQp + 1) )    // DJI: If maxQP is too small, then the I frame is too big.
                        prca->RCMaxQP += 2;
                    else
                        prca->RCMaxQP -= 2;
                        
                    DLOG_Error("D%d %d %d\n", prca->RCMaxQP, prca->actual_bitrate,rc_setting[prca->ac_br_index].UpperQP );
                }
            } 
    
            bits[prca->passed_frame_index] = prca->fbits_tmp;
        }
        prca->passed_frame_index ++;
        if( prca->passed_frame_index >= fps )
            prca->passed_frame_index = 0;


        if(prca->RCMaxQP > rc_setting[prca->ac_br_index].UpperQP) 
            prca->RCMaxQP = rc_setting[prca->ac_br_index].UpperQP;
		if(prca->RCMaxQP < rc_setting[prca->ac_br_index].MaxQP)
			prca->RCMaxQP = rc_setting[prca->ac_br_index].MaxQP;
    }
}

