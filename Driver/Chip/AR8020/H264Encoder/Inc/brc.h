#ifndef _BRC_H_
#define _BRC_H_

//#define ARCAST

#define MAX_FRAME_BITS_CNT 8
#define LOG_MAX_FRAME_BITS_CNT 3
#define DROP_SCENE_CHANGE

#define RC_MODE_0 0
#define RC_MODE_1 1
#define RC_MODE_2 2
#define RC_MODE_3 3
#define I_SLICE 2
#define P_SLICE 0
#define RC_MODEL_HISTORY 21
#define MIN_INT64 0x8000000000000000
#define MIN_INT   0x80000000
#define MAX_INT   0x7fffffff
typedef long long int64;
typedef unsigned char Boolean;
typedef unsigned char uint8;
typedef unsigned int  uint;


#define MIN_INT64 0x8000000000000000
#define MIN_INT   0x80000000
#define MAX_INT   0x7fffffff

enum {
	TAKE_PHOTO_IDLE = 0,
	TAKE_PHOTO_INSERT_I_START,
	TAKE_PHOTO_ING,
	TAKE_PHOTO_INSERT_I_END,
};
//// generic rate control variables
typedef struct my_rc{
//// top set(img,params)
  /**********************************************************************/
  /* Take photo	parameters											    */
  /**********************************************************************/
  uint8		        photography_enable;	    // enable signal
  uint8 	        photography_state;	    // 4 states, TAKE_PHOTO_IDLE, TAKE_PHOTO_INSERT_I_START, TAKE_PHOTO_ING, TAKE_PHOTO_INSERT_I_END
  uint8 	        photography_frmcnt;

  /**********************************************************************/
  /* real time parameters											    */
  /**********************************************************************/
  unsigned char     realtime_mode;			// enable signal

  /**********************************************************************/
  /* rate control parameters										    */
  /**********************************************************************/
  unsigned char     enable;					// enable signal, channel open/close
  unsigned char     rc_enable;				// enable signal, rate control open/close
  unsigned char     RCUpdateMode;
  unsigned char     framerate;
  int               height;                  // height
  int               width;                   // width
  int               bit_rate;                // bitrate
  unsigned char     ac_br_index;
  int               size;                    // width * height
  unsigned char     PMaxQpChange;            // Not Used, now
  unsigned char     intra_period;			 // GOP
  unsigned char     RCISliceBitRatio;		 // IPRatio
  unsigned char     RCISliceBitRatioMax;	 // IPRatio Max
  unsigned char     RCMinQP;                 // 
  unsigned char     RCMaxQP;             
  int               FrameSizeInMbs;          // 
  int               FrameHeightInMbs;        // 
  int               BasicUnit;               //
  int               MBPerRow;                //
  
  /**********************************************************************/
  /* rate control temp variables									    */
  /**********************************************************************/
  int               gop_cnt;
  int               frame_cnt;
  int               bu_cnt;
  int               slice_qp;
  int               NumberofHeaderBits;
  int               NumberofTextureBits;
  int               NumberofBasicUnitHeaderBits;
  int               NumberofBasicUnitTextureBits;
  int               NumberofGOP;
  int               codedbu_cnt;
  unsigned char     c1_over;
  int               type;
  int               qp;
  int               PrevBitRate;
  int               TotalFrameMAD;
  int               RCISliceTargetBits;
  unsigned char     cmadequ0;
  unsigned char     prev_ac_br_index;
  unsigned char     PrevIntraPeriod;
  int               PrevFbits;               // Previous total bits, used to calculate the total bits of current bu.
  int               frame_mad;				 // Total frame MAD
  int               frame_tbits;			 // Total frame texture bits
  int               frame_hbits;			 // Total frame header bits
  int               frame_abits;			 // Total frame all bits
  int               TotalMADBasicUnit; 
  int               CurrentBufferFullness;
  int               RemainingBits;
  int               RCPSliceBits;
  int               RCISliceBits;
  int               NPSlice;
  int               NISlice;
  int               GAMMAP_1p;
  int               BETAP_1p;
  int               TargetBufferLevel;
  int               MyInitialQp;
  int               PAverageQp;
  int				LastIQp;
  int				TotalIQp;
  int				TargetForBasicUnit;
  int               PreviousPictureMAD_8p;       //
  int               MADPictureC1_12p;            //
  int               MADPictureC2_12p;            //
  int               PMADPictureC1_12p;           //
  int               PMADPictureC2_12p;           //
  int               PPictureMAD_8p;              //
  int               PictureMAD_8p  [20];         //
  int               ReferenceMAD_8p[20];         //
  int               mad_tmp0 [20];               //
  int               mad_tmp0_valid [20];         //
  int               mad_tmp1 [20];               //
  int               mad_tmp2 [20];               //
  int               m_rgQp_8p [20];              //
  int               m_rgRp_8p [20];              //
  int               m_rgRp_8prr8 [20];           //
  int               rc_tmp0 [20];                //
  int               rc_tmp1 [20];                //
  int               rc_tmp2 [20];                //
  int               rc_tmp3 [20];                //
  int               rc_tmp4 [20];                //
  unsigned char     rc_hold;                     //
  unsigned char     mad_hold;                    //
  char              rc_rgRejected [20];          //
  char              mad_rgRejected [20];         //
  int               m_X1_8p;                     //
  int               m_X2_8p;                     //
  int               Pm_X1_8p;                    //
  int               Pm_X2_8p;                    //
  int               Pm_Qp;                       //
  int               Pm_Hp;                       //
  int               MADm_windowSize;             //
  int               m_windowSize;                //
  int               m_Qc;                        //
  int               TotalFrameQP;                //
  int               PAveHeaderBits1;             //
  int               PAveHeaderBits2;             //
  int               PAveHeaderBits3;             //
  int               PAveFrameQP;                 //
  int               TotalNumberofBasicUnit;      //
  int               NumberofCodedPFrame;         //
  int               TotalQpforPPicture;          //
  int               CurrentMAD_8p;               //
  int               TotalBUMAD_12p;              //
  int               PreviousMAD_8p;              //
  int               DDquant;                     //
  int               QPLastPFrame;                //
  int               QPLastGOP;                   //
  int               BUPFMAD_8p[70];              //
  int               BUCFMAD_8p[70];              //
  int               RCISliceActualBits;          //
  int               GOPOverdue;                  //
  // rate control variables
  int               Xp;                          //
  int               Xb;                          //
  int               Target;                      //
  int               Np;                          //
  int               UpperBound1;                 //
  int               UpperBound2;                 //
  int               LowerBound;                  //
  int               DeltaP;                      //
  int               TotalPFrame;                 //
  //// for feedback
  int               aof_inc_qp;                   //
  unsigned char     fd_row_cnt;                   //
  unsigned char	    fd_last_row;				   
  unsigned char	    fd_last_p;
  unsigned char	    fd_iframe;
  unsigned char	    fd_reset; 				     // 
  unsigned char	    fd_irq_en;				   
  
  int               tbits_tmp;                    //
  int               hbits_tmp;                    //
  int               fbits_tmp;                    //
  int               mad_tmp;                      //
  int               ymsel_tmp;                    //
  int               ymseh_tmp;                    //
  int               poweron_rc_params_set;        //



  unsigned char     re_bitrate;              // Not Used, now
  int               no_frm_base;             // Not Used, now
  unsigned char     RCIoverPRatio;			 // Not Used, now
  int               header_bits;             // Not Used, now
  int               new_bitrate;             // Not Used, now
  unsigned char     wireless_screen;		 // Not Used, now
  unsigned char     changeToIFrame;
  unsigned char     insertOneIFrame;
  unsigned char     PrevFrmPSNRLow;
  unsigned char     HBitsRatioABits_level;
  unsigned char     nextPFgotoIF;
  unsigned char     IFduration;				
  unsigned char     PrevRCMinQP;
  unsigned char     PSNRDropSharply;
  
  /**********************************************************************/
  /* Special Params for Quality Optimization						    */
  /**********************************************************************/
  unsigned char     gop_change_NotResetRC;
  long long         ifrm_ymse;
  long long 		min_pfrm_ymse;
  long long 		ymse_frame;
  

  unsigned int      dvp_lb_freesize;

#if 0
  unsigned int close_bs_channel;
  unsigned int insertIframe_since_buffer;
#endif

  unsigned int previous_frame_mse[8];
  unsigned int previous_frame_mse_idx;
  unsigned int previous_total_mse;

  unsigned int last_row_cnt;
  unsigned int last_bu_cnt;
  unsigned int row_cnt_in_bu;

  uint8        Iframe_causedby_scene_change;
  uint8        drop_scene_change_frame_en;
  uint8        curr_frame_is_dropped;
  uint         frame_bits_cnt;
  uint         target_buff_levelx2;
  uint         target_buff_levelx3;
  uint         frame_bits[MAX_FRAME_BITS_CNT];
  int          ymse_lastframe;
  int          ymse_pframe[8];
  int          last_pframe_psnr;
  int          pframe_psnr;

  uint8        insert_i_frame;
  uint         ave_frame_bits;
  uint         previous_frame_mad[3];
  uint         iframe_bits;

  // adaptive Max QP
  unsigned int actual_bitrate;
  unsigned int passed_frame_cnt;
  unsigned int passed_frame_index;

  unsigned short ret_width;
  unsigned short ret_hight;
  int			 enc_frame_cnt;

} RC_DATA;

enum {
	NORMAL = 0,
	OPEN   = 1,
	CLOSE  = 2
};

typedef struct _rc_setting {
	unsigned int MinQP;
	unsigned int MaxQP;
	unsigned int UpperQP;
	unsigned int MaxBR;
} RC_SETTING;


int aof_v0_frame;
int aof_v1_frame;

int  my_imin(int a, int b);
int  my_iequmin(int a, int b);
int  my_imax(int a, int b);
int  my_iClip3(int low, int high, int x);
int  my_sqrt(int x);
int  my_sqrt64(int64 x);
int  Qstep2QP_8p(int Qstep_8p);
int  QP2Qstep_8p(int QP);
void my_rc_ac_br(int view); // @lhu
//void my_calc_prevFramePSNR(unsigned char psnr_level, int view);
unsigned short my_divider2psnr(int my_divider);
void my_ac_RCISliceBitRatio(unsigned char RCISliceBitRatioMax, int view);
void my_rc_params( unsigned int view_idx );
void my_rc_init_seq( unsigned int view_idx );
void my_rc_init_GOP( unsigned int view_idx, int np);
void my_rc_init_pict( unsigned int view_idx, int mult);
void my_rc_update_pict(unsigned int view_idx, int nbits);
void my_updatePparams( unsigned int view_idx );
void my_rc_update_pict_frame( unsigned int view_idx );
void my_updateRCModel( unsigned int view_idx );
void my_RCModelEstimator(unsigned int view_idx, int n_realSize, int n_windowSize, char *rc_rgRejected);
void my_updateMADModel( unsigned int view_idx );
void my_MADModelEstimator(unsigned int view_idx, int n_realSize, int n_windowSize, char *mad_rgRejected);
void my_updateQPNonPicAFF( unsigned int view_idx );
int  my_updateFirstP( unsigned int view_idx );
int  my_updateNegativeTarget(unsigned int view_idx, int m_Qp);
int  my_updateFirstBU( unsigned int view_idx );
void my_updateLastBU( unsigned int view_idx );
void my_predictCurrPicMAD( unsigned int view_idx );
void my_updateModelQPBU( unsigned int view_idx, int m_Qp);
void my_updateModelQPFrame( unsigned int view_idx, int m_Bits);
int  my_rc_handle_mb( unsigned int view_idx );
void my_rc_update_bu_stats( unsigned int view_idx );
void my_rc_update_frame_stats( unsigned int view_idx );
void my_rc_init_gop_params( unsigned int view_idx );
void my_rc_init_frame( unsigned int view_idx );
void my_rc_store_header_bits( unsigned int view_idx );
//int  my_updateQPRC1( );
int  my_updateQPRC3( unsigned int view_idx );
//int  my_updateQPRC0( );
void my_initial_all( unsigned int view_idx );
void my_feedback(unsigned int view_idx, unsigned int feedback);
void my_hold( unsigned int view_idx );
int  (*my_updateQP)( unsigned int view_idx );

void update_aof_cycle();
void update_aof();
void take_photography( unsigned int view_idx  );
void drop_frame_by_scene_change(unsigned int view_idx);
void insert_i_frame_by_mad(unsigned int view_idx);
void adaptive_increase_max_qp( unsigned int view_idx );



int VEBRC_IRQ_Handler(unsigned int view0_feedback, unsigned int view1_feedback);


#endif
