#ifndef __TRANCEIVER_H__
#define __TRANCEIVER_H__

#ifdef __cplusplus
extern "C"
{
#endif
void app_net( int txrx );
void app_set_ip_filter(unsigned char *data,unsigned char index);
void app_log_ctrl(unsigned char en,unsigned char port);

#ifdef __cplusplus
}
#endif 

#endif

