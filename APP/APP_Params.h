#ifndef  __APP_PARAMS_H
#define  __APP_PARAMS_H


#define ADC_VREF         3.3f
#define SAMPLING_RES     0.0005f
#define ADC_DATARANGE    4096
#define OPAMP_GAIN       20 


#define CURRENT_CONV_COEFF   (0.08058608f)  //(3.3/(20*0.0005*4096))
// 0.08/30(电流基准)*65536 = 174.7(中间变量)   174.7*65536 = 11537153
#define ADC_TO_PU_GAIN_VAL   11537153   //Q16

#endif