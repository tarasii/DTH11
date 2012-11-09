#line 1 "..\\STM32L1xx_StdPeriph_Driver\\src\\stm32l1xx_adc.c"









































































  

 
#line 1 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_adc.h"




















  

 







 
#line 1 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"




































 



 



 
    






  


 
  


 


       










 





#line 88 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"







   





 





 


#line 117 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 
#line 129 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 



 




 




 
 
typedef enum IRQn
{
 
  NonMaskableInt_IRQn         = -14,     
  MemoryManagement_IRQn       = -12,     
  BusFault_IRQn               = -11,     
  UsageFault_IRQn             = -10,     
  SVCall_IRQn                 = -5,      
  DebugMonitor_IRQn           = -4,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      

 
  WWDG_IRQn                   = 0,       
  PVD_IRQn                    = 1,       
  TAMPER_STAMP_IRQn           = 2,       
  RTC_WKUP_IRQn               = 3,       
  FLASH_IRQn                  = 4,       
  RCC_IRQn                    = 5,       
  EXTI0_IRQn                  = 6,       
  EXTI1_IRQn                  = 7,       
  EXTI2_IRQn                  = 8,       
  EXTI3_IRQn                  = 9,       
  EXTI4_IRQn                  = 10,      
  DMA1_Channel1_IRQn          = 11,      
  DMA1_Channel2_IRQn          = 12,      
  DMA1_Channel3_IRQn          = 13,      
  DMA1_Channel4_IRQn          = 14,      
  DMA1_Channel5_IRQn          = 15,      
  DMA1_Channel6_IRQn          = 16,      
  DMA1_Channel7_IRQn          = 17,      
  ADC1_IRQn                   = 18,      
  USB_HP_IRQn                 = 19,      
  USB_LP_IRQn                 = 20,      
  DAC_IRQn                    = 21,      
  COMP_IRQn                   = 22,      
  EXTI9_5_IRQn                = 23,      
  LCD_IRQn                    = 24,      
  TIM9_IRQn                   = 25,      
  TIM10_IRQn                  = 26,      
  TIM11_IRQn                  = 27,      
  TIM2_IRQn                   = 28,      
  TIM3_IRQn                   = 29,      
  TIM4_IRQn                   = 30,      
  I2C1_EV_IRQn                = 31,      
  I2C1_ER_IRQn                = 32,      
  I2C2_EV_IRQn                = 33,      
  I2C2_ER_IRQn                = 34,      
  SPI1_IRQn                   = 35,      
  SPI2_IRQn                   = 36,      
  USART1_IRQn                 = 37,      
  USART2_IRQn                 = 38,      
  USART3_IRQn                 = 39,      
  EXTI15_10_IRQn              = 40,      
  RTC_Alarm_IRQn              = 41,      
  USB_FS_WKUP_IRQn            = 42,      
  TIM6_IRQn                   = 43,      





  TIM7_IRQn                   = 44,      
  SDIO_IRQn                   = 45,      
  TIM5_IRQn                   = 46,      
  SPI3_IRQn                   = 47,      
  UART4_IRQn                  = 48,      
  UART5_IRQn                  = 49,      
  DMA2_Channel1_IRQn          = 50,      
  DMA2_Channel2_IRQn          = 51,      
  DMA2_Channel3_IRQn          = 52,      
  DMA2_Channel4_IRQn          = 53,      
  DMA2_Channel5_IRQn          = 54,      
  AES_IRQn                    = 55,      
  TS_IRQn                     = 56       

} IRQn_Type;



 

#line 1 "C:\\PRG\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"
 




















 






















 




 


 

 













#line 89 "C:\\PRG\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"


 







#line 119 "C:\\PRG\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"

#line 1 "C:\\PRG\\Keil\\ARM\\RV31\\Inc\\stdint.h"
 
 





 









#line 25 "C:\\PRG\\Keil\\ARM\\RV31\\Inc\\stdint.h"







 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     
typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;

     
typedef   signed       __int64 intmax_t;
typedef unsigned       __int64 uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     


     


     


     

     


     


     


     

     



     



     


     
    
 



#line 196 "C:\\PRG\\Keil\\ARM\\RV31\\Inc\\stdint.h"

     







     










     











#line 260 "C:\\PRG\\Keil\\ARM\\RV31\\Inc\\stdint.h"



 


#line 121 "C:\\PRG\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"
#line 1 "C:\\PRG\\Keil\\ARM\\CMSIS\\Include\\core_cmInstr.h"
 




















 





 



 


 









 







 







 






 








 







 







 









 









 
__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}








 
__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}









 











 









 









 









 











 











 











 







 










 










 









 






#line 615 "C:\\PRG\\Keil\\ARM\\CMSIS\\Include\\core_cmInstr.h"

   

#line 122 "C:\\PRG\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"
#line 1 "C:\\PRG\\Keil\\ARM\\CMSIS\\Include\\core_cmFunc.h"
 




















 





 



 


 





 
 






 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}







 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}







 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}







 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}







 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}







 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}







 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}







 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}







 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}







 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}







 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}








 







 








 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}







 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xff);
}







 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}







 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & (uint32_t)1);
}




#line 293 "C:\\PRG\\Keil\\ARM\\CMSIS\\Include\\core_cmFunc.h"


#line 612 "C:\\PRG\\Keil\\ARM\\CMSIS\\Include\\core_cmFunc.h"

 


#line 123 "C:\\PRG\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"








 
#line 153 "C:\\PRG\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"

 






 
#line 169 "C:\\PRG\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"

 












 


 





 


 
typedef union
{
  struct
  {

    uint32_t _reserved0:27;               





    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       

    uint32_t _reserved0:15;               





    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 


 
typedef struct
{
  volatile uint32_t ISER[8];                  
       uint32_t RESERVED0[24];
  volatile uint32_t ICER[8];                  
       uint32_t RSERVED1[24];
  volatile uint32_t ISPR[8];                  
       uint32_t RESERVED2[24];
  volatile uint32_t ICPR[8];                  
       uint32_t RESERVED3[24];
  volatile uint32_t IABR[8];                  
       uint32_t RESERVED4[56];
  volatile uint8_t  IP[240];                  
       uint32_t RESERVED5[644];
  volatile  uint32_t STIR;                     
}  NVIC_Type;

 



 






 


 
typedef struct
{
  volatile const  uint32_t CPUID;                    
  volatile uint32_t ICSR;                     
  volatile uint32_t VTOR;                     
  volatile uint32_t AIRCR;                    
  volatile uint32_t SCR;                      
  volatile uint32_t CCR;                      
  volatile uint8_t  SHP[12];                  
  volatile uint32_t SHCSR;                    
  volatile uint32_t CFSR;                     
  volatile uint32_t HFSR;                     
  volatile uint32_t DFSR;                     
  volatile uint32_t MMFAR;                    
  volatile uint32_t BFAR;                     
  volatile uint32_t AFSR;                     
  volatile const  uint32_t PFR[2];                   
  volatile const  uint32_t DFR;                      
  volatile const  uint32_t ADR;                      
  volatile const  uint32_t MMFR[4];                  
  volatile const  uint32_t ISAR[5];                  
       uint32_t RESERVED0[5];
  volatile uint32_t CPACR;                    
} SCB_Type;

 















 






























 




#line 396 "C:\\PRG\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"

 





















 









 


















 










































 









 









 















 






 


 
typedef struct
{
       uint32_t RESERVED0[1];
  volatile const  uint32_t ICTR;                     

  volatile uint32_t ACTLR;                    



} SCnSCB_Type;

 



 










 






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t LOAD;                     
  volatile uint32_t VAL;                      
  volatile const  uint32_t CALIB;                    
} SysTick_Type;

 












 



 



 









 






 


 
typedef struct
{
  volatile  union
  {
    volatile  uint8_t    u8;                   
    volatile  uint16_t   u16;                  
    volatile  uint32_t   u32;                  
  }  PORT [32];                           
       uint32_t RESERVED0[864];
  volatile uint32_t TER;                      
       uint32_t RESERVED1[15];
  volatile uint32_t TPR;                      
       uint32_t RESERVED2[15];
  volatile uint32_t TCR;                      
       uint32_t RESERVED3[29];                                  
  volatile  uint32_t IWR;                      
  volatile const  uint32_t IRR;                      
  volatile uint32_t IMCR;                     
       uint32_t RESERVED4[43];                                  
  volatile  uint32_t LAR;                      
  volatile const  uint32_t LSR;                      
       uint32_t RESERVED5[6];                                   
  volatile const  uint32_t PID4;                     
  volatile const  uint32_t PID5;                     
  volatile const  uint32_t PID6;                     
  volatile const  uint32_t PID7;                     
  volatile const  uint32_t PID0;                     
  volatile const  uint32_t PID1;                     
  volatile const  uint32_t PID2;                     
  volatile const  uint32_t PID3;                     
  volatile const  uint32_t CID0;                     
  volatile const  uint32_t CID1;                     
  volatile const  uint32_t CID2;                     
  volatile const  uint32_t CID3;                     
} ITM_Type;

 



 



























 



 



 



 









   






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t CYCCNT;                   
  volatile uint32_t CPICNT;                   
  volatile uint32_t EXCCNT;                   
  volatile uint32_t SLEEPCNT;                 
  volatile uint32_t LSUCNT;                   
  volatile uint32_t FOLDCNT;                  
  volatile const  uint32_t PCSR;                     
  volatile uint32_t COMP0;                    
  volatile uint32_t MASK0;                    
  volatile uint32_t FUNCTION0;                
       uint32_t RESERVED0[1];
  volatile uint32_t COMP1;                    
  volatile uint32_t MASK1;                    
  volatile uint32_t FUNCTION1;                
       uint32_t RESERVED1[1];
  volatile uint32_t COMP2;                    
  volatile uint32_t MASK2;                    
  volatile uint32_t FUNCTION2;                
       uint32_t RESERVED2[1];
  volatile uint32_t COMP3;                    
  volatile uint32_t MASK3;                    
  volatile uint32_t FUNCTION3;                
} DWT_Type;

 






















































 



 



 



 



 



 



 



























   






 


 
typedef struct
{
  volatile uint32_t SSPSR;                    
  volatile uint32_t CSPSR;                    
       uint32_t RESERVED0[2];
  volatile uint32_t ACPR;                     
       uint32_t RESERVED1[55];
  volatile uint32_t SPPR;                     
       uint32_t RESERVED2[131];
  volatile const  uint32_t FFSR;                     
  volatile uint32_t FFCR;                     
  volatile const  uint32_t FSCR;                     
       uint32_t RESERVED3[759];
  volatile const  uint32_t TRIGGER;                  
  volatile const  uint32_t FIFO0;                    
  volatile const  uint32_t ITATBCTR2;                
       uint32_t RESERVED4[1];
  volatile const  uint32_t ITATBCTR0;                
  volatile const  uint32_t FIFO1;                    
  volatile uint32_t ITCTRL;                   
       uint32_t RESERVED5[39];
  volatile uint32_t CLAIMSET;                 
  volatile uint32_t CLAIMCLR;                 
       uint32_t RESERVED7[8];
  volatile const  uint32_t DEVID;                    
  volatile const  uint32_t DEVTYPE;                  
} TPI_Type;

 



 



 












 






 



 





















 



 





















 



 



 


















 






   







 


 
typedef struct
{
  volatile const  uint32_t TYPE;                     
  volatile uint32_t CTRL;                     
  volatile uint32_t RNR;                      
  volatile uint32_t RBAR;                     
  volatile uint32_t RASR;                     
  volatile uint32_t RBAR_A1;                  
  volatile uint32_t RASR_A1;                  
  volatile uint32_t RBAR_A2;                  
  volatile uint32_t RASR_A2;                  
  volatile uint32_t RBAR_A3;                  
  volatile uint32_t RASR_A3;                  
} MPU_Type;

 









 









 



 









 






























 







 


 
typedef struct
{
  volatile uint32_t DHCSR;                    
  volatile  uint32_t DCRSR;                    
  volatile uint32_t DCRDR;                    
  volatile uint32_t DEMCR;                    
} CoreDebug_Type;

 




































 






 







































 






 

 
#line 1227 "C:\\PRG\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"

#line 1236 "C:\\PRG\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"






 










 

 



 




 










 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07);                

  reg_value  =  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;                                                    
  reg_value &= ~((0xFFFFUL << 16) | (7UL << 8));              
  reg_value  =  (reg_value                                 |
                ((uint32_t)0x5FA << 16) |
                (PriorityGroupTmp << 8));                                      
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR =  reg_value;
}







 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8)) >> 8);    
}







 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}







 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}











 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}







 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}







 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}










 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}










 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[((uint32_t)(IRQn) & 0xF)-4] = ((priority << (8 - 4)) & 0xff); }  
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[(uint32_t)(IRQn)] = ((priority << (8 - 4)) & 0xff);    }         
}












 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[((uint32_t)(IRQn) & 0xF)-4] >> (8 - 4)));  }  
  else {
    return((uint32_t)(((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[(uint32_t)(IRQn)]           >> (8 - 4)));  }  
}













 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 4) ? 4 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 4) < 7) ? 0 : PriorityGroupTmp - 7 + 4;

  return (
           ((PreemptPriority & ((1 << (PreemptPriorityBits)) - 1)) << SubPriorityBits) |
           ((SubPriority     & ((1 << (SubPriorityBits    )) - 1)))
         );
}













 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 4) ? 4 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 4) < 7) ? 0 : PriorityGroupTmp - 7 + 4;

  *pPreemptPriority = (Priority >> SubPriorityBits) & ((1 << (PreemptPriorityBits)) - 1);
  *pSubPriority     = (Priority                   ) & ((1 << (SubPriorityBits    )) - 1);
}





 
static __inline void NVIC_SystemReset(void)
{
  __dsb(0xF);                                                     
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = ((0x5FA << 16)      |
                 (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8)) |
                 (1UL << 2));                    
  __dsb(0xF);                                                      
  while(1);                                                     
}

 



 




 

















 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if (ticks > (0xFFFFFFUL << 0))  return (1);             

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (ticks & (0xFFFFFFUL << 0)) - 1;       
  NVIC_SetPriority (SysTick_IRQn, (1<<4) - 1);   
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2) |
                   (1UL << 1)   |
                   (1UL << 0);                     
  return (0);                                                   
}



 



 




 

extern volatile int32_t ITM_RxBuffer;                     












 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if ((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL << 0))                  &&       
      (((ITM_Type *) (0xE0000000UL) )->TER & (1UL << 0)        )                    )      
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0].u32 == 0);
    ((ITM_Type *) (0xE0000000UL) )->PORT[0].u8 = (uint8_t) ch;
  }
  return (ch);
}








 
static __inline int32_t ITM_ReceiveChar (void) {
  int32_t ch = -1;                            

  if (ITM_RxBuffer != 0x5AA55AA5) {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5;        
  }

  return (ch);
}








 
static __inline int32_t ITM_CheckChar (void) {

  if (ITM_RxBuffer == 0x5AA55AA5) {
    return (0);                                  
  } else {
    return (1);                                  
  }
}

 





#line 231 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"
#line 1 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\system_stm32l1xx.h"



















 



 



   
  


 









 



 




 

extern uint32_t SystemCoreClock;           



 



 



 



 



 



 
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 
  


   
 
#line 232 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"
#line 233 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



   

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;


typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;



  









 


#line 284 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 



    



 

typedef struct
{
  volatile uint32_t SR;
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SMPR1;
  volatile uint32_t SMPR2;
  volatile uint32_t SMPR3;
  volatile uint32_t JOFR1;
  volatile uint32_t JOFR2;
  volatile uint32_t JOFR3;
  volatile uint32_t JOFR4;
  volatile uint32_t HTR;
  volatile uint32_t LTR;
  volatile uint32_t SQR1;
  volatile uint32_t SQR2;
  volatile uint32_t SQR3;
  volatile uint32_t SQR4;
  volatile uint32_t SQR5;
  volatile uint32_t JSQR;
  volatile uint32_t JDR1;
  volatile uint32_t JDR2;
  volatile uint32_t JDR3;
  volatile uint32_t JDR4;
  volatile uint32_t DR;
  volatile uint32_t SMPR0;
} ADC_TypeDef;

typedef struct
{
  volatile uint32_t CSR;
  volatile uint32_t CCR;
} ADC_Common_TypeDef;




 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t SR;
  volatile uint32_t DINR;
  volatile uint32_t DOUTR;
  volatile uint32_t KEYR0;
  volatile uint32_t KEYR1;
  volatile uint32_t KEYR2;
  volatile uint32_t KEYR3;
  volatile uint32_t IVR0;
  volatile uint32_t IVR1;
  volatile uint32_t IVR2;
  volatile uint32_t IVR3;
} AES_TypeDef;



 

typedef struct
{
  volatile uint32_t CSR;
} COMP_TypeDef;



 

typedef struct
{
  volatile uint32_t DR;
  volatile uint8_t  IDR;
  uint8_t   RESERVED0;
  uint16_t  RESERVED1;
  volatile uint32_t CR;
} CRC_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t SWTRIGR;
  volatile uint32_t DHR12R1;
  volatile uint32_t DHR12L1;
  volatile uint32_t DHR8R1;
  volatile uint32_t DHR12R2;
  volatile uint32_t DHR12L2;
  volatile uint32_t DHR8R2;
  volatile uint32_t DHR12RD;
  volatile uint32_t DHR12LD;
  volatile uint32_t DHR8RD;
  volatile uint32_t DOR1;
  volatile uint32_t DOR2;
  volatile uint32_t SR;  
} DAC_TypeDef;



 

typedef struct
{
  volatile uint32_t IDCODE;
  volatile uint32_t CR;
  volatile uint32_t APB1FZ;
  volatile uint32_t APB2FZ;
}DBGMCU_TypeDef;



 

typedef struct
{
  volatile uint32_t CCR;
  volatile uint32_t CNDTR;
  volatile uint32_t CPAR;
  volatile uint32_t CMAR;
} DMA_Channel_TypeDef;

typedef struct
{
  volatile uint32_t ISR;
  volatile uint32_t IFCR;
} DMA_TypeDef;



 

typedef struct
{
  volatile uint32_t IMR;
  volatile uint32_t EMR;
  volatile uint32_t RTSR;
  volatile uint32_t FTSR;
  volatile uint32_t SWIER;
  volatile uint32_t PR;
} EXTI_TypeDef;



 

typedef struct
{
  volatile uint32_t ACR;
  volatile uint32_t PECR;
  volatile uint32_t PDKEYR;
  volatile uint32_t PEKEYR;
  volatile uint32_t PRGKEYR;
  volatile uint32_t OPTKEYR;
  volatile uint32_t SR;
  volatile uint32_t OBR;
  volatile uint32_t WRPR;
  uint32_t   RESERVED[23];
  volatile uint32_t WRPR1;
  volatile uint32_t WRPR2;       
} FLASH_TypeDef;



 
  
typedef struct
{
  volatile uint32_t RDP;
  volatile uint32_t USER;
  volatile uint32_t WRP01;
  volatile uint32_t WRP23;
  volatile uint32_t WRP45;
  volatile uint32_t WRP67;
  volatile uint32_t WRP89;
  volatile uint32_t WRP1011;
} OB_TypeDef;



 

typedef struct
{
  volatile uint32_t CSR;
  volatile uint32_t OTR;
  volatile uint32_t LPOTR;
} OPAMP_TypeDef;



 

typedef struct
{
  volatile uint32_t BTCR[8];   
} FSMC_Bank1_TypeDef; 



 
  
typedef struct
{
  volatile uint32_t BWTR[7];
} FSMC_Bank1E_TypeDef;



 

typedef struct
{
  volatile uint32_t MODER;
  volatile uint16_t OTYPER;
  uint16_t RESERVED0;
  volatile uint32_t OSPEEDR;
  volatile uint32_t PUPDR;
  volatile uint16_t IDR;
  uint16_t RESERVED1;
  volatile uint16_t ODR;
  uint16_t RESERVED2;
  volatile uint16_t BSRRL;  
  volatile uint16_t BSRRH;  
  volatile uint32_t LCKR;
  volatile uint32_t AFR[2];
  volatile uint16_t BRR;
  uint16_t RESERVED3;
} GPIO_TypeDef;



 

typedef struct
{
  volatile uint32_t MEMRMP;
  volatile uint32_t PMC;
  volatile uint32_t EXTICR[4];
} SYSCFG_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t OAR1;
  uint16_t  RESERVED2;
  volatile uint16_t OAR2;
  uint16_t  RESERVED3;
  volatile uint16_t DR;
  uint16_t  RESERVED4;
  volatile uint16_t SR1;
  uint16_t  RESERVED5;
  volatile uint16_t SR2;
  uint16_t  RESERVED6;
  volatile uint16_t CCR;
  uint16_t  RESERVED7;
  volatile uint16_t TRISE;
  uint16_t  RESERVED8;
} I2C_TypeDef;



 

typedef struct
{
  volatile uint32_t KR;
  volatile uint32_t PR;
  volatile uint32_t RLR;
  volatile uint32_t SR;
} IWDG_TypeDef;




 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t FCR;
  volatile uint32_t SR;
  volatile uint32_t CLR;
  uint32_t RESERVED;
  volatile uint32_t RAM[16];
} LCD_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CSR;
} PWR_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t ICSCR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t AHBRSTR;
  volatile uint32_t APB2RSTR;
  volatile uint32_t APB1RSTR;
  volatile uint32_t AHBENR;
  volatile uint32_t APB2ENR;
  volatile uint32_t APB1ENR;
  volatile uint32_t AHBLPENR;
  volatile uint32_t APB2LPENR;
  volatile uint32_t APB1LPENR;
  volatile uint32_t CSR;
} RCC_TypeDef;



 

typedef struct
{
  volatile uint32_t ICR;
  volatile uint32_t ASCR1;
  volatile uint32_t ASCR2;
  volatile uint32_t HYSCR1;
  volatile uint32_t HYSCR2;
  volatile uint32_t HYSCR3;
  volatile uint32_t HYSCR4;
} RI_TypeDef;



 

typedef struct
{
  volatile uint32_t TR;
  volatile uint32_t DR;
  volatile uint32_t CR;
  volatile uint32_t ISR;
  volatile uint32_t PRER;
  volatile uint32_t WUTR;
  volatile uint32_t CALIBR;
  volatile uint32_t ALRMAR;
  volatile uint32_t ALRMBR;
  volatile uint32_t WPR;
  volatile uint32_t SSR;
  volatile uint32_t SHIFTR;
  volatile uint32_t TSTR;
  volatile uint32_t TSDR;
  volatile uint32_t TSSSR;
  volatile uint32_t CALR;
  volatile uint32_t TAFCR;
  volatile uint32_t ALRMASSR;
  volatile uint32_t ALRMBSSR;
  uint32_t RESERVED7;
  volatile uint32_t BKP0R;
  volatile uint32_t BKP1R;
  volatile uint32_t BKP2R;
  volatile uint32_t BKP3R;
  volatile uint32_t BKP4R;
  volatile uint32_t BKP5R;
  volatile uint32_t BKP6R;
  volatile uint32_t BKP7R;
  volatile uint32_t BKP8R;
  volatile uint32_t BKP9R;
  volatile uint32_t BKP10R;
  volatile uint32_t BKP11R;
  volatile uint32_t BKP12R;
  volatile uint32_t BKP13R;
  volatile uint32_t BKP14R;
  volatile uint32_t BKP15R;
  volatile uint32_t BKP16R;
  volatile uint32_t BKP17R;
  volatile uint32_t BKP18R;
  volatile uint32_t BKP19R;
  volatile uint32_t BKP20R;
  volatile uint32_t BKP21R;
  volatile uint32_t BKP22R;
  volatile uint32_t BKP23R;
  volatile uint32_t BKP24R;
  volatile uint32_t BKP25R;
  volatile uint32_t BKP26R;
  volatile uint32_t BKP27R;
  volatile uint32_t BKP28R;
  volatile uint32_t BKP29R;
  volatile uint32_t BKP30R;
  volatile uint32_t BKP31R;
} RTC_TypeDef;



 

typedef struct
{
  volatile uint32_t POWER;
  volatile uint32_t CLKCR;
  volatile uint32_t ARG;
  volatile uint32_t CMD;
  volatile const uint32_t RESPCMD;
  volatile const uint32_t RESP1;
  volatile const uint32_t RESP2;
  volatile const uint32_t RESP3;
  volatile const uint32_t RESP4;
  volatile uint32_t DTIMER;
  volatile uint32_t DLEN;
  volatile uint32_t DCTRL;
  volatile const uint32_t DCOUNT;
  volatile const uint32_t STA;
  volatile uint32_t ICR;
  volatile uint32_t MASK;
  uint32_t  RESERVED0[2];
  volatile const uint32_t FIFOCNT;
  uint32_t  RESERVED1[13];
  volatile uint32_t FIFO;
} SDIO_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t SR;
  uint16_t  RESERVED2;
  volatile uint16_t DR;
  uint16_t  RESERVED3;
  volatile uint16_t CRCPR;
  uint16_t  RESERVED4;
  volatile uint16_t RXCRCR;
  uint16_t  RESERVED5;
  volatile uint16_t TXCRCR;
  uint16_t  RESERVED6; 
  volatile uint16_t I2SCFGR;
  uint16_t  RESERVED7;
  volatile uint16_t I2SPR;
  uint16_t  RESERVED8;
} SPI_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t      RESERVED0;
  volatile uint16_t CR2;
  uint16_t      RESERVED1;
  volatile uint16_t SMCR;
  uint16_t      RESERVED2;
  volatile uint16_t DIER;
  uint16_t      RESERVED3;
  volatile uint16_t SR;
  uint16_t      RESERVED4;
  volatile uint16_t EGR;
  uint16_t      RESERVED5;
  volatile uint16_t CCMR1;
  uint16_t      RESERVED6;
  volatile uint16_t CCMR2;
  uint16_t      RESERVED7;
  volatile uint16_t CCER;
  uint16_t      RESERVED8;
  volatile uint32_t CNT;
  volatile uint16_t PSC;
  uint16_t      RESERVED10;
  volatile uint32_t ARR;
  uint32_t      RESERVED12;
  volatile uint32_t CCR1;
  volatile uint32_t CCR2;
  volatile uint32_t CCR3;
  volatile uint32_t CCR4;
  uint32_t      RESERVED17;
  volatile uint16_t DCR;
  uint16_t      RESERVED18;
  volatile uint16_t DMAR;
  uint16_t      RESERVED19;
  volatile uint16_t OR;
  uint16_t      RESERVED20;
} TIM_TypeDef;



 
 
typedef struct
{
  volatile uint16_t SR;
  uint16_t  RESERVED0;
  volatile uint16_t DR;
  uint16_t  RESERVED1;
  volatile uint16_t BRR;
  uint16_t  RESERVED2;
  volatile uint16_t CR1;
  uint16_t  RESERVED3;
  volatile uint16_t CR2;
  uint16_t  RESERVED4;
  volatile uint16_t CR3;
  uint16_t  RESERVED5;
  volatile uint16_t GTPR;
  uint16_t  RESERVED6;
} USART_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFR;
  volatile uint32_t SR;
} WWDG_TypeDef;



 
  


 










 




#line 873 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 884 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 895 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"





#line 908 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 915 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"










 
  


   

#line 956 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 973 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 980 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"




#line 992 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"













 



 



 
    
 
 
 
 
 
 
 
 

 
#line 1034 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 1042 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 1051 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

















  
 






































 


















































 


















































 




 













































 


 


 


 


 


 


 






#line 1292 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 1299 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 1306 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 1313 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 1321 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 1328 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 1335 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 1342 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 1349 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 1356 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 1364 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 1371 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 1378 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 1385 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 1392 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 1399 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 1407 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 1414 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 1421 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 1428 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 1435 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 1442 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 1450 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 1457 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 1464 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 1471 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 1478 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 1485 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"


 
#line 1494 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 1501 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 1508 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 1515 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"





 


 


 


 


 


 




 





 
#line 1554 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 





 
 
 
 
 
 













#line 1586 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 




 


 


 


 


 


 


 


 


 


 


 
 
 
 
 

 

































 
 
 
 
 
 
#line 1700 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 





 




 
 
 
 
 

 


 


 


 
 
 
 
 

 









































 



 


 


 


 


 


 


 



 



 



 


 


 



 
 
 
 
 

 


#line 1845 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 









 

#line 1869 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 





 
 
 
 
 

 
#line 1911 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 1941 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 1951 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"















 
#line 1975 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"















 
#line 1999 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"















 
#line 2023 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"















 
#line 2047 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"















 
#line 2071 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"















 
#line 2095 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"















 


 


 


 


 


 


 


 


 


 



 


 


 



 


 


 


 



 


 


 


 


 
 
 
 
 

 
#line 2207 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 2233 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 2259 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 2285 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 2311 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 2337 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
 
 
 
 
 

 






 
#line 2365 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 


 


 


 


 











 
#line 2398 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 


 


 

 
 
 
 
 
 











#line 2434 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 











#line 2457 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 











#line 2480 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 











#line 2503 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 








































 








































 








































 








































 


































 


































 


































 


































 
 
 
 
 
   
#line 2862 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

    
#line 2880 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

   
#line 2930 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

   
#line 2980 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

   
#line 2998 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

    
#line 3016 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

   
#line 3050 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 3069 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 3079 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 3089 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
 
 
 
 

 
#line 3111 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 3120 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"







 



#line 3141 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 



 


 
#line 3166 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 3176 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 




 


 
 
 
 
 

 


 





 


 



 
 
 
 
 

 














 































 
#line 3267 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 



 


 
 
 
 
 

 











 
#line 3302 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"










 
#line 3319 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"





 
 
 
 
 
 


















 



#line 3362 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 




 









 











 
#line 3400 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"






 











 






 












 
#line 3448 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 





 











 
#line 3476 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"






 






 
#line 3498 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 3506 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 3515 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"


 
#line 3532 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"
 
 
#line 3542 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 3564 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 3581 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"


 
#line 3592 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"


 
#line 3615 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 3634 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 3644 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 3666 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 













 







 
#line 3698 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"


 
 
 
 
 
 
#line 3733 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 3763 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 3791 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 3810 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 



 


 



 
#line 3863 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 3905 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 


 


 



 
#line 3944 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 3964 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 


 
#line 3982 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 4004 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 4012 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 4020 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 
 
 
 
 

 









#line 4143 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 4151 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 4159 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 


 


 


 


 





















 




 
 
 
 
 
 
#line 4211 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 


 







  
#line 4232 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



  
#line 4244 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



  
#line 4256 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



  
#line 4267 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 







  
#line 4284 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



  
#line 4295 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



  
#line 4306 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



  
#line 4317 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 







  
#line 4334 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



  
#line 4345 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



  
#line 4356 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



  
#line 4367 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 







  
#line 4384 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



  
#line 4395 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



  
#line 4406 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



  
#line 4417 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"
 
 
 
 
 
 

 

































 
#line 4491 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 4520 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 4539 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 4557 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 4576 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 4594 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 4613 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 4631 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 4650 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
 
 
 
 

 
















 









#line 4692 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 



























 
#line 4737 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 4751 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 4761 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 




























 





















 




























 





















 
#line 4880 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 


 


 


 


 


 


 


 


 
#line 4912 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

#line 4919 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 


 




 
 
 
 
 

 
#line 4945 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 


 



 
#line 4969 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 4978 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"







 
#line 4998 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 5009 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 
 
 
 
 

 
 























 























 























 























 























 























 























 























 
 
#line 5226 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 5238 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 






 
#line 5255 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



     


 
 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


#line 5399 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 


#line 5411 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 


#line 5423 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 


#line 5435 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 


#line 5447 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 


#line 5459 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 


#line 5471 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 


#line 5483 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 

 


#line 5497 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 


#line 5509 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 


#line 5521 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 


#line 5533 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 


#line 5545 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 


#line 5557 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 


#line 5569 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 


#line 5581 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 


#line 5593 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 


#line 5605 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 


#line 5617 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 


#line 5629 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 


#line 5641 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 


#line 5653 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 


#line 5665 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 


#line 5677 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 
 
 
 
 

 
#line 5695 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 
#line 5707 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"







 


 
 
 
 
 

 





 


 


 




 
 
 
 
 

 
#line 5780 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 5815 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 5850 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 5885 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
#line 5920 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 





 





 





 





 





 





 





 





 






 
#line 5987 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 



 









 
#line 6011 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"




 




 
#line 6027 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 





 
#line 6049 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 
 





 
#line 6064 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"
 
#line 6071 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"

 




 






 


 


 



 

 

  

