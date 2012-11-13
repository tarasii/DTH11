#line 1 "main.c"
#line 1 "main.h"
 



 
#line 1 "utils.h"
 



 
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

 




 






 


 


 



 

 

  

#line 1 ".\\stm32l1xx_conf.h"



















  

 



 
 
#line 1 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_adc.h"




















  

 







 
#line 1 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"




































 



 



 
    
#line 6131 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"



 

  

 

 
#line 33 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_adc.h"



 



 

 



 
  
typedef struct
{
  uint32_t ADC_Resolution;                
 
  
  FunctionalState ADC_ScanConvMode;       

 
  
  FunctionalState ADC_ContinuousConvMode; 

 
  
  uint32_t ADC_ExternalTrigConvEdge;      

 
  
  uint32_t ADC_ExternalTrigConv;          

 
  
  uint32_t ADC_DataAlign;                 
 
  
  uint8_t  ADC_NbrOfConversion;           

 
}ADC_InitTypeDef;

typedef struct 
{                                              
  uint32_t ADC_Prescaler;                 

 
}ADC_CommonInitTypeDef;

 



  





  









  




  









  





  












  



  











  



  

 




 




 



 


 



 


#line 196 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_adc.h"


  



  
  







  



  
  
#line 243 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_adc.h"






#line 262 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_adc.h"


  



  

#line 278 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_adc.h"

#line 287 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_adc.h"


  



  

#line 303 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_adc.h"

#line 312 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_adc.h"



 



  











  




  


 



 


 





 


 



 


 


#line 377 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_adc.h"


  



  











  



  
  
#line 408 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_adc.h"

#line 416 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_adc.h"


  



  
  




 




  



  
  
#line 448 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_adc.h"
  

   







  



  
  




  



 
   




  



 
   




  



  
  




  



 
   




  



  
  




  



 
   




  



  

 
  

    
void ADC_DeInit(ADC_TypeDef* ADCx); 

  
void ADC_Init(ADC_TypeDef* ADCx, ADC_InitTypeDef* ADC_InitStruct);
void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct);
void ADC_CommonInit(ADC_CommonInitTypeDef* ADC_CommonInitStruct);
void ADC_CommonStructInit(ADC_CommonInitTypeDef* ADC_CommonInitStruct);
void ADC_Cmd(ADC_TypeDef* ADCx, FunctionalState NewState);

 
void ADC_PowerDownCmd(ADC_TypeDef* ADCx, uint32_t ADC_PowerDown, FunctionalState NewState);
void ADC_DelaySelectionConfig(ADC_TypeDef* ADCx, uint8_t ADC_DelayLength);

 
void ADC_AnalogWatchdogCmd(ADC_TypeDef* ADCx, uint32_t ADC_AnalogWatchdog);
void ADC_AnalogWatchdogThresholdsConfig(ADC_TypeDef* ADCx, uint16_t HighThreshold,uint16_t LowThreshold);
void ADC_AnalogWatchdogSingleChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel);

 
void ADC_TempSensorVrefintCmd(FunctionalState NewState);

 
void ADC_RegularChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);
void ADC_SoftwareStartConv(ADC_TypeDef* ADCx);
FlagStatus ADC_GetSoftwareStartConvStatus(ADC_TypeDef* ADCx);
void ADC_EOCOnEachRegularChannelCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_ContinuousModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_DiscModeChannelCountConfig(ADC_TypeDef* ADCx, uint8_t Number);
void ADC_DiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
uint16_t ADC_GetConversionValue(ADC_TypeDef* ADCx);

 
void ADC_DMACmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_DMARequestAfterLastTransferCmd(ADC_TypeDef* ADCx, FunctionalState NewState);

 
void ADC_InjectedChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);
void ADC_InjectedSequencerLengthConfig(ADC_TypeDef* ADCx, uint8_t Length);
void ADC_SetInjectedOffset(ADC_TypeDef* ADCx, uint8_t ADC_InjectedChannel, uint16_t Offset);
void ADC_ExternalTrigInjectedConvConfig(ADC_TypeDef* ADCx, uint32_t ADC_ExternalTrigInjecConv);
void ADC_ExternalTrigInjectedConvEdgeConfig(ADC_TypeDef* ADCx, uint32_t ADC_ExternalTrigInjecConvEdge);
void ADC_SoftwareStartInjectedConv(ADC_TypeDef* ADCx);
FlagStatus ADC_GetSoftwareStartInjectedConvCmdStatus(ADC_TypeDef* ADCx);
void ADC_AutoInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_InjectedDiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
uint16_t ADC_GetInjectedConversionValue(ADC_TypeDef* ADCx, uint8_t ADC_InjectedChannel);

 
void ADC_ITConfig(ADC_TypeDef* ADCx, uint16_t ADC_IT, FunctionalState NewState);
FlagStatus ADC_GetFlagStatus(ADC_TypeDef* ADCx, uint16_t ADC_FLAG);
void ADC_ClearFlag(ADC_TypeDef* ADCx, uint16_t ADC_FLAG);
ITStatus ADC_GetITStatus(ADC_TypeDef* ADCx, uint16_t ADC_IT);
void ADC_ClearITPendingBit(ADC_TypeDef* ADCx, uint16_t ADC_IT);









  



  

 
#line 29 ".\\stm32l1xx_conf.h"
#line 1 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_crc.h"




















  

 







 
#line 33 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_crc.h"



 



 

 
 



 



 

 
  

void CRC_ResetDR(void);
uint32_t CRC_CalcCRC(uint32_t Data);
uint32_t CRC_CalcBlockCRC(uint32_t pBuffer[], uint32_t BufferLength);
uint32_t CRC_GetCRC(void);
void CRC_SetIDRegister(uint8_t IDValue);
uint8_t CRC_GetIDRegister(void);









 



 

 
#line 30 ".\\stm32l1xx_conf.h"
#line 1 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_comp.h"




















  

 







 
#line 33 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_comp.h"



 



 

 



 
  
typedef struct
{
  uint32_t COMP_Speed;               
 
  uint32_t COMP_InvertingInput;      
 
  uint32_t COMP_OutputSelect;        
 
   
}COMP_InitTypeDef;

 
   


  






 






 


  



 

#line 94 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_comp.h"

#line 103 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_comp.h"


  



 

#line 119 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_comp.h"

#line 128 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_comp.h"


  
  


 








 
  


  

 
 

 
void COMP_DeInit(void);

 
void COMP_Init(COMP_InitTypeDef* COMP_InitStruct);
void COMP_Cmd(FunctionalState NewState);
uint8_t COMP_GetOutputLevel(uint32_t COMP_Selection);

 
void COMP_WindowCmd(FunctionalState NewState);

 
void COMP_VrefintOutputCmd(FunctionalState NewState);









  



 

 
#line 31 ".\\stm32l1xx_conf.h"
#line 1 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_dac.h"




















  

 







 
#line 33 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_dac.h"
 


 



 

 



 
  
typedef struct
{
  uint32_t DAC_Trigger;                      
 

  uint32_t DAC_WaveGeneration;               

 

  uint32_t DAC_LFSRUnmask_TriangleAmplitude; 

 

  uint32_t DAC_OutputBuffer;                 
 
}DAC_InitTypeDef;

 



 



 
  
#line 84 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_dac.h"

#line 93 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_dac.h"
                                 


 
  


 

#line 108 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_dac.h"


 
  


 

#line 140 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_dac.h"

#line 165 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_dac.h"


 



 







 
  


 







 



 

#line 203 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_dac.h"


 



 







 



 





 



  
  





  




  
  

  




  



  

 
  

   
void DAC_DeInit(void);

 
void DAC_Init(uint32_t DAC_Channel, DAC_InitTypeDef* DAC_InitStruct);
void DAC_StructInit(DAC_InitTypeDef* DAC_InitStruct);
void DAC_Cmd(uint32_t DAC_Channel, FunctionalState NewState);
void DAC_SoftwareTriggerCmd(uint32_t DAC_Channel, FunctionalState NewState);
void DAC_DualSoftwareTriggerCmd(FunctionalState NewState);
void DAC_WaveGenerationCmd(uint32_t DAC_Channel, uint32_t DAC_Wave, FunctionalState NewState);
void DAC_SetChannel1Data(uint32_t DAC_Align, uint16_t Data);
void DAC_SetChannel2Data(uint32_t DAC_Align, uint16_t Data);
void DAC_SetDualChannelData(uint32_t DAC_Align, uint16_t Data2, uint16_t Data1);
uint16_t DAC_GetDataOutputValue(uint32_t DAC_Channel);

 
void DAC_DMACmd(uint32_t DAC_Channel, FunctionalState NewState);

 
void DAC_ITConfig(uint32_t DAC_Channel, uint32_t DAC_IT, FunctionalState NewState);
FlagStatus DAC_GetFlagStatus(uint32_t DAC_Channel, uint32_t DAC_FLAG);
void DAC_ClearFlag(uint32_t DAC_Channel, uint32_t DAC_FLAG);
ITStatus DAC_GetITStatus(uint32_t DAC_Channel, uint32_t DAC_IT);
void DAC_ClearITPendingBit(uint32_t DAC_Channel, uint32_t DAC_IT);









  



  

 
#line 32 ".\\stm32l1xx_conf.h"
#line 1 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_dbgmcu.h"




















  

 







 
#line 33 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_dbgmcu.h"



 



 

 
 



 






#line 65 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_dbgmcu.h"








  

 
 

uint32_t DBGMCU_GetREVID(void);
uint32_t DBGMCU_GetDEVID(void);
void DBGMCU_Config(uint32_t DBGMCU_Periph, FunctionalState NewState);
void DBGMCU_APB1PeriphConfig(uint32_t DBGMCU_Periph, FunctionalState NewState);
void DBGMCU_APB2PeriphConfig(uint32_t DBGMCU_Periph, FunctionalState NewState);









 



 

 
#line 33 ".\\stm32l1xx_conf.h"
#line 1 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_dma.h"




















  

 







 
#line 33 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_dma.h"



 



 

 



 

typedef struct
{
  uint32_t DMA_PeripheralBaseAddr;  

  uint32_t DMA_MemoryBaseAddr;      

  uint32_t DMA_DIR;                
 

  uint32_t DMA_BufferSize;         

 

  uint32_t DMA_PeripheralInc;      
 

  uint32_t DMA_MemoryInc;          
 

  uint32_t DMA_PeripheralDataSize; 
 

  uint32_t DMA_MemoryDataSize;     
 

  uint32_t DMA_Mode;               


 

  uint32_t DMA_Priority;           
 

  uint32_t DMA_M2M;                
 
}DMA_InitTypeDef;

 



 

#line 98 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_dma.h"



 







 



 







 



 







 



 

#line 145 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_dma.h"


 



 

#line 159 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_dma.h"


 



 






 



 

#line 186 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_dma.h"


 



 







 



 






#line 239 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_dma.h"



#line 256 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_dma.h"



 



 
#line 292 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_dma.h"



#line 309 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_dma.h"


 



 





 



 

 
 

  
void DMA_DeInit(DMA_Channel_TypeDef* DMAy_Channelx);

 
void DMA_Init(DMA_Channel_TypeDef* DMAy_Channelx, DMA_InitTypeDef* DMA_InitStruct);
void DMA_StructInit(DMA_InitTypeDef* DMA_InitStruct);
void DMA_Cmd(DMA_Channel_TypeDef* DMAy_Channelx, FunctionalState NewState);

 
void DMA_SetCurrDataCounter(DMA_Channel_TypeDef* DMAy_Channelx, uint16_t DataNumber);
uint16_t DMA_GetCurrDataCounter(DMA_Channel_TypeDef* DMAy_Channelx);

 
void DMA_ITConfig(DMA_Channel_TypeDef* DMAy_Channelx, uint32_t DMA_IT, FunctionalState NewState);
FlagStatus DMA_GetFlagStatus(uint32_t DMA_FLAG);
void DMA_ClearFlag(uint32_t DMA_FLAG);
ITStatus DMA_GetITStatus(uint32_t DMA_IT);
void DMA_ClearITPendingBit(uint32_t DMA_IT);









 



 

 
#line 34 ".\\stm32l1xx_conf.h"
#line 1 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_exti.h"




















  

 







 
#line 33 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_exti.h"



 



 

 



 

typedef enum
{
  EXTI_Mode_Interrupt = 0x00,
  EXTI_Mode_Event = 0x04
}EXTIMode_TypeDef;





 

typedef enum
{
  EXTI_Trigger_Rising = 0x08,
  EXTI_Trigger_Falling = 0x0C,  
  EXTI_Trigger_Rising_Falling = 0x10
}EXTITrigger_TypeDef;






 

typedef struct
{
  uint32_t EXTI_Line;               
 
   
  EXTIMode_TypeDef EXTI_Mode;       
 

  EXTITrigger_TypeDef EXTI_Trigger; 
 

  FunctionalState EXTI_LineCmd;     
  
}EXTI_InitTypeDef;

 



 



 

#line 132 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_exti.h"




                                                                                                  


#line 151 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_exti.h"



 



 

 
 
 
void EXTI_DeInit(void);

 
void EXTI_Init(EXTI_InitTypeDef* EXTI_InitStruct);
void EXTI_StructInit(EXTI_InitTypeDef* EXTI_InitStruct);
void EXTI_GenerateSWInterrupt(uint32_t EXTI_Line);

 
FlagStatus EXTI_GetFlagStatus(uint32_t EXTI_Line);
void EXTI_ClearFlag(uint32_t EXTI_Line);
ITStatus EXTI_GetITStatus(uint32_t EXTI_Line);
void EXTI_ClearITPendingBit(uint32_t EXTI_Line);









 



 

 
#line 35 ".\\stm32l1xx_conf.h"
#line 1 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_flash.h"




















  

 







 
#line 33 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_flash.h"



 



 

 



  
typedef enum
{ 
  FLASH_BUSY = 1,
  FLASH_ERROR_WRP,
  FLASH_ERROR_PROGRAM,
  FLASH_COMPLETE,
  FLASH_TIMEOUT
}FLASH_Status;

 
  


  
  


  







  



 
   





  



 
  





  



 
  

#line 133 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_flash.h"







 



  



  


  
 






  



 







 



 







 



 







 



 

#line 208 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_flash.h"

#line 215 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_flash.h"



 
  


  

#line 232 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_flash.h"
 








  



  



















 
  


  




  



  

 
 
  


   
   
void FLASH_SetLatency(uint32_t FLASH_Latency);
void FLASH_PrefetchBufferCmd(FunctionalState NewState);
void FLASH_ReadAccess64Cmd(FunctionalState NewState);
void FLASH_SLEEPPowerDownCmd(FunctionalState NewState);

    
void FLASH_Unlock(void);
void FLASH_Lock(void);
FLASH_Status FLASH_ErasePage(uint32_t Page_Address);
FLASH_Status FLASH_FastProgramWord(uint32_t Address, uint32_t Data);

   
void DATA_EEPROM_Unlock(void);
void DATA_EEPROM_Lock(void);
void DATA_EEPROM_FixedTimeProgramCmd(FunctionalState NewState);
FLASH_Status DATA_EEPROM_EraseWord(uint32_t Address);
FLASH_Status DATA_EEPROM_FastProgramByte(uint32_t Address, uint8_t Data);
FLASH_Status DATA_EEPROM_FastProgramHalfWord(uint32_t Address, uint16_t Data);
FLASH_Status DATA_EEPROM_FastProgramWord(uint32_t Address, uint32_t Data);
FLASH_Status DATA_EEPROM_ProgramByte(uint32_t Address, uint8_t Data);
FLASH_Status DATA_EEPROM_ProgramHalfWord(uint32_t Address, uint16_t Data);
FLASH_Status DATA_EEPROM_ProgramWord(uint32_t Address, uint32_t Data);

 
void FLASH_OB_Unlock(void);
void FLASH_OB_Lock(void);
void FLASH_OB_Launch(void);
FLASH_Status FLASH_OB_WRPConfig(uint32_t OB_WRP, FunctionalState NewState);
FLASH_Status FLASH_OB_RDPConfig(uint8_t OB_RDP);
FLASH_Status FLASH_OB_UserConfig(uint8_t OB_IWDG, uint8_t OB_STOP, uint8_t OB_STDBY);
FLASH_Status FLASH_OB_BORConfig(uint8_t OB_BOR);
uint8_t FLASH_OB_GetUser(void);
uint32_t FLASH_OB_GetWRP(void);
FlagStatus FLASH_OB_GetRDP(void);
uint8_t FLASH_OB_GetBOR(void);

   
void FLASH_ITConfig(uint32_t FLASH_IT, FunctionalState NewState);
FlagStatus FLASH_GetFlagStatus(uint32_t FLASH_FLAG);
void FLASH_ClearFlag(uint32_t FLASH_FLAG);
FLASH_Status FLASH_GetStatus(void);
FLASH_Status FLASH_WaitForLastOperation(uint32_t Timeout);





  
FLASH_Status FLASH_RUNPowerDownCmd(FunctionalState NewState);
FLASH_Status FLASH_ProgramHalfPage(uint32_t Address, uint32_t* pBuffer);
FLASH_Status DATA_EEPROM_EraseDoubleWord(uint32_t Address);
FLASH_Status DATA_EEPROM_ProgramDoubleWord(uint32_t Address, uint64_t Data);
  








 



  

 
#line 36 ".\\stm32l1xx_conf.h"
#line 1 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_gpio.h"




















  

 







 
#line 33 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_gpio.h"



 



 

 

#line 50 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_gpio.h"



  
typedef enum
{ 
  GPIO_Mode_IN   = 0x00,  
  GPIO_Mode_OUT  = 0x01,  
  GPIO_Mode_AF   = 0x02,  
  GPIO_Mode_AN   = 0x03   
}GPIOMode_TypeDef;




 



  
typedef enum
{ GPIO_OType_PP = 0x00,
  GPIO_OType_OD = 0x01
}GPIOOType_TypeDef;




 



  
typedef enum
{ 
  GPIO_Speed_400KHz = 0x00,  
  GPIO_Speed_2MHz   = 0x01,  
  GPIO_Speed_10MHz  = 0x02,  
  GPIO_Speed_40MHz  = 0x03   
}GPIOSpeed_TypeDef;




 



  
typedef enum
{ GPIO_PuPd_NOPULL = 0x00,
  GPIO_PuPd_UP     = 0x01,
  GPIO_PuPd_DOWN   = 0x02
}GPIOPuPd_TypeDef;




 



 
typedef enum
{ Bit_RESET = 0,
  Bit_SET
}BitAction;




 



  
typedef struct
{
  uint32_t GPIO_Pin;              
 

  GPIOMode_TypeDef GPIO_Mode;     
 

  GPIOSpeed_TypeDef GPIO_Speed;   
 

  GPIOOType_TypeDef GPIO_OType;   
 

  GPIOPuPd_TypeDef GPIO_PuPd;     
 
}GPIO_InitTypeDef;

 



 
  


 
#line 170 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_gpio.h"

#line 188 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_gpio.h"


 



  
#line 211 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_gpio.h"

#line 228 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_gpio.h"


 



 



  
#line 245 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_gpio.h"



  



  




  





  




  




  





  



  



  




  


#line 307 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_gpio.h"



 



 
    




 



   
  
 
 

 
void GPIO_DeInit(GPIO_TypeDef* GPIOx);

 
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);
void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct);
void GPIO_PinLockConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

 
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx);
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx);
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal);
void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal);

 
void GPIO_PinAFConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinSource, uint8_t GPIO_AF);









 



 

 
#line 37 ".\\stm32l1xx_conf.h"
#line 1 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_syscfg.h"




















  

 







 
#line 33 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_syscfg.h"



 



  
  
 
 



  
  


  
#line 58 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_syscfg.h"
                                      
#line 65 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_syscfg.h"


 



  
#line 104 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_syscfg.h"


 



  



   






 
  


 










 


  



  
  








  
  


  
  












  
  


  
                                                             
#line 193 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_syscfg.h"

#line 210 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_syscfg.h"



  



  
  
 
#line 246 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_syscfg.h"

   
#line 260 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_syscfg.h"

#line 298 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_syscfg.h"



 



 














 



 
#line 342 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_syscfg.h"





 



 

 
 

 
void SYSCFG_DeInit(void);
void SYSCFG_RIDeInit(void);

  
void SYSCFG_MemoryRemapConfig(uint8_t SYSCFG_MemoryRemap);
void SYSCFG_USBPuCmd(FunctionalState NewState);
void SYSCFG_EXTILineConfig(uint8_t EXTI_PortSourceGPIOx, uint8_t EXTI_PinSourcex);

  
void SYSCFG_RITIMSelect(uint32_t TIM_Select);
void SYSCFG_RITIMInputCaptureConfig(uint32_t RI_InputCapture, uint32_t RI_InputCaptureRouting);
void SYSCFG_RIResistorConfig(uint32_t RI_Resistor, FunctionalState NewState);
void SYSCFG_RISwitchControlModeCmd(FunctionalState NewState);
void SYSCFG_RIIOSwitchConfig(uint32_t RI_IOSwitch, FunctionalState NewState);
void SYSCFG_RIHysteresisConfig(uint8_t RI_Port, uint16_t RI_Pin,
                               FunctionalState NewState);








  



  

 
#line 38 ".\\stm32l1xx_conf.h"
#line 1 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_i2c.h"




















  

 







 
#line 33 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_i2c.h"



 



 

 



 

typedef struct
{
  uint32_t I2C_ClockSpeed;          
 

  uint16_t I2C_Mode;                
 

  uint16_t I2C_DutyCycle;           
 

  uint16_t I2C_OwnAddress1;         
 

  uint16_t I2C_Ack;                 
 

  uint16_t I2C_AcknowledgedAddress; 
 
}I2C_InitTypeDef;

 




 





 

#line 88 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_i2c.h"


 



 







  



 







 



 







 



 







  



 

#line 162 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_i2c.h"


 



 







 



 







  



 







  



 

#line 220 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_i2c.h"



#line 230 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_i2c.h"


 



 



 

#line 249 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_i2c.h"



 

#line 268 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_i2c.h"



#line 282 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_i2c.h"


 



 





 








 
 

























 

 


 





























 

  
 


 
 

 







 

























 

    
 



 



 



























 

  
 

 


 
 


 






 

#line 488 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_i2c.h"


 



 




 



 




 



 

 
 

 
void I2C_DeInit(I2C_TypeDef* I2Cx);

 
void I2C_Init(I2C_TypeDef* I2Cx, I2C_InitTypeDef* I2C_InitStruct);
void I2C_StructInit(I2C_InitTypeDef* I2C_InitStruct);
void I2C_Cmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GenerateSTART(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GenerateSTOP(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_AcknowledgeConfig(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_OwnAddress2Config(I2C_TypeDef* I2Cx, uint8_t Address);
void I2C_DualAddressCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GeneralCallCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_SoftwareResetCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_SMBusAlertConfig(I2C_TypeDef* I2Cx, uint16_t I2C_SMBusAlert);
void I2C_ARPCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_StretchClockCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_FastModeDutyCycleConfig(I2C_TypeDef* I2Cx, uint16_t I2C_DutyCycle);
void I2C_Send7bitAddress(I2C_TypeDef* I2Cx, uint8_t Address, uint8_t I2C_Direction);

  
void I2C_SendData(I2C_TypeDef* I2Cx, uint8_t Data);
uint8_t I2C_ReceiveData(I2C_TypeDef* I2Cx);

  
void I2C_TransmitPEC(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_PECPositionConfig(I2C_TypeDef* I2Cx, uint16_t I2C_PECPosition);
void I2C_CalculatePEC(I2C_TypeDef* I2Cx, FunctionalState NewState);
uint8_t I2C_GetPEC(I2C_TypeDef* I2Cx);

 
void I2C_DMACmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_DMALastTransferCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);


 
uint16_t I2C_ReadRegister(I2C_TypeDef* I2Cx, uint8_t I2C_Register);
void I2C_ITConfig(I2C_TypeDef* I2Cx, uint16_t I2C_IT, FunctionalState NewState);





























































































 





 
ErrorStatus I2C_CheckEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT);




 
uint32_t I2C_GetLastEvent(I2C_TypeDef* I2Cx);




 
FlagStatus I2C_GetFlagStatus(I2C_TypeDef* I2Cx, uint32_t I2C_FLAG);


void I2C_ClearFlag(I2C_TypeDef* I2Cx, uint32_t I2C_FLAG);
ITStatus I2C_GetITStatus(I2C_TypeDef* I2Cx, uint32_t I2C_IT);
void I2C_ClearITPendingBit(I2C_TypeDef* I2Cx, uint32_t I2C_IT);









  



  

 
#line 39 ".\\stm32l1xx_conf.h"
#line 1 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_iwdg.h"




















  

 







 
#line 33 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_iwdg.h"



 



 

 
 



 



 







 



 

#line 79 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_iwdg.h"


 



 







 



 

 
 

 
void IWDG_WriteAccessCmd(uint16_t IWDG_WriteAccess);
void IWDG_SetPrescaler(uint8_t IWDG_Prescaler);
void IWDG_SetReload(uint16_t Reload);
void IWDG_ReloadCounter(void);

 
void IWDG_Enable(void);

 
FlagStatus IWDG_GetFlagStatus(uint16_t IWDG_FLAG);









 



 

 
#line 40 ".\\stm32l1xx_conf.h"
#line 1 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_lcd.h"




















  

 







 
#line 33 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_lcd.h"



 



  

 
 


 

typedef struct
{
  uint32_t LCD_Prescaler;     
 
  uint32_t LCD_Divider;       
 
  uint32_t LCD_Duty;          
 
  uint32_t LCD_Bias;          
  
  uint32_t LCD_VoltageSource; 
 
}LCD_InitTypeDef;


 



 



 

#line 89 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_lcd.h"

#line 106 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_lcd.h"



 
  


 

#line 131 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_lcd.h"

#line 148 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_lcd.h"



 




 
  














  
  



 
  









  
    


 
  





                           


   



 






 


 



 

#line 230 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_lcd.h"

#line 239 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_lcd.h"


 




 

#line 256 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_lcd.h"

#line 265 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_lcd.h"


 



 













     



 

#line 299 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_lcd.h"

#line 308 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_lcd.h"


 



 

#line 324 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_lcd.h"

#line 333 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_lcd.h"


 
      


 

#line 347 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_lcd.h"








    



 

#line 377 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_lcd.h"

#line 394 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_lcd.h"



   
   


 

 
 

 
void LCD_DeInit(void);

 
void LCD_Init(LCD_InitTypeDef* LCD_InitStruct);
void LCD_StructInit(LCD_InitTypeDef* LCD_InitStruct);
void LCD_Cmd(FunctionalState NewState);
void LCD_WaitForSynchro(void);
void LCD_HighDriveCmd(FunctionalState NewState);
void LCD_MuxSegmentCmd(FunctionalState NewState);
void LCD_PulseOnDurationConfig(uint32_t LCD_PulseOnDuration);
void LCD_DeadTimeConfig(uint32_t LCD_DeadTime);
void LCD_BlinkConfig(uint32_t LCD_BlinkMode, uint32_t LCD_BlinkFrequency);
void LCD_ContrastConfig(uint32_t LCD_Contrast);

 
void LCD_Write(uint32_t LCD_RAMRegister, uint32_t LCD_Data);
void LCD_UpdateDisplayRequest(void);

 
void LCD_ITConfig(uint32_t LCD_IT, FunctionalState NewState);
FlagStatus LCD_GetFlagStatus(uint32_t LCD_FLAG);
void LCD_ClearFlag(uint32_t LCD_FLAG);
ITStatus LCD_GetITStatus(uint32_t LCD_IT);
void LCD_ClearITPendingBit(uint32_t LCD_IT);









 



 

 
#line 41 ".\\stm32l1xx_conf.h"
#line 1 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_pwr.h"




















  

 







 
#line 33 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_pwr.h"



 



  

 
 



  



  

#line 66 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_pwr.h"


 



 

#line 80 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_pwr.h"


 

  


 










     
  


 







 



 




 


 
  


 




 


 



 

#line 146 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_pwr.h"








 



 

 
 

  
void PWR_DeInit(void);

  
void PWR_RTCAccessCmd(FunctionalState NewState);

  
void PWR_PVDLevelConfig(uint32_t PWR_PVDLevel);
void PWR_PVDCmd(FunctionalState NewState);

  
void PWR_WakeUpPinCmd(uint32_t PWR_WakeUpPin, FunctionalState NewState);

  
void PWR_FastWakeUpCmd(FunctionalState NewState);
void PWR_UltraLowPowerCmd(FunctionalState NewState);

  
void PWR_VoltageScalingConfig(uint32_t PWR_VoltageScaling);

  
void PWR_EnterLowPowerRunMode(FunctionalState NewState);
void PWR_EnterSleepMode(uint32_t PWR_Regulator, uint8_t PWR_SLEEPEntry);
void PWR_EnterSTOPMode(uint32_t PWR_Regulator, uint8_t PWR_STOPEntry);
void PWR_EnterSTANDBYMode(void);

  
FlagStatus PWR_GetFlagStatus(uint32_t PWR_FLAG);
void PWR_ClearFlag(uint32_t PWR_FLAG);









 



 

 
#line 42 ".\\stm32l1xx_conf.h"
#line 1 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_rcc.h"




















  

 







 
#line 33 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_rcc.h"



 



 

 

typedef struct
{
  uint32_t SYSCLK_Frequency;
  uint32_t HCLK_Frequency;
  uint32_t PCLK1_Frequency;
  uint32_t PCLK2_Frequency;
}RCC_ClocksTypeDef;

 



 



 









  



 

#line 83 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_rcc.h"

#line 91 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_rcc.h"



  
  


 








  



 

#line 122 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_rcc.h"









 



 










 
  


 

#line 160 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_rcc.h"


 



 

#line 182 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_rcc.h"


  



 

#line 198 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_rcc.h"


 
  



 

#line 214 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_rcc.h"












 
  


 








 



 

#line 257 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_rcc.h"


 



 

#line 275 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_rcc.h"






 



 

#line 294 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_rcc.h"




  



 

#line 320 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_rcc.h"




 



 

#line 338 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_rcc.h"







 



 












   



 

#line 381 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_rcc.h"

#line 389 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_rcc.h"






 



 

 
 

 
void RCC_DeInit(void);

 
void RCC_HSEConfig(uint8_t RCC_HSE);
ErrorStatus RCC_WaitForHSEStartUp(void);
void RCC_MSIRangeConfig(uint32_t RCC_MSIRange);
void RCC_AdjustMSICalibrationValue(uint8_t MSICalibrationValue);
void RCC_MSICmd(FunctionalState NewState);
void RCC_AdjustHSICalibrationValue(uint8_t HSICalibrationValue);
void RCC_HSICmd(FunctionalState NewState);
void RCC_LSEConfig(uint8_t RCC_LSE);
void RCC_LSICmd(FunctionalState NewState);
void RCC_PLLConfig(uint8_t RCC_PLLSource, uint8_t RCC_PLLMul, uint8_t RCC_PLLDiv);
void RCC_PLLCmd(FunctionalState NewState);
void RCC_ClockSecuritySystemCmd(FunctionalState NewState);
void RCC_MCOConfig(uint8_t RCC_MCOSource, uint8_t RCC_MCODiv);

 
void RCC_SYSCLKConfig(uint32_t RCC_SYSCLKSource);
uint8_t RCC_GetSYSCLKSource(void);
void RCC_HCLKConfig(uint32_t RCC_SYSCLK);
void RCC_PCLK1Config(uint32_t RCC_HCLK);
void RCC_PCLK2Config(uint32_t RCC_HCLK);
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);

 
void RCC_RTCCLKConfig(uint32_t RCC_RTCCLKSource);
void RCC_RTCCLKCmd(FunctionalState NewState);
void RCC_RTCResetCmd(FunctionalState NewState);

void RCC_AHBPeriphClockCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);
void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);

void RCC_AHBPeriphResetCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);
void RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);

void RCC_AHBPeriphClockLPModeCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);
void RCC_APB2PeriphClockLPModeCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void RCC_APB1PeriphClockLPModeCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);

 
void RCC_ITConfig(uint8_t RCC_IT, FunctionalState NewState);
FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG);
void RCC_ClearFlag(void);
ITStatus RCC_GetITStatus(uint8_t RCC_IT);
void RCC_ClearITPendingBit(uint8_t RCC_IT);









 



  

 
#line 43 ".\\stm32l1xx_conf.h"
#line 1 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_rtc.h"




















  

 







 
#line 33 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_rtc.h"



 



  

 



  
typedef struct
{
  uint32_t RTC_HourFormat;   
 
  
  uint32_t RTC_AsynchPrediv; 
 
  
  uint32_t RTC_SynchPrediv;  
  
}RTC_InitTypeDef;



 
typedef struct
{
  uint8_t RTC_Hours;    


 

  uint8_t RTC_Minutes;  
 
  
  uint8_t RTC_Seconds;  
 

  uint8_t RTC_H12;      
 
}RTC_TimeTypeDef; 



 
typedef struct
{
  uint32_t RTC_WeekDay; 
 
  
  uint32_t RTC_Month;   
 

  uint8_t RTC_Date;     
 
  
  uint8_t RTC_Year;     
 
}RTC_DateTypeDef;



 
typedef struct
{
  RTC_TimeTypeDef RTC_AlarmTime;      

  uint32_t RTC_AlarmMask;            
 

  uint32_t RTC_AlarmDateWeekDaySel;  
 
  
  uint8_t RTC_AlarmDateWeekDay;      



 
}RTC_AlarmTypeDef;

 



  




  






  



  

 


  




  




  



  







  



  






  



  




  



  
#line 203 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_rtc.h"



  



  
  
#line 226 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_rtc.h"


  




  
#line 242 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_rtc.h"



  




  








  




  
#line 272 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_rtc.h"



  



  







  



  
#line 305 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_rtc.h"


  



  






  



  




 







  



  






  




  








  




  











  



  







  



  





    
                               


  







  



                                                                      

#line 457 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_rtc.h"


  



  






  



  
#line 494 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_rtc.h"



  



  
#line 508 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_rtc.h"









  



  


 
  

  
ErrorStatus RTC_DeInit(void);


  
ErrorStatus RTC_Init(RTC_InitTypeDef* RTC_InitStruct);
void RTC_StructInit(RTC_InitTypeDef* RTC_InitStruct);
void RTC_WriteProtectionCmd(FunctionalState NewState);
ErrorStatus RTC_EnterInitMode(void);
void RTC_ExitInitMode(void);
ErrorStatus RTC_WaitForSynchro(void);
ErrorStatus RTC_RefClockCmd(FunctionalState NewState);

  
ErrorStatus RTC_SetTime(uint32_t RTC_Format, RTC_TimeTypeDef* RTC_TimeStruct);
void RTC_TimeStructInit(RTC_TimeTypeDef* RTC_TimeStruct);
void RTC_GetTime(uint32_t RTC_Format, RTC_TimeTypeDef* RTC_TimeStruct);
ErrorStatus RTC_SetDate(uint32_t RTC_Format, RTC_DateTypeDef* RTC_DateStruct);
void RTC_DateStructInit(RTC_DateTypeDef* RTC_DateStruct);
void RTC_GetDate(uint32_t RTC_Format, RTC_DateTypeDef* RTC_DateStruct);

  
void RTC_SetAlarm(uint32_t RTC_Format, uint32_t RTC_Alarm, RTC_AlarmTypeDef* RTC_AlarmStruct);
void RTC_AlarmStructInit(RTC_AlarmTypeDef* RTC_AlarmStruct);
void RTC_GetAlarm(uint32_t RTC_Format, uint32_t RTC_Alarm, RTC_AlarmTypeDef* RTC_AlarmStruct);
ErrorStatus RTC_AlarmCmd(uint32_t RTC_Alarm, FunctionalState NewState);

  
void RTC_WakeUpClockConfig(uint32_t RTC_WakeUpClock);
void RTC_SetWakeUpCounter(uint32_t RTC_WakeUpCounter);
uint32_t RTC_GetWakeUpCounter(void);
ErrorStatus RTC_WakeUpCmd(FunctionalState NewState);

  
void RTC_DayLightSavingConfig(uint32_t RTC_DayLightSaving, uint32_t RTC_StoreOperation);
uint32_t RTC_GetStoreOperation(void);

  
void RTC_OutputConfig(uint32_t RTC_Output, uint32_t RTC_OutputPolarity);

  
ErrorStatus RTC_DigitalCalibConfig(uint32_t RTC_CalibSign, uint32_t Value);
ErrorStatus RTC_DigitalCalibCmd(FunctionalState NewState);
void RTC_CalibOutputCmd(FunctionalState NewState);

  
void RTC_TimeStampCmd(uint32_t RTC_TimeStampEdge, FunctionalState NewState);
void RTC_GetTimeStamp(uint32_t RTC_Format, RTC_TimeTypeDef* RTC_StampTimeStruct, 
                                      RTC_DateTypeDef* RTC_StampDateStruct);
                                  

  
void RTC_TamperTriggerConfig(uint32_t RTC_Tamper, uint32_t RTC_TamperTrigger);
void RTC_TamperCmd(uint32_t RTC_Tamper, FunctionalState NewState);

  
void RTC_WriteBackupRegister(uint32_t RTC_BKP_DR, uint32_t Data);
uint32_t RTC_ReadBackupRegister(uint32_t RTC_BKP_DR);

  
void RTC_OutputTypeConfig(uint32_t RTC_OutputType);


  
void RTC_ITConfig(uint32_t RTC_IT, FunctionalState NewState);
FlagStatus RTC_GetFlagStatus(uint32_t RTC_FLAG);
void RTC_ClearFlag(uint32_t RTC_FLAG);
ITStatus RTC_GetITStatus(uint32_t RTC_IT);
void RTC_ClearITPendingBit(uint32_t RTC_IT);









  



  

 
#line 44 ".\\stm32l1xx_conf.h"
#line 1 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_spi.h"




















  

 







 
#line 33 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_spi.h"



 



  

 



 

typedef struct
{
  uint16_t SPI_Direction;           
 

  uint16_t SPI_Mode;                
 

  uint16_t SPI_DataSize;            
 

  uint16_t SPI_CPOL;                
 

  uint16_t SPI_CPHA;                
 

  uint16_t SPI_NSS;                 

 
 
  uint16_t SPI_BaudRatePrescaler;   



 

  uint16_t SPI_FirstBit;            
 

  uint16_t SPI_CRCPolynomial;        
}SPI_InitTypeDef;

 



 






 
  
#line 102 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_spi.h"


 



 







 



 







  



 







 



 







 



 







  



 

#line 186 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_spi.h"


  



 







 



 






 



 







 



 






 



 







 



 

#line 258 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_spi.h"












 



 

#line 286 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_spi.h"


 



 




 



 

#line 322 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_spi.h"


 
  


 

 
 

  
void SPI_I2S_DeInit(SPI_TypeDef* SPIx);

 
void SPI_Init(SPI_TypeDef* SPIx, SPI_InitTypeDef* SPI_InitStruct);
void SPI_StructInit(SPI_InitTypeDef* SPI_InitStruct);
void SPI_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_DataSizeConfig(SPI_TypeDef* SPIx, uint16_t SPI_DataSize);
void SPI_BiDirectionalLineConfig(SPI_TypeDef* SPIx, uint16_t SPI_Direction);
void SPI_NSSInternalSoftwareConfig(SPI_TypeDef* SPIx, uint16_t SPI_NSSInternalSoft);
void SPI_SSOutputCmd(SPI_TypeDef* SPIx, FunctionalState NewState);

  
void SPI_I2S_SendData(SPI_TypeDef* SPIx, uint16_t Data);
uint16_t SPI_I2S_ReceiveData(SPI_TypeDef* SPIx);

 
void SPI_CalculateCRC(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_TransmitCRC(SPI_TypeDef* SPIx);
uint16_t SPI_GetCRC(SPI_TypeDef* SPIx, uint8_t SPI_CRC);
uint16_t SPI_GetCRCPolynomial(SPI_TypeDef* SPIx);

 
void SPI_I2S_DMACmd(SPI_TypeDef* SPIx, uint16_t SPI_I2S_DMAReq, FunctionalState NewState);

 
void SPI_I2S_ITConfig(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT, FunctionalState NewState);
FlagStatus SPI_I2S_GetFlagStatus(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG);
void SPI_I2S_ClearFlag(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG);
ITStatus SPI_I2S_GetITStatus(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT);
void SPI_I2S_ClearITPendingBit(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT);









 



 

 
#line 45 ".\\stm32l1xx_conf.h"
#line 1 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_tim.h"




















  

 







 
#line 33 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_tim.h"



 



  

 




 

typedef struct
{
  uint16_t TIM_Prescaler;         
 

  uint16_t TIM_CounterMode;       
 

  uint16_t TIM_Period;            

  

  uint16_t TIM_ClockDivision;     
 

} TIM_TimeBaseInitTypeDef;       



 

typedef struct
{
  uint16_t TIM_OCMode;        
 

  uint16_t TIM_OutputState;   
 

  uint16_t TIM_Pulse;         
 

  uint16_t TIM_OCPolarity;    
 

} TIM_OCInitTypeDef;



 

typedef struct
{

  uint16_t TIM_Channel;      
 

  uint16_t TIM_ICPolarity;   
 

  uint16_t TIM_ICSelection;  
 

  uint16_t TIM_ICPrescaler;  
 

  uint16_t TIM_ICFilter;     
 
} TIM_ICInitTypeDef;

 

  


 

#line 124 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_tim.h"

 
#line 132 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_tim.h"

 




 





 
#line 151 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_tim.h"

 






 








 

#line 190 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_tim.h"


 



 







  



 










                                 





  



 

#line 237 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_tim.h"


 



 

#line 255 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_tim.h"


  



 







 




 







  




 







  



 

#line 307 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_tim.h"


  



 

#line 323 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_tim.h"


  



 

#line 339 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_tim.h"


  



 

#line 354 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_tim.h"

#line 361 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_tim.h"


  



 

#line 404 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_tim.h"


  



 

#line 448 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_tim.h"


  



 

#line 463 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_tim.h"



  



 

#line 480 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_tim.h"


  



 

#line 508 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_tim.h"


  



 







  



  






 



 







  



 







  



 

#line 569 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_tim.h"


  




 

#line 585 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_tim.h"
   


  



 

#line 600 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_tim.h"


  



 







  



 





                                     


  



 







  



 

#line 661 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_tim.h"


  



 

#line 677 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_tim.h"


  



 







  
  


 

#line 718 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_tim.h"



  



 




  



 




 



 







 



 













#line 778 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_tim.h"



 



 
  
 
  

 
void TIM_DeInit(TIM_TypeDef* TIMx);
void TIM_TimeBaseInit(TIM_TypeDef* TIMx, TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
void TIM_TimeBaseStructInit(TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
void TIM_PrescalerConfig(TIM_TypeDef* TIMx, uint16_t Prescaler, uint16_t TIM_PSCReloadMode);
void TIM_CounterModeConfig(TIM_TypeDef* TIMx, uint16_t TIM_CounterMode);
void TIM_SetCounter(TIM_TypeDef* TIMx, uint32_t Counter);
void TIM_SetAutoreload(TIM_TypeDef* TIMx, uint32_t Autoreload);
uint32_t TIM_GetCounter(TIM_TypeDef* TIMx);
uint16_t TIM_GetPrescaler(TIM_TypeDef* TIMx);
void TIM_UpdateDisableConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_UpdateRequestConfig(TIM_TypeDef* TIMx, uint16_t TIM_UpdateSource);
void TIM_ARRPreloadConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectOnePulseMode(TIM_TypeDef* TIMx, uint16_t TIM_OPMode);
void TIM_SetClockDivision(TIM_TypeDef* TIMx, uint16_t TIM_CKD);
void TIM_Cmd(TIM_TypeDef* TIMx, FunctionalState NewState);

 
void TIM_OC1Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC2Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC3Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC4Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OCStructInit(TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_SelectOCxM(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_OCMode);
void TIM_SetCompare1(TIM_TypeDef* TIMx, uint32_t Compare1);
void TIM_SetCompare2(TIM_TypeDef* TIMx, uint32_t Compare2);
void TIM_SetCompare3(TIM_TypeDef* TIMx, uint32_t Compare3);
void TIM_SetCompare4(TIM_TypeDef* TIMx, uint32_t Compare4);
void TIM_ForcedOC1Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC2Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC3Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC4Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_OC1PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC2PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC3PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC4PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC1FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC2FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC3FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC4FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_ClearOC1Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC2Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC3Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC4Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_OC1PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC2PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC3PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC4PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_SelectOCREFClear(TIM_TypeDef* TIMx, uint16_t TIM_OCReferenceClear);
void TIM_CCxCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCx);

 
void TIM_ICInit(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_ICStructInit(TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_PWMIConfig(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
uint32_t TIM_GetCapture1(TIM_TypeDef* TIMx);
uint32_t TIM_GetCapture2(TIM_TypeDef* TIMx);
uint32_t TIM_GetCapture3(TIM_TypeDef* TIMx);
uint32_t TIM_GetCapture4(TIM_TypeDef* TIMx);
void TIM_SetIC1Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC2Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC3Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC4Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);

 
void TIM_ITConfig(TIM_TypeDef* TIMx, uint16_t TIM_IT, FunctionalState NewState);
void TIM_GenerateEvent(TIM_TypeDef* TIMx, uint16_t TIM_EventSource);
FlagStatus TIM_GetFlagStatus(TIM_TypeDef* TIMx, uint16_t TIM_FLAG);
void TIM_ClearFlag(TIM_TypeDef* TIMx, uint16_t TIM_FLAG);
ITStatus TIM_GetITStatus(TIM_TypeDef* TIMx, uint16_t TIM_IT);
void TIM_ClearITPendingBit(TIM_TypeDef* TIMx, uint16_t TIM_IT);
void TIM_DMAConfig(TIM_TypeDef* TIMx, uint16_t TIM_DMABase, uint16_t TIM_DMABurstLength);
void TIM_DMACmd(TIM_TypeDef* TIMx, uint16_t TIM_DMASource, FunctionalState NewState);
void TIM_SelectCCDMA(TIM_TypeDef* TIMx, FunctionalState NewState);

 
void TIM_InternalClockConfig(TIM_TypeDef* TIMx);
void TIM_ITRxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);
void TIM_TIxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_TIxExternalCLKSource,
                                uint16_t TIM_ICPolarity, uint16_t ICFilter);
void TIM_ETRClockMode1Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
                             uint16_t ExtTRGFilter);
void TIM_ETRClockMode2Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, 
                             uint16_t TIM_ExtTRGPolarity, uint16_t ExtTRGFilter);


 
void TIM_SelectInputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);
void TIM_SelectOutputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_TRGOSource);
void TIM_SelectSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_SlaveMode);
void TIM_SelectMasterSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_MasterSlaveMode);
void TIM_ETRConfig(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
                   uint16_t ExtTRGFilter);

                    
void TIM_EncoderInterfaceConfig(TIM_TypeDef* TIMx, uint16_t TIM_EncoderMode,
                                uint16_t TIM_IC1Polarity, uint16_t TIM_IC2Polarity);
void TIM_SelectHallSensor(TIM_TypeDef* TIMx, FunctionalState NewState);

 
void TIM_RemapConfig(TIM_TypeDef* TIMx, uint16_t TIM_Remap);










  



 

 
#line 46 ".\\stm32l1xx_conf.h"
#line 1 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_usart.h"




















  

 







 
#line 33 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_usart.h"



 



  

  



  
  
typedef struct
{
  uint32_t USART_BaudRate;            


 

  uint16_t USART_WordLength;          
 

  uint16_t USART_StopBits;            
 

  uint16_t USART_Parity;              




 
 
  uint16_t USART_Mode;                
 

  uint16_t USART_HardwareFlowControl; 

 
} USART_InitTypeDef;



  
  
typedef struct
{

  uint16_t USART_Clock;   
 

  uint16_t USART_CPOL;    
 

  uint16_t USART_CPHA;    
 

  uint16_t USART_LastBit; 

 
} USART_ClockInitTypeDef;

 



  
  






  
  


                                    




  



  
  
#line 132 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_usart.h"


  



  
  
#line 146 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_usart.h"


  



  
  





  



  
#line 173 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_usart.h"


  



  






  



 
  






  



 







 



 







  



 
  
#line 250 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_usart.h"


 



 







  



 







 



 
  







 



 







  



 

#line 322 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_usart.h"
                              








  



  

 
  

  
void USART_DeInit(USART_TypeDef* USARTx);

 
void USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct);
void USART_StructInit(USART_InitTypeDef* USART_InitStruct);
void USART_ClockInit(USART_TypeDef* USARTx, USART_ClockInitTypeDef* USART_ClockInitStruct);
void USART_ClockStructInit(USART_ClockInitTypeDef* USART_ClockInitStruct);
void USART_Cmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SetPrescaler(USART_TypeDef* USARTx, uint8_t USART_Prescaler);
void USART_OverSampling8Cmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_OneBitMethodCmd(USART_TypeDef* USARTx, FunctionalState NewState);

  
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);
uint16_t USART_ReceiveData(USART_TypeDef* USARTx);

 
void USART_SetAddress(USART_TypeDef* USARTx, uint8_t USART_Address);
void USART_WakeUpConfig(USART_TypeDef* USARTx, uint16_t USART_WakeUp);
void USART_ReceiverWakeUpCmd(USART_TypeDef* USARTx, FunctionalState NewState);

 
void USART_LINBreakDetectLengthConfig(USART_TypeDef* USARTx, uint16_t USART_LINBreakDetectLength);
void USART_LINCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SendBreak(USART_TypeDef* USARTx);

 
void USART_HalfDuplexCmd(USART_TypeDef* USARTx, FunctionalState NewState);

 
void USART_SmartCardCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SmartCardNACKCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SetGuardTime(USART_TypeDef* USARTx, uint8_t USART_GuardTime);

 
void USART_IrDAConfig(USART_TypeDef* USARTx, uint16_t USART_IrDAMode);
void USART_IrDACmd(USART_TypeDef* USARTx, FunctionalState NewState);

 
void USART_DMACmd(USART_TypeDef* USARTx, uint16_t USART_DMAReq, FunctionalState NewState);

 
void USART_ITConfig(USART_TypeDef* USARTx, uint16_t USART_IT, FunctionalState NewState);
FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint16_t USART_FLAG);
void USART_ClearFlag(USART_TypeDef* USARTx, uint16_t USART_FLAG);
ITStatus USART_GetITStatus(USART_TypeDef* USARTx, uint16_t USART_IT);
void USART_ClearITPendingBit(USART_TypeDef* USARTx, uint16_t USART_IT);









  



  

 
#line 47 ".\\stm32l1xx_conf.h"
#line 1 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_wwdg.h"




















  

 







 
#line 33 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_wwdg.h"



 



  

 
 



  
  


  
  
#line 63 "..\\STM32L1xx_StdPeriph_Driver\\inc\\stm32l1xx_wwdg.h"



  



  

 
 
   
void WWDG_DeInit(void);

 
void WWDG_SetPrescaler(uint32_t WWDG_Prescaler);
void WWDG_SetWindowValue(uint8_t WindowValue);
void WWDG_EnableIT(void);
void WWDG_SetCounter(uint8_t Counter);

 
void WWDG_Enable(uint8_t Counter);

 
FlagStatus WWDG_GetFlagStatus(void);
void WWDG_ClearFlag(void);









  



  

 
#line 48 ".\\stm32l1xx_conf.h"
#line 1 "..\\STM32L1xx_StdPeriph_Driver\\inc\\misc.h"




















  

 







 
#line 33 "..\\STM32L1xx_StdPeriph_Driver\\inc\\misc.h"



 



 

 



 

typedef struct
{
  uint8_t NVIC_IRQChannel;                    


 

  uint8_t NVIC_IRQChannelPreemptionPriority;  

 

  uint8_t NVIC_IRQChannelSubPriority;         

 

  FunctionalState NVIC_IRQChannelCmd;         

    
} NVIC_InitTypeDef;

























 

 



 



 







 



 

#line 122 "..\\STM32L1xx_StdPeriph_Driver\\inc\\misc.h"


 



 

#line 140 "..\\STM32L1xx_StdPeriph_Driver\\inc\\misc.h"















 



 







 



 

 
  

void NVIC_PriorityGroupConfig(uint32_t NVIC_PriorityGroup);
void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);
void NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset);
void NVIC_SystemLPConfig(uint8_t LowPowerMode, FunctionalState NewState);
void SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource);









 



 

 
#line 49 ".\\stm32l1xx_conf.h"

 
 

 
 

 
#line 72 ".\\stm32l1xx_conf.h"



 
#line 6102 "..\\..\\..\\..\\INC\\ST\\STM32L1xx\\stm32l1xx.h"




 

















 









 

  

 

 
#line 7 "utils.h"
#line 1 "C:\\PRG\\Keil\\ARM\\RV31\\Inc\\stdio.h"
 
 
 





 






 









#line 34 "C:\\PRG\\Keil\\ARM\\RV31\\Inc\\stdio.h"


  
  typedef unsigned int size_t;    








 
 

 
  typedef struct __va_list __va_list;





   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 125 "C:\\PRG\\Keil\\ARM\\RV31\\Inc\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 
extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 944 "C:\\PRG\\Keil\\ARM\\RV31\\Inc\\stdio.h"



 
#line 8 "utils.h"


 
 











 
 




 
void Delay(uint32_t nTime);

#line 7 "main.h"
#line 1 "ext_lcd.h"



#line 5 "ext_lcd.h"







 
#line 22 "ext_lcd.h"





 





















void Write_LCD( unsigned char* message );
void Init_Ext_LCD(void);
void Clr_LCD(void);
void GotoHome(void);
void GotoXY(uint8_t pos, uint8_t line);

void LCD_CURS(uint8_t enable, uint8_t curs, uint8_t blink);
void LCD_ON(uint8_t curs, uint8_t blink);
void LCD_OFF(void);



#line 8 "main.h"
#line 1 "onewire.h"



#line 5 "onewire.h"
#line 6 "onewire.h"



















uint8_t OW_Send(uint8_t sendReset, uint8_t *command, uint8_t cLen, uint8_t *data, uint8_t dLen, uint8_t readStart);
uint8_t OW_Scan(uint8_t *buf, uint8_t num);

#line 9 "main.h"


















#line 33 "main.h"







 




 












 
typedef struct
{
    uint16_t myVREF;
    uint16_t TS_CAL_COLD;
    uint16_t reserved;
    uint16_t TS_CAL_HOT;
} CALIB_TypeDef;

 
 
 

void TimingDelay_Decrement(void);
void Init_ext_LCD_GPIOs (void);
void Init_GPIOs (void);
void RCC_Configuration(void);
uint16_t uint16_time_diff(uint16_t now, uint16_t before);
void configureADC_Temp(void);
void DAC_Config(void);
void configureDMA(void);
void processTempData(void);
uint16_t GetTemperature(uint8_t *idbuf);
float CalculateTemperature(uint16_t dirtytemp);
void acquireTemperatureData(void);
void setADCDMA_TransferComplete(void);
void clearADCDMA_TransferComplete(void);


#line 2 "main.c"

static volatile uint32_t TimingDelay;
RCC_ClocksTypeDef RCC_Clocks;


ADC_InitTypeDef ADC_InitStructure;
ADC_CommonInitTypeDef ADC_CommonInitStructure;


DMA_InitTypeDef DMA_InitStructure;


CALIB_TypeDef calibdata;     
volatile _Bool flag_ADCDMA_TransferComplete;
uint16_t ADC_ConvertedValueBuff[20];

uint32_t refAVG, tempAVG, preasureAVG;
int32_t temperature_C;
float voltage_V, preasure_V; 

volatile uint16_t systick_ms = 0;

float humidity, capacitance;

float temperature;
uint16_t temperature_data;
uint8_t idbuf[2][8];
uint8_t num_ow;

volatile uint16_t dirty_cycle = 0, period = 0; 

volatile uint8_t mode = 0, first_time_in_mode = 1, flag_UserButton = 0;

void SysTick_Handler(void) {
    TimingDelay_Decrement();
}

void EXTI0_IRQHandler(void)
{
   
	flag_UserButton = 1;
  EXTI_ClearITPendingBit(((uint32_t)0x00000001));
}

void TIM2_IRQHandler(void)
{
  
  
     
    TIM_ClearITPendingBit(((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000)), ((uint16_t)0x0004));

	  period = TIM_GetCapture1(((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000)));
    dirty_cycle = TIM_GetCapture2(((TIM_TypeDef *) (((uint32_t)0x40000000) + 0x0000)));
	
}

void DMA1_Channel1_IRQHandler    (void)
{
  DMA_ClearFlag(((uint32_t)0x00000002));
  setADCDMA_TransferComplete();   
}



int main(void){
	
	uint8_t buf[5], res;
	
	
	char strDisp[25];
	
	
	
	RTC_DateTypeDef RTCDateStr;
	RTC_TimeTypeDef RTCTimeStr;
	
	
	
	RCC_Configuration();
	
	Init_GPIOs();
	
	Init_ext_LCD_GPIOs();
		
	configureDMA();
	
	configureADC_Temp();	
	
	
	Init_Ext_LCD();
	
	
	Write_LCD("HUMIDITY SENSOR");
	
	
	
	
  
	
		
	
	while(1){
	
    if (flag_UserButton == 1){
			if(++mode == 3){mode = 0;}
      flag_UserButton = 0;
			first_time_in_mode = 1;
    }

 
		

 		
 				 
 		

 		
 		
		
		
		res = OW_Send(1, "\xff", 1, buf, 5, 0);

		

		
		switch (mode){
			case 0:				
				if (first_time_in_mode==1) {
					
					Clr_LCD();
					first_time_in_mode = 0;
					
					((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x0400))->BSRRH = ((uint16_t)0x0080);
					((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x0400))->BSRRH = ((uint16_t)0x0040);
					
				}
				
 				
 				
 				

				
				sprintf(strDisp, "%d %02x%02x%02x%02x%02x", res, buf[0],buf[1],buf[2],buf[3],buf[4]);
 				GotoXY(0,1);
 				Write_LCD((unsigned char *) strDisp);

				
				break;
			case 1:
				if (first_time_in_mode==1) {
					
					Clr_LCD();
					first_time_in_mode = 0;
					
					((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x0400))->ODR ^= ((uint16_t)0x0080);
					((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x0000))->BSRRH = ((uint16_t)0x0004);
				}
					
 				sprintf(strDisp, "Page 1");
 				GotoXY(0,0);
 				Write_LCD((unsigned char *) strDisp);
										
				break;
			case 2:
				if (first_time_in_mode==1) {
					
					Clr_LCD();
					
					first_time_in_mode = 0;
					
					((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x0400))->ODR ^= ((uint16_t)0x0080);
					((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x0400))->ODR ^= ((uint16_t)0x0040);
					((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x0000))->BSRRL = ((uint16_t)0x0004);
					
				}
					
 				sprintf(strDisp, "Page 2");
 				GotoXY(0,0);
				Write_LCD((unsigned char *) strDisp);
				break;
		}			

	}
	
}

void acquireTemperatureData(void)
{
  
  
  

   
  
  
   
  

   
  

   
  DMA_DeInit(((DMA_Channel_TypeDef *) (((((uint32_t)0x40000000) + 0x20000) + 0x6000) + 0x0008)));
  DMA_Init(((DMA_Channel_TypeDef *) (((((uint32_t)0x40000000) + 0x20000) + 0x6000) + 0x0008)), &DMA_InitStructure);
  DMA_Cmd(((DMA_Channel_TypeDef *) (((((uint32_t)0x40000000) + 0x20000) + 0x6000) + 0x0008)), ENABLE);
  
   
  DMA_ITConfig(((DMA_Channel_TypeDef *) (((((uint32_t)0x40000000) + 0x20000) + 0x6000) + 0x0008)), ((uint32_t)0x00000002), ENABLE);

    
  ADC_DMACmd(((ADC_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2400)), DISABLE);

      
  ADC_DMACmd(((ADC_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2400)), ENABLE);
  
   
  clearADCDMA_TransferComplete(); 
  
   
  ADC_SoftwareStartConv(((ADC_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2400)));
}

void insertionSort(uint16_t *numbers, uint32_t array_size) 
{
  
	uint32_t i, j;
	uint32_t index;

  for (i=1; i < array_size; i++) {
    index = numbers[i];
    j = i;
    while ((j > 0) && (numbers[j-1] > index)) {
      numbers[j] = numbers[j-1];
      j = j - 1;
    }
    numbers[j] = index;
  }
}

uint32_t interquartileMean(uint16_t *array, uint32_t numOfSamples)
{
    uint32_t sum=0;
    uint32_t  index, maxindex;
      
	maxindex = 3 * numOfSamples / 4;
    for (index = (numOfSamples / 4); index < maxindex; index++){
            sum += array[index];
    }
	 
    return ( sum / (numOfSamples / 2) );
}

void processTempData(void)
{
  uint32_t index, dataSum;

   
  insertionSort(ADC_ConvertedValueBuff, 12);
  
   
  
	tempAVG = ADC_ConvertedValueBuff[0];

  
  
  
  
  
  
  
  dataSum = 0;
    
  for (index=12; index < 20-4; index++){
    dataSum += ADC_ConvertedValueBuff[index];
  }
   
  preasureAVG = dataSum / 4 ;

  dataSum = 0;
    
  for (index=12+4; index < 20; index++){
    dataSum += ADC_ConvertedValueBuff[index];
  }
   
  refAVG = dataSum / 4 ;


   
  temperature_C = tempAVG - (int32_t) calibdata.TS_CAL_COLD;	
  temperature_C = temperature_C * (int32_t)(110 - 25);                      
  temperature_C = temperature_C / 
                  (int32_t)(calibdata.TS_CAL_HOT - calibdata.TS_CAL_COLD); 
  temperature_C = temperature_C + 25; 
	
   
  voltage_V = (1.224L/refAVG) * 4096;

   
  preasure_V = (755.0 * preasureAVG) / 1020;
	
}

uint16_t GetTemperature(uint8_t *idbuf){
	uint16_t dirtytemp;
 	uint8_t buf[2];
 	uint8_t cmd[12];
 	uint8_t i;
	



 	
	cmd[0]=0x55;
	for (i=1;i<9;i++)
	{
		cmd[i]=idbuf[i-1];
	}
	cmd[9]=0x44;
	
 	OW_Send(1, cmd, 10, 0, 0, 0xff);
  
	Delay(10);
	
	cmd[9]=0xbe;
	cmd[10]=0xff;
	cmd[11]=0xff;
  OW_Send(1, cmd, 12, buf, 2, 10);
	
	dirtytemp = buf[1]*0x100+buf[0];
	
	return dirtytemp;
}



float CalculateTemperature(uint16_t dirtytemp){
	float temp;
	if( dirtytemp > 1000 ) { 
		dirtytemp = 4096 - dirtytemp; 
		dirtytemp = -dirtytemp; 
	}
	temp = dirtytemp * 0.0625;
	if (temp>125){temp=0;}
	return temp;
}



void Init_ext_LCD_GPIOs (void){
  GPIO_InitTypeDef GPIO_InitStructure;
         
 
   
  GPIO_InitStructure.GPIO_Pin = ((uint16_t)0x0020) | ((uint16_t)0x0010);  


  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init( ((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x0400)), &GPIO_InitStructure);
    
 
   
  GPIO_InitStructure.GPIO_Pin = ((uint16_t)0x0001) | ((uint16_t)0x0002) | ((uint16_t)0x0004) | ((uint16_t)0x0008) ;                               
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init( ((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x0800)), &GPIO_InitStructure);  
	
} 

void RCC_Configuration(void){

  
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 2000);

   
  RCC_AHBPeriphClockCmd(((uint32_t)0x00000001) | ((uint32_t)0x00000002) | ((uint32_t)0x00000004)| ((uint32_t)0x00000008)| ((uint32_t)0x00000010)| ((uint32_t)0x00000020), ENABLE);     
	RCC_AHBPeriphClockCmd(((uint32_t)0x01000000), ENABLE);
	
	RCC_APB1PeriphClockCmd(((uint32_t)0x00000001) | ((uint32_t)0x00020000) | ((uint32_t)0x20000000), ENABLE);	
	
	
  RCC_APB2PeriphClockCmd(((uint32_t)0x00000200) | ((uint32_t)0x00000001), ENABLE);

	 
  PWR_RTCAccessCmd(ENABLE);





   
  RCC_LSEConfig(((uint8_t)0x01)); 

   
	while (RCC_GetFlagStatus(((uint8_t)0x49)) == RESET)
	{}
  
  RCC_RTCCLKCmd(ENABLE);
   
   
  RCC_RTCCLKConfig(((uint32_t)0x00010000)); 
}  


void Init_GPIOs (void){
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
  

  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

   
  GPIO_InitStructure.GPIO_Pin = ((uint16_t)0x0001);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_Init(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x0000)), &GPIO_InitStructure);

   
  SYSCFG_EXTILineConfig(((uint8_t)0x00),((uint8_t)0x00));

   
  EXTI_InitStructure.EXTI_Line = ((uint32_t)0x00000001) ;  
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

   
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure); 

 
  GPIO_InitStructure.GPIO_Pin = ((uint16_t)0x0080)|((uint16_t)0x0040);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x0400)), &GPIO_InitStructure);
  ((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x0400))->BSRRH = ((uint16_t)0x0080);	
  ((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x0400))->BSRRH = ((uint16_t)0x0040);
  
 




 








	








	
 	

	
	GPIO_InitStructure.GPIO_Pin = ((uint16_t)0x0004);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  
  
  
  GPIO_Init(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x0000)), &GPIO_InitStructure);
		
  GPIO_PinAFConfig(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x0000)), ((uint8_t)0x02), ((uint8_t)0x07));

	GPIO_InitStructure.GPIO_Pin = ((uint16_t)0x0008);
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  
  
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x0000)), &GPIO_InitStructure);
		
  GPIO_PinAFConfig(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x0000)), ((uint8_t)0x03), ((uint8_t)0x07));

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = ((uint16_t)0x0000);
	USART_InitStructure.USART_StopBits = ((uint16_t)0x0000);
	USART_InitStructure.USART_Parity = ((uint16_t)0x0000);
	USART_InitStructure.USART_HardwareFlowControl = ((uint16_t)0x0000);
	USART_InitStructure.USART_Mode = ((uint16_t)0x0008) | ((uint16_t)0x0004);

	USART_Init(((USART_TypeDef *) (((uint32_t)0x40000000) + 0x4400)), &USART_InitStructure);
	USART_Cmd(((USART_TypeDef *) (((uint32_t)0x40000000) + 0x4400)), ENABLE);

 
  GPIO_InitStructure.GPIO_Pin = ((uint16_t)0x0010)  ;                               
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_Init( ((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x0000)), &GPIO_InitStructure);





   




} 

FunctionalState  testUserCalibData(void)
{
  int32_t testdiff;
  FunctionalState retval = DISABLE;
  
  testdiff = ((CALIB_TypeDef *) ((uint32_t)0x08080000))->TS_CAL_HOT - ((CALIB_TypeDef *) ((uint32_t)0x08080000))->TS_CAL_COLD;
  
  if ( testdiff > (int32_t) 50 )    retval = ENABLE;
  
  return retval;
}

FunctionalState  testFactoryCalibData(void)
{
  int32_t testdiff;
  FunctionalState retval = DISABLE;
  
  testdiff = ((CALIB_TypeDef *) ((uint32_t)0x1FF80078))->TS_CAL_HOT - ((CALIB_TypeDef *) ((uint32_t)0x1FF80078))->TS_CAL_COLD;
  
  if ( testdiff > (int32_t) 50 )    retval = ENABLE;
  
  return retval;
}

void  writeCalibData(CALIB_TypeDef* calibStruct)
{
	volatile FLASH_Status FLASHStatus = FLASH_COMPLETE; 

  uint32_t  Address = 0;
  uint32_t  dataToWrite;
  
   
  DATA_EEPROM_Unlock();
  
         
  FLASH_ClearFlag(((uint32_t)0x00000002)|((uint32_t)0x00000100) | ((uint32_t)0x00000200)
                  | ((uint32_t)0x00000400) | ((uint32_t)0x00000800));	
  
  
   
 
  Address = (uint32_t) ((CALIB_TypeDef *) ((uint32_t)0x08080000));


  dataToWrite = 0x00;
  dataToWrite = (uint32_t)(calibStruct->TS_CAL_COLD) << 16;
  
  FLASHStatus = DATA_EEPROM_ProgramWord(Address, dataToWrite);

  if(FLASHStatus != FLASH_COMPLETE)
  {
    while(1);  
  }

  Address += 4;

  dataToWrite = 0x00;
  dataToWrite = (uint32_t)(calibStruct->TS_CAL_HOT) << 16;
  
  FLASHStatus = DATA_EEPROM_ProgramWord(Address, dataToWrite);
  
}

void configureADC_Temp(void)
{
  uint32_t ch_index;
	volatile uint16_t 	T_StartupTimeDelay;

   
  if ( testUserCalibData() == ENABLE ) calibdata = *((CALIB_TypeDef *) ((uint32_t)0x08080000));
  else if ( testFactoryCalibData() == ENABLE ) calibdata = *((CALIB_TypeDef *) ((uint32_t)0x1FF80078));
  else {
    calibdata.TS_CAL_COLD = 0x2A8;
    calibdata.TS_CAL_HOT = 0x362;
    writeCalibData(&calibdata);
    calibdata = *((CALIB_TypeDef *) ((uint32_t)0x08080000));
  }

   
  
  
   
  ADC_TempSensorVrefintCmd(ENABLE); 
  
   
  T_StartupTimeDelay = 1024;
  while (T_StartupTimeDelay--);

   
  ADC_CommonInitStructure.ADC_Prescaler = ((uint32_t)0x00020000);
  ADC_CommonInit(&ADC_CommonInitStructure);
  
  
   
  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_Resolution = ((uint32_t)0x00000000);	          
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;	                          
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;			  
  ADC_InitStructure.ADC_ExternalTrigConv = ((uint32_t)0x00000000); 
  ADC_InitStructure.ADC_DataAlign = ((uint32_t)0x00000000);                  
  ADC_InitStructure.ADC_NbrOfConversion = 20;             
  
  ADC_Init(((ADC_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2400)), &ADC_InitStructure);

    

    for (ch_index = 1; ch_index <= 12; ch_index++){
      ADC_RegularChannelConfig(((ADC_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2400)), ((uint8_t)0x10), ch_index, 
                               ((uint8_t)0x07));
    }

  ADC_RegularChannelConfig(((ADC_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2400)), ((uint8_t)0x0D), 13, ((uint8_t)0x07));
  ADC_RegularChannelConfig(((ADC_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2400)), ((uint8_t)0x0D), 14, ((uint8_t)0x07));
  ADC_RegularChannelConfig(((ADC_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2400)), ((uint8_t)0x0D), 15, ((uint8_t)0x07));
  ADC_RegularChannelConfig(((ADC_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2400)), ((uint8_t)0x0D), 16, ((uint8_t)0x07));

	ADC_RegularChannelConfig(((ADC_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2400)), ((uint8_t)0x11), 17, ((uint8_t)0x07));
  ADC_RegularChannelConfig(((ADC_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2400)), ((uint8_t)0x11), 18, ((uint8_t)0x07));
  ADC_RegularChannelConfig(((ADC_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2400)), ((uint8_t)0x11), 19, ((uint8_t)0x07));
  ADC_RegularChannelConfig(((ADC_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2400)), ((uint8_t)0x11), 20, ((uint8_t)0x07));

   
  ADC_Cmd(((ADC_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2400)), ENABLE);

   
  while(ADC_GetFlagStatus(((ADC_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2400)), ((uint16_t)0x0040)) == RESET); 
		
}

void configureDMA(void)
{
   
  NVIC_InitTypeDef NVIC_InitStructure;
  
   
  

   
  DMA_DeInit(((DMA_Channel_TypeDef *) (((((uint32_t)0x40000000) + 0x20000) + 0x6000) + 0x0008)));
  
  
	
	 
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(((ADC_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2400))->DR);	     
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_ConvertedValueBuff;  
  DMA_InitStructure.DMA_DIR = ((uint32_t)0x00000000);                         
  DMA_InitStructure.DMA_BufferSize = 20;                     
  DMA_InitStructure.DMA_PeripheralInc = ((uint32_t)0x00000000);	     
  DMA_InitStructure.DMA_MemoryInc = ((uint32_t)0x00000080);                    
  DMA_InitStructure.DMA_PeripheralDataSize = ((uint32_t)0x00000100);
  DMA_InitStructure.DMA_MemoryDataSize = ((uint32_t)0x00000400);	     
  DMA_InitStructure.DMA_Mode = ((uint32_t)0x00000000);                              
  DMA_InitStructure.DMA_Priority = ((uint32_t)0x00002000);	                     
  DMA_InitStructure.DMA_M2M = ((uint32_t)0x00000000);                               
  DMA_Init(((DMA_Channel_TypeDef *) (((((uint32_t)0x40000000) + 0x20000) + 0x6000) + 0x0008)), &DMA_InitStructure);								 

    
  DMA_ITConfig(((DMA_Channel_TypeDef *) (((((uint32_t)0x40000000) + 0x20000) + 0x6000) + 0x0008)), ((uint32_t)0x00000002), ENABLE);
  
   
  NVIC_InitStructure.NVIC_IRQChannel =   DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
}

uint16_t uint16_time_diff(uint16_t now, uint16_t before)
{
  return (now >= before) ? (now - before) : (65535 - before + now);
}

void Delay(uint32_t nTime){
  TimingDelay = nTime;

  while(TimingDelay != 0);
  
}

void TimingDelay_Decrement(void){

  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }	
	++systick_ms;
}

void setADCDMA_TransferComplete(void)
{
  flag_ADCDMA_TransferComplete = !0;
}

void clearADCDMA_TransferComplete(void)
{
  flag_ADCDMA_TransferComplete = 0;
}


