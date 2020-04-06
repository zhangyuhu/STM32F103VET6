#include <board.h>
#include <rtthread.h>
#include <stdio.h>

#include "stm32f1xx_hal_conf.h"
#include "drv_dm9000x.h"

#define DBG_TAG "dm9000x"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/*
DATA0  DATA1  DATA2  DATA3  DATA4  DATA5  DATA6  DATA7  DATA8  DATA9  DATA10  DATA11  DATA12  DATA13  DATA14  DATA15
PD14   PD15	  PD0    PD1    PE7	   PE8	  PE9    PE10   PE11   PE12   PE13    PE14    PE15    PD8     PD9 	  PD10
*/

#define  EXINIT                        GPIO_PIN_0    // PB0
#define  CMD                           GPIO_PIN_11 	 // PD11
#define	 CS                            GPIO_PIN_7	 // PD7
#define  IOW                           GPIO_PIN_5    // PD5
#define  IOR                           GPIO_PIN_4    // PD4
#define  D0                            GPIO_PIN_14 	 // PD14
#define  D1                            GPIO_PIN_15 	 // PD15
#define  D2                            GPIO_PIN_0 	 // PD0
#define  D3                            GPIO_PIN_1 	 // PD1
#define  D4                            GPIO_PIN_7 	 // PE7
#define  D5                            GPIO_PIN_8 	 // PE8
#define  D6                            GPIO_PIN_9 	 // PE9
#define  D7                            GPIO_PIN_10 	 // PE10
#define  D8                            GPIO_PIN_11 	 // PE11
#define  D9                            GPIO_PIN_12 	 // PE12
#define  D10                           GPIO_PIN_13 	 // PE13
#define  D11                           GPIO_PIN_14 	 // PE14
#define  D12                           GPIO_PIN_15 	 // PE15
#define  D13                           GPIO_PIN_8 	 // PD8
#define  D14                           GPIO_PIN_9 	 // PD9
#define  D15                           GPIO_PIN_10 	 // PD10
#define  DATA_PD                       (D0 | D1 | D2 | D3 | D13 | D14 | D15)                                   
#define  DATA_PE				       (D4 | D5 | D6 | D7 | D8  | D9  | D10  | D11  | D12)

#define  EXINT_PORT                    GPIOB		 // GPIOB
#define  DM9000A_RESET_0               GPIOD->BRR = GPIO_PIN_13;
#define  DM9000A_RESET_1			   GPIOD->BSRR = GPIO_PIN_13;
#define DM9000X_BYTE_MODE              0x01
#define DM9000X_WORD_MODE              0x00

#define DM9000_IO_CMD    *((__IO rt_uint16_t *)0x60000000) // 命令 状态 C/D(A16)=0
#define DM9000_IO_DATA   *((__IO rt_uint16_t *)0x60020000) //数据 C/D(A16)= 1 0x60010000
/*
#define DM9000_IO_CMD    *((__IO rt_uint8_t *)((uint32_t)0x60000000)) // 命令 状态 C/D(A16)=0
#define DM9000_IO_DATA   *((__IO rt_uint8_t *)((uint32_t)0x600F0000)) //数据 C/D(A16)= 1 0x60010000
*/

static void dm9000_delay(rt_uint32_t delay)
{
	while (delay--);
}

void GPIO_FSMC_Configuration(void)
{ 

    RCC->AHBENR|=1<<RCC_AHBENR_FSMCEN_Pos;          //使能FSMC时钟
    RCC->APB2ENR|=1<<RCC_APB2ENR_AFIOEN_Pos;
    RCC->APB2ENR|=1<<RCC_APB2ENR_IOPBEN_Pos;        //使能PORTB时钟
    RCC->APB2ENR|=1<<RCC_APB2ENR_IOPCEN_Pos;         //使能PORTC时钟
    RCC->APB2ENR|=1<<RCC_APB2ENR_IOPDEN_Pos;         //使能PORTD时钟
    RCC->APB2ENR|=1<<RCC_APB2ENR_IOPEEN_Pos;         //使能PORTE时钟

	GPIO_InitTypeDef  GPIO_InitStructure; 
	//复位脚
	GPIO_InitStructure.Pin   = GPIO_PIN_13;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	
	
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);//D2
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);//D3
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);//NOE   //LCD_RD
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);//NWE	//LCD_WR
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource7 , GPIO_AF_FSMC);//NE1 	//LCD_CS
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);//D13
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);//D14
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);//D15
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_FSMC);	     //LCD_RS
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);//D0
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);//D1
	
	GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 |
	                        GPIO_PIN_7 |GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 |
	                        GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	
	/* GPIOE configuration */
	
//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7 , GPIO_AF_FSMC);//D4
//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8 , GPIO_AF_FSMC);//D5
//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9 , GPIO_AF_FSMC);//D6
//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10 , GPIO_AF_FSMC);//D7
//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11 , GPIO_AF_FSMC);//D8
//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource12 , GPIO_AF_FSMC);//D9
//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13 , GPIO_AF_FSMC);//D10
//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14 , GPIO_AF_FSMC);//D11
//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource15 , GPIO_AF_FSMC);//D12
	
	GPIO_InitStructure.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
	                        GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
	                        GPIO_PIN_15;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);
}  				
void DM9000A_FSMCConfig(void) 
{ 
    SRAM_HandleTypeDef hsram;
    FSMC_NORSRAM_TimingTypeDef Timing;


    hsram.Instance  = FSMC_NORSRAM_DEVICE;
    hsram.Extended  = FSMC_NORSRAM_EXTENDED_DEVICE;

    Timing. AddressSetupTime = 1;
    Timing. AddressHoldTime = 1;
    Timing. DataSetupTime = 2;
    Timing. BusTurnAroundDuration = 1;
    Timing. CLKDivision = 1;
    Timing. DataLatency = 1;
    Timing. AccessMode = FSMC_ACCESS_MODE_A;


    hsram.Init. NSBank= FSMC_NORSRAM_BANK4;
    hsram.Init. DataAddressMux= FSMC_DATA_ADDRESS_MUX_DISABLE;
    hsram.Init. MemoryType =FSMC_MEMORY_TYPE_SRAM;
    hsram.Init. MemoryDataWidth=FSMC_NORSRAM_MEM_BUS_WIDTH_16;
    hsram.Init. BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
    hsram.Init. WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
    hsram.Init. WrapMode = FSMC_WRAP_MODE_DISABLE;
    hsram.Init. WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
    hsram.Init. WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
    hsram.Init. WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
    hsram.Init. ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
    hsram.Init. AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
    hsram.Init. WriteBurst = FSMC_WRITE_BURST_DISABLE;

    if(HAL_SRAM_Init(&hsram, &Timing, &Timing) != HAL_OK)
    {
        return;//出错
    }

} 
void dm9000x_gpio_inital(void)
{
  GPIO_FSMC_Configuration( );
  DM9000A_FSMCConfig( );
}

static rt_uint16_t ior(rt_uint16_t reg)
{
    rt_uint16_t read_data = 0;

	DM9000_IO_CMD =  reg;
	read_data = DM9000_IO_DATA; 
 
    return read_data;
}

static void iow(rt_uint16_t reg, rt_uint16_t value)
{
	 DM9000_IO_CMD =  reg;
	 DM9000_IO_DATA = 	value;
}
//void dm9000x_inital(void);
rt_uint32_t dm9000x_read_id(void)
{
    rt_uint32_t id = 0x00;
//while( 1)
{	//dm9000x_inital();
	id  = ior( DM9000_VIDL ) &0x00ff;
	id |= (ior( DM9000_VIDH )&0x00ff) << 8;
	id |= (ior( DM9000_PIDL )&0x00ff) << 16;
	id |= (ior( DM9000_PIDH )&0x00ff) << 24;


//	printf("DM9000x ID:0x%x \n",id); id = 0;
//	for( id = 0; id < 1000; id++)
//	(DM9000_IO_CMD) = id;
dm9000_delay(0x1000000);
}
	if(DM9000_ID == id)
	     return  id;
	
	return 0;	 
}

void dm9000x_set_macaddr(rt_uint8_t *macaddr)
{
    rt_uint8_t i = 6;

	for(i=0; i<6; i++)
        iow(DM9000_PAR + i, macaddr[i]);

//	printf("Mac: ");
//	for(i=0; i<6; i++)
    //    printf("%x:",ior(DM9000_PAR+i));
//	printf("\n");
}

void dm9000x_inital(rt_uint8_t *macaddr)
{
	rt_uint8_t i = 0x00;

	dm9000x_gpio_inital();
	dm9000_delay (5000);
	
    /* 设置 GPCR(1EH) bit[0]=1，使DM9000的GPIO3为输出 */
	iow(DM9000_GPCR, 0x01);

    /* GPR bit[0]=0 使DM9000的GPIO3输出为低以激活内部PHY */
    iow(DM9000_GPR, 0x00);
	
    /* 延时2ms以上等待PHY上电 */
    dm9000_delay(20000);

	/* 软件复位 */
    iow(DM9000_NCR, 0x03);

	/* 延时20us以上等待软件复位完成 */
    dm9000_delay(5000);

	/* 复位完成，设置正常工作模式 */
    iow(DM9000_NCR, 0x00);

	/* 第二次软件复位，为了确保软件复位完全成功。此步骤是必要的 */
    iow(DM9000_NCR, 0x03);

    dm9000_delay(5000);
    iow(DM9000_NCR, 0x00);

    /*以上完成了DM9000的复位操作*/

	 /* 设置 GPCR(1EH) bit[0]=1，使DM9000的GPIO3为输出 */
	iow(DM9000_GPCR, 0x01);

    /* GPR bit[0]=0 使DM9000的GPIO3输出为低以激活内部PHY */
    iow(DM9000_GPR, 0x00);

    rt_uint32_t id = dm9000x_read_id();
	LOG_D("dm9000x_read_id : 0x%x",id);
    /*  */
    iow(DM9000_NSR, 0x2c);//清除各种状态标志位

    iow(DM9000_ISR, 0x3f);//清除所有中断标志位

    iow(DM9000_RCR, 0x39);//接收控制

    iow(DM9000_TCR, 0x00);//发送控制

    iow(DM9000_BPTR, 0x3f);

    iow(DM9000_FCTR, 0x3a);

    iow(DM9000_FCR, 0xff);

    iow(DM9000_SMCR, 0x00);

    for(i=0; i<6; i++)
    {
        iow(DM9000_PAR + i, macaddr[i]);
    }

    LOG_D("Mac: ");
	for(i=0; i<6; i++)
	    LOG_RAW("%x:",ior(DM9000_PAR+i) & 0x00ff);
	LOG_RAW("\n");


    iow(DM9000_NSR, 0x2c);

    iow(DM9000_ISR, 0x3f);

    iow(DM9000_IMR, 0x81);
}
void dm9000x_sendpacket( rt_uint8_t* packet, rt_uint16_t len)
{
    rt_uint16_t length = len;
	rt_uint16_t io_mode;
	rt_uint16_t i,Temp;
    iow(DM9000_IMR, 0x80);//先禁止网卡中断，防止在发送数据时被中断干扰

	io_mode = ior(DM9000_ISR) >> 7;  
	
	if(DM9000X_BYTE_MODE == (io_mode & 0x01))
		{
		while(length--)
			{
			iow(DM9000_MWCMD,*(packet++));
			}
		}
    else {
	        for(i=0;i<length;i+=2)
			{
			//	dm9000_delay(20);
			//	iow(DM9000_MWCMD,(packet[i]&0xff));
			//	iow(DM9000_MWCMD,(packet[i]>>8));
			Temp = packet[i]|( packet[i+1]<< 8);
				iow(DM9000_MWCMD,Temp);	
			}
	        LOG_D("dm9000x_sendpacket:dm9000 io mode is 16bit.....!\n");
		  }
    iow(DM9000_TXPLH, (len>>8) & 0x0ff);
    iow(DM9000_TXPLL, len & 0x0ff);
	
    iow(DM9000_TCR, 0x01);//发送数据到以太网上

	while(!(ior(DM9000_NSR)&0x0C))
	    dm9000_delay(5);    
    
    iow(DM9000_NSR, 0x00);//清除状态寄存器，由于发送数据没有设置中断，因此不必处理中断标志位

    iow(DM9000_IMR, 0x81);//DM9000网卡的接收中断使能
}

extern rt_uint8_t mac_addr[6];
rt_uint16_t dm9000x_receivepacket(rt_uint8_t* packet, rt_uint16_t maxlen)
{
    rt_uint8_t mac_addr[]={0x00,0xe0,0x3d,0xf4,0xdd,0xf7};
    rt_uint8_t ready;
	rt_uint8_t rx_io_mode;
	rt_uint32_t  rx_status = 0x00;
	rt_uint32_t  rx_length = 0x00;
	rt_uint32_t  rx_length_bak = 0x00;
	rt_uint32_t  temp=0x0000;
	rt_uint32_t  i;
    ready = 0;//希望读取到“01H”

/*以上为有效数据包前的4个状态字节*/
    if(ior(DM9000_ISR) & 0x01)
    {
        iow(DM9000_ISR, 0x01);
    }

/*清除接收中断标志位*/
 
    ready = ior(DM9000_MRCMDX); // 第一次读取，一般读取到的是 00H
    if((ready & 0x0ff) != 0x01)
        {
        ready = ior(DM9000_MRCMDX); // 第二次读取，总能获取到数据
        if((ready & 0x01) != 0x01)
        {
            if((ready & 0x01) != 0x00) //若第二次读取到的不是 01H 或 00H ，则表示没有初始化成功
            {
                 iow(DM9000_IMR, 0x80);//屏幕网卡中断
                 dm9000x_inital(mac_addr);//重新初始化
                 iow(DM9000_IMR, 0x81);//打开网卡中断
            }
            goto rx_erro;
         }
    }
 
	rx_io_mode = ior(DM9000_ISR) >> 7;

	if(DM9000X_BYTE_MODE == (rx_io_mode & 0x01))
		{
		rx_status = ior(DM9000_MRCMD) + (ior(DM9000_MRCMD) << 8);
		rx_length = ior(DM9000_MRCMD) + (ior(DM9000_MRCMD) << 8);
		
		if(!(rx_status & 0xbf00) && (rx_length < 1522))
			{
			rx_length_bak = rx_length;
			
			while(rx_length_bak--)
			    {
			    *(packet++) = ior(DM9000_MRCMD);
			    }
			}
		else
			{
		    LOG_D("bad data!\r\n");
			goto rx_erro;
			}	
		}
   else  if(DM9000X_WORD_MODE == (rx_io_mode & 0x00))
		{			  ////////////前四位 是接受包的状态字节/////////////////
			rx_status = ior(DM9000_MRCMD);
			LOG_D("DM9000X_WORD_MODE:rx_status = %d\r\n",rx_status);
			rx_length = ior(DM9000_MRCMD);
			LOG_D("DM9000X_WORD_MODE:rx_length = %d\r\n",rx_length);
	
			if(!(rx_status & 0xbf00) && (rx_length < 1522))
				{
					rx_length_bak = rx_length;
					LOG_D("Rx_length = %d\r\n",rx_length);
				   
					for(i = 0; i < rx_length; i += 2)
					{
						temp= ior(DM9000_MRCMD);
						packet[i]=temp & 0xff;
						packet[i+1]=(temp>>8) & 0xff;
					}

					LOG_D("DM9000X_WORD_MODE Packet\r\n");
				 }
			 else
			{  rx_length = 0;
			    LOG_D("bad data!\r\n");
				return 0;
			}	
			
			LOG_D("dm9000 io mode is 16bit.....!\n");
    	}


    return rx_length;
    
    rx_erro:
		return 0;
}
