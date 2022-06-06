#include "app/framework/include/af.h"
#include "function_control.h"

struct serial_command recv_buf;
struct report_command  report_mcu;
void data_initialize(void)
{
  recv_buf.data_length = 0;                 //结构体初始化后续可以优化为宏定义
  recv_buf.temp_length = 0;
}


void Serial_Recv_Data(void)
{
  //emberAfCorePrintln("11111111111111");
  uint8_t bytes = 0;
  bytes = emberSerialReadAvailable(COM_USART1);
  unsigned char *str;
  if(bytes) {
      emberSerialReadData(COM_USART1,
                          &recv_buf.cmd[recv_buf.data_length],
                          bytes,
                          NULL);
      recv_buf.data_length += bytes;
#if 1
      if(recv_buf.data_length > 10) {
          uint8_t m;
          for(m=0;m<recv_buf.data_length;m++) {
              emberAfCorePrintln("%d:%x",m,recv_buf.cmd[m]);
          }
      }
#endif
      if(recv_buf.data_length > 2) {
          str = &recv_buf.cmd[recv_buf.temp_length];
          while(recv_buf.temp_length < recv_buf.data_length){       //处理过的长度 < 接收的数据
              if(str[0] == HEAD_CODE){                              //收到正确帧头
                  if((recv_buf.temp_length+(str[1]+2)) <= recv_buf.data_length )
                  {                                                 //收到的剩余数据最少为一帧
                      //下面处理数据
                      static int ret = 0;
                      if(MultiIRCountSumCheckCommand(str) != str[str[1]+1]){
                          str++;                                    //中间帧错误继续往后处理。
                          recv_buf.temp_length++;                   //校验和错误
                      }else{
                         ret = MultiIRDataReportProcessFunction(&report_mcu,&recv_buf,&str);
                         if(ret){                                   //处理完毕跳出循环
                             break;
                         }
                      }

                  }else{                                            //剩余数据不足一帧，则退出处理继续接收
                      break;
                  }
              } else {
                  str++;                                            //串口接收的第一帧不为0XFA 往后走
                  recv_buf.temp_length++;
              } //END  recv error;
          }

      } //END recv_buf.data_length

  } //END  SerialRead

}

/***************************************************************
 * @user MultiIR computes checksum commands.
 * ptr[1]:后续数据长度。ptr[1]+1:需要计算校验和的长度。
 *
 * return:Check SUM。
 *
 ***************************************************************/
int MultiIRCountSumCheckCommand(unsigned char *ptr)
{
  uint8_t i = 0;
  int SUM = 0;
  for(i=0;i<ptr[1]+1;i++)
  {
      SUM += ptr[i];
      //emberAfCorePrintln("ptr[%d] %x",i,ptr[i] );
  }
  SUM = SUM%256;
  emberAfCorePrintln("check %x",SUM );
  return SUM;
}


uint8_t MultiIRDataReportProcessFunction(serial_report_command *report_process,
                                         serial_recv_command *recv_buf,
                                         unsigned char **ptr)
{
  int offset = 0;
  offset = (*ptr)[1] +2;                                          //偏移数据的总长度  一帧数据长度
  memcpy(&report_process->cmd[0],*ptr,offset);                    //上报时用report_mcu.cmd
  report_process->data_length = offset;                           //函数内这两个数的作用相同
  recv_buf->temp_length += offset;                                //记录现在处理到的长度
  /**这个地方要用事件设置20MS的低电平**/                             //先发ACK再上报到网关。
  switch ((*ptr)[2])
  {
  case DATA_REPORT_TYPE:                                          //数据上报类型
    msgflag=DATA_REPORT_TYPE;
    emberEventControlSetDelayMS(StartLowLevelEventControl,0);     //message to mcu. 只要有数据来就要ACK
    ReportMessageToGateway(report_process);                       //message to gateway 上报网关要区分是主动下发，还是主动上报。
    break;
  case NETWORK_TYPE:                                              //使能入网事件  入网状态再次入网为离网然后入网
    msgflag=NETWORK_TYPE;
    emberEventControlSetDelayMS(StartLowLevelEventControl,0);     //message to mcu. 只要有数据来就要ACK
    VerifyDeviceFunction(report_process);
    NetworkSteeringOperation(report_process);
    break;
  case LED_TYPE:                                                  //带串口的mcu 灯是有mcu控制。
    /* code LED type 只能是回复的消息，mcu不会直接下发LED_TYPE的数据。 */
    msgflag=LED_TYPE;
    //emberEventControlSetDelayMS(StartLowLevelEventControl,0);   //主动下发命令收到的ACK , 不需要在下发
    break;
  case ALARM_TYPE:
    /* code */
    break;
  default:
    break;
  }

  

  emberAfCorePrintln("-log1-");
  emberAfCorePrintln("temp_length %x",recv_buf->temp_length);

  if (recv_buf->temp_length == recv_buf->data_length) {           //数据处理完毕
      //memset(report_process->cmd,0,sizeof(report_process->cmd));
      memset(recv_buf->cmd,0,sizeof(recv_buf->cmd));
      recv_buf->data_length = 0;
      recv_buf->temp_length = 0;

      return 1;
  }
  *ptr += offset;
  memset(report_process->cmd,0,sizeof(report_process->cmd));

  return 0;
}

void ReportMessgeToMCU(serial_report_command *report_mcu)
{
  int8u i;
  int SUM = 0;
  halInternalResetWatchDog();
  report_mcu->cmd[0] = MODULE_CMD_HEADER;                                       //改变帧头，其他数据不动
  for(i = 0;i < report_mcu->data_length-1;i++)
  {
      SUM += report_mcu->cmd[i];
  }
  report_mcu->cmd[report_mcu->data_length-1]   = SUM%256;                       //重新计算校验和
  WakeUpMessageToMCU();                                                         //发送叫醒数据
  emberSerialWriteData(COM_USART1, report_mcu->cmd, report_mcu->data_length);   //数据发送
}

void ReportMessageToGateway(serial_report_command *report_message)
{
  { 
    uint8_t data[2] = {0,0};
    
    if (report_message->cmd[9]&0x20)                                            //防拆报警
    {
     //对应上报状态对应位置1                                                     
      emberAfReadServerAttribute(1,                                             //读属性为取之前状态
                              ZCL_IAS_ZONE_CLUSTER_ID,
                              ZCL_ZONE_STATUS_ATTRIBUTE_ID,
                              (uint8_t*)data,
                              8);    
      data[0] |= 0x04;                                                          //ZoneStatus第3位置1 报警
      ZoneStatus = (((data[0]) & 0x00ff) +(data[1]<<8 & 0xff00)) ;      
      emberAfCorePrintln("data[0]=%x data[1] %x.",data[0],data[1]);
      
      emberAfWriteServerAttribute(1,                                            //此属性不勾选reporting.不会report，报警信息。
                                  ZCL_IAS_ZONE_CLUSTER_ID,
                                  ZCL_ZONE_STATUS_ATTRIBUTE_ID,
                                  (uint8_t*)data,
                                  ZCL_BITMAP16_ATTRIBUTE_TYPE); 
      
    }else{                                                                      //取消报警
      //对应上报状态对应位置0
      emberAfReadServerAttribute(1,                                             //读属性为取之前状态
                              ZCL_IAS_ZONE_CLUSTER_ID,
                              ZCL_ZONE_STATUS_ATTRIBUTE_ID,
                              (uint8_t*)data,
                              8);    
      data[0] &= 0xFB;                                                          //ZoneStatus第3位置0 取消报警
      ZoneStatus = (((data[0]) & 0x00ff) +(data[1]<<8 & 0xff00));      
      emberAfWriteServerAttribute(1,
                                  ZCL_IAS_ZONE_CLUSTER_ID,
                                  ZCL_ZONE_STATUS_ATTRIBUTE_ID,
                                  (uint8_t*)data,
                                  ZCL_BITMAP16_ATTRIBUTE_TYPE);
      
    }

    if (report_message->cmd[9]&0x10)                                            //阈值报警
    {
      //对应上报状态对应位置1                                                     
      emberAfReadServerAttribute(1,                                             //读属性为取之前状态
                                ZCL_IAS_ZONE_CLUSTER_ID,
                                ZCL_ZONE_STATUS_ATTRIBUTE_ID,
                                (uint8_t*)data,
                                8);
      data[0] |= 0x01;                                                          //ZoneStatus最低位置1 报警
      ZoneStatus = (((data[0]) & 0x00ff) +(data[1]<<8 & 0xff00));      
      emberAfCorePrintln("data[0]=%x data[1] %x.",data[0],data[1]);
      emberAfCorePrintln("cmd[9]=0x10 ZoneStatus is %2x.",ZoneStatus);
      emberAfWriteServerAttribute(1,
                                  ZCL_IAS_ZONE_CLUSTER_ID,
                                  ZCL_ZONE_STATUS_ATTRIBUTE_ID,
                                  (uint8_t*)data,
                                  ZCL_BITMAP16_ATTRIBUTE_TYPE);
    }else{                                                                      //取消报警
      //对应上报状态对应位置0
      emberAfReadServerAttribute(1,                                             //读属性为取之前状态
                                ZCL_IAS_ZONE_CLUSTER_ID,
                                ZCL_ZONE_STATUS_ATTRIBUTE_ID,
                                (uint8_t*)data,
                                8);
      data[0] &= 0xFE;                                                          //ZoneStatus最低位置0 取消报警
      ZoneStatus = (((data[0]) & 0x00ff) +(data[1]<<8 & 0xff00)); 
      emberAfWriteServerAttribute(1,
                                  ZCL_IAS_ZONE_CLUSTER_ID,
                                  ZCL_ZONE_STATUS_ATTRIBUTE_ID,
                                  (uint8_t*)data,
                                  ZCL_BITMAP16_ATTRIBUTE_TYPE);     
    }

    if (report_message->cmd[9]&0x40)                                            //低压报警
    {
      //对应上报状态对应位置1                                                     
      emberAfReadServerAttribute(1,                                             //读属性为取之前状态
                              ZCL_IAS_ZONE_CLUSTER_ID,
                              ZCL_ZONE_STATUS_ATTRIBUTE_ID,
                              (uint8_t*)data,
                              8);
      data[0] |= 0x08;                                                          //ZoneStatus第四位置1 低压报警
      ZoneStatus = (((data[0]) & 0x00ff) +(data[1]<<8 & 0xff00));
      emberAfWriteServerAttribute(1,
                                  ZCL_IAS_ZONE_CLUSTER_ID,
                                  ZCL_ZONE_STATUS_ATTRIBUTE_ID,
                                  (uint8_t*)data,
                                  ZCL_BITMAP16_ATTRIBUTE_TYPE);     
    }else{                                                                      //取消报警
      //对应上报状态对应位置0
      emberAfReadServerAttribute(1,                                             //读属性为取之前状态
                              ZCL_IAS_ZONE_CLUSTER_ID,
                              ZCL_ZONE_STATUS_ATTRIBUTE_ID,
                              (uint8_t*)data,
                              8);
      data[0] &= 0xF7;                                                          //ZoneStatus第四位置0 取消报警
      ZoneStatus = (((data[0]) & 0x00ff) +(data[1]<<8 & 0xff00));    
      emberAfWriteServerAttribute(1,
                                  ZCL_IAS_ZONE_CLUSTER_ID,
                                  ZCL_ZONE_STATUS_ATTRIBUTE_ID,
                                  (uint8_t*)data,
                                  ZCL_BITMAP16_ATTRIBUTE_TYPE);  
    }
    emberEventControlSetDelayMS(SentZoneStatusEventControl,0);                  //开始发送命令EVENT
    //麦乐克的muc协议不存在上报电压的百分比
    if (report_message->cmd[9]&0x04)                                            //有电压信息
    {
      uint8_t voltage;
      voltage = report_message->cmd[10];                                        //report_message->cmd[10] 数据2
      emberAfWriteServerAttribute(1,
                              ZCL_POWER_CONFIG_CLUSTER_ID,
                              ZCL_BATTERY_PERCENTAGE_REMAINING_ATTRIBUTE_ID,
                              (uint8_t*)&voltage,
                              ZCL_DATA8_ATTRIBUTE_TYPE);
    } else {                                                                    //无电压信息
      /*什么都不做*/
    }

  }
}

void InitiativeSendLedToMCU(void)
{
  uint8_t buffer[12]={0xF5,0x0A,0x03,0x70,0x00,0x00,0x03,0x01,0x51,0xAA,0x00,0x71};//配网灯闪烁 0x51代表烟感，气感0x41
  uint8_t buffer1[12]={0xF5,0x0A,0x03,0x80,0x00,0x00,0x03,0x01,0x51,0xFF,0x00,0xD6};//绿灯长亮 5S
  uint8_t buffer2[12]={0xF5,0x0A,0x03,0x90,0x00,0x00,0x03,0x01,0x51,0x00,0x00,0xE7};//黄灯长亮 5S

  WakeUpMessageToMCU();                                                         //发送叫醒数据

  if(SEND_LED_TYPE==ledfalg){
      emberSerialWriteData(COM_USART1, buffer, sizeof(buffer));   //数据发送

  }else if(SEND_LED_SUCCESS==ledfalg){
      emberSerialWriteData(COM_USART1, buffer1, sizeof(buffer1));   //数据发送

  }else if(LED_FAILED_OR_CLOSE==ledfalg){
      emberSerialWriteData(COM_USART1, buffer2, sizeof(buffer2));   //数据发送
      emberAfCorePrintln("buffer2[%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x]",
                               buffer2[0],buffer2[1],buffer2[2],buffer2[3],
                               buffer2[4],buffer2[5],buffer2[6],buffer2[7],
                               buffer2[8],buffer2[9],buffer2[10],buffer2[11]);
  }

}



void WakeUpMessageToMCU(void)
{
  static uint8_t buffer[2] = {0xAA,0xAA};

  emberSerialWriteData(COM_USART1, buffer, sizeof(buffer));   //数据发送
}
//电平置低事件
void StartLowLevelEventHandler(void)
{
  emberEventControlSetInactive(StartLowLevelEventControl);
  GPIO_PinOutClear(GPIO_OUT_PORT, GPIO_OUT_PIN);
  emberEventControlSetDelayMS(StopLowLevelEventControl,20);                     //20ms后IO置高
}

void StopLowLevelEventHandler(void)
{
  emberEventControlSetInactive(StopLowLevelEventControl);
  GPIO_PinOutSet(GPIO_OUT_PORT, GPIO_OUT_PIN);
  emberEventControlSetDelayMS(ReportMessgeToMCUEventControl,0);                 //IO置高后开始发送ACK

}
//发送消息到MCU
void ReportMessgeToMCUEventHandler(void)
{
  emberEventControlSetInactive(ReportMessgeToMCUEventControl);
  //回应ACK
  if(DATA_REPORT_TYPE==msgflag || NETWORK_TYPE==msgflag){
    ReportMessgeToMCU(&report_mcu);  //mcu 发送过来数据给MCU的ACK
    msgflag = 0;
  }else if(LED_TYPE==msgflag){       //收到ACK
      resendtimes = 0;
      msgflag = 0;
      emberEventControlSetInactive(StartLowLevelEventControl);//停止发送
      //判断ledflag的情况  做出相应的状态在清零
      if(ledfalg==SEND_LED_SUCCESS){
          emberEventControlSetDelayMS(afterFiveSecondsCloseLedEventControl,4000);
          ledfalg = 0;
      }else {
         ledfalg = 0;
      }

  }
  emberAfCorePrintln("1   ledfalg=%d",ledfalg);
  //主动发送
  if (SEND_LED_TYPE==ledfalg || SEND_LED_SUCCESS==ledfalg || LED_FAILED_OR_CLOSE==ledfalg){
    InitiativeSendLedToMCU();        //主动下发控制LED的数据包。 
    resendtimes++;
    if(resendtimes<20){
      emberEventControlSetDelayMS(StartLowLevelEventControl,100);
    }else{                           //大于20次停止发送
      resendtimes = 0;
      ledfalg = 0;
    }
    
  }
  

}

void afterFiveSecondsCloseLedEventHandler(void)
{
  emberEventControlSetInactive(afterFiveSecondsCloseLedEventControl);
  ledfalg=LED_FAILED_OR_CLOSE;
  emberEventControlSetDelayMS(StartLowLevelEventControl,0);
}



void SentZoneStatusEventHandler(void)
{
  emberEventControlSetInactive(SentZoneStatusEventControl);
  emberAfFillCommandIasZoneClusterZoneStatusChangeNotification(
                                                                ZoneStatus,
                                                                0, // extended status, must be zero per spec
                                                                emberAfPluginIasZoneServerGetZoneId(1),
                                                                0); // called "delay" in the spec
  emberAfSetCommandEndpoints(1, 1);
  emberAfSendCommandUnicastToBindings();

}

void NetworkSteeringOperation(serial_report_command *report_message)
{
  //if (report_message->cmd[8]==MotionSensor)
  {

    if (report_message->cmd[9]==0x02)                                           //按键入网                                   
    {
      if(emberAfNetworkState() == EMBER_JOINED_NETWORK) {                       //在网状态
          emberLeaveNetwork(); //离开网络
      } 

      emberEventControlSetActive(emberAfKeepConnectEventControl);//开启入网事件
      //燃气，烟雾需要发送命令控制led.
      
      emberEventControlSetActive(emberAfLedJoinNetworkStatusEventControl);//入网灯闪烁事件
      
 
    }else{                                                                     

    }

  }

}

void emberAfKeepConnectEventControlHandler(void)
{
  
  emberEventControlSetInactive(emberAfKeepConnectEventControl);
  if (networkJoinAttempts < REJOIN_ATTEMPTS)                   //重复入网3次
  {
      networkJoinAttempts++;
      emberAfPluginNetworkSteeringStart();
      emberEventControlSetDelayQS(emberAfKeepConnectEventControl,QS_BETWEEN_JOIN_ATTEMPTS);
  } else {                                      //大于3次入网失败，指示灯状态。
      networkJoinAttempts =0;
      ledfalg=LED_FAILED_OR_CLOSE;                                             //
      emberEventControlSetDelayMS(StartLowLevelEventControl,0);            //发送失败消息到MCU
      emberEventControlSetInactive(emberAfLedJoinNetworkStatusEventControl);
  }

}

void emberAfLedJoinNetworkStatusEventHandler(void)
{
  emberEventControlSetInactive(emberAfLedJoinNetworkStatusEventControl);
  ledfalg=SEND_LED_TYPE;

  emberEventControlSetDelayMS(StartLowLevelEventControl,0);

}





/**
*@user 通过入网信息去判断zonetype 如果不同就写新的zonetype。
*      如果以后要写pid 也再此函数里面实现。
*      由于要修改插件里面的zone type，不能只修改zonetype attribute，所以这个地方作用暂时不大。不知道pid什么情况。
**/
void VerifyDeviceFunction(serial_report_command *process_message)
{
  static uint8_t type[2] = {0,0};
  static uint16_t zonetype = 0x0000;
  static uint8_t value =0x00;
  emberAfReadServerAttribute(1,
                            ZCL_IAS_ZONE_CLUSTER_ID,
                            ZCL_ZONE_TYPE_ATTRIBUTE_ID,
                            (uint8_t*)type,
                            8);
  zonetype = ((type[0] & 0x00ff) +(type[1]<<8 & 0xff00));  
  switch (zonetype)
  {
  case 0x000d:                                                           //值的意思参见zcl文档
    value = MotionSensor;
    break;
  case 0x0028:
    value = FireSensor;
    break;
  case 0x002b:
    value = CarbonMonoxideSensor;
    break;    
  default:
    break;
  }

  if (value == process_message->cmd[8])                                   //设备类型与zonetype相等
  {
    /* nothing to do. */
  } else {
    if (process_message->cmd[8] == MotionSensor)
    {
      zonetype = 0x000d;
    } else if (process_message->cmd[8] == CarbonMonoxideSensor)
    {
      zonetype = 0x0028;
    } else if (process_message->cmd[8] == FireSensor)
    {
      zonetype = 0x002b;
    }
                                                                          //写设备类型
    type[0] = (zonetype & 0x00ff);
    type[1] = (zonetype & 0xff00)>>8;                                                                     
    emberAfWriteServerAttribute(1,
                                ZCL_IAS_ZONE_CLUSTER_ID,
                                ZCL_ZONE_TYPE_ATTRIBUTE_ID,
                                (uint8_t*)type,
                                ZCL_BITMAP16_ATTRIBUTE_TYPE);
  }
  

}




