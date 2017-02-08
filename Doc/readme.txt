本软件使用说明：

1，设备初始化需要进行SensorInit.h的修改，修改内容可参考AiderProtocol

#define DEVICE   BIRMM_RTU100                            //设备类型为燃气智能监测终端
#define REPORTDATA  GAS                                  //上报数据为气体浓度
#define SENSORID {0x31,0x20,0x16,0x08,0x31,0x02}        //传感器ID号
#define SOFTVERSION 0x20                                //软件版本号
#define HARDVERSION 0x10                                //硬件版本号


2, 上报数据时，需要修改AiderProtocol.c内的SendDataToServ()函数

3，本版本的存储结构如下：


  
