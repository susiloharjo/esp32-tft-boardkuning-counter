#include <Arduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>

// TFT_eSPI instance
TFT_eSPI tft = TFT_eSPI();

// LVGL display buffer
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[320 * 20]; // Buffer for 20 lines (increased for better performance)

// Display driver
static lv_disp_drv_t disp_drv;

// Global variables
lv_obj_t *counter_label;
int counter = 0;

// GT911 Touch Controller Implementation
#define GT_CMD_WR           0XBA         //写命令0xBA
#define GT_CMD_RD           0XBB         //读命令0XBB

#define GT911_MAX_WIDTH     320          //Touchscreen pad max width
#define GT911_MAX_HEIGHT    480          //Touchscreen pad max height

//GT911 部分寄存器定义
#define GT_CTRL_REG         0X8040       //GT911控制寄存器
#define GT_CFGS_REG         0X8047       //GT911配置起始地址寄存器
#define GT_CHECK_REG        0X80FF       //GT911校验和寄存器
#define GT_PID_REG          0X8140       //GT911产品ID寄存器

#define GT_GSTID_REG        0X814E       //GT911当前检测到的触摸情况
#define GT911_READ_XY_REG   0x814E       /* 坐标寄存器 */
#define CT_MAX_TOUCH        5            //电容触摸屏最大支持的点数

int IIC_SCL = 32;
int IIC_SDA = 33;
int IIC_RST = 25;

#define IIC_SCL_0  digitalWrite(IIC_SCL,LOW)
#define IIC_SCL_1  digitalWrite(IIC_SCL,HIGH)

#define IIC_SDA_0  digitalWrite(IIC_SDA,LOW)
#define IIC_SDA_1  digitalWrite(IIC_SDA,HIGH)

#define IIC_RST_0  digitalWrite(IIC_RST,LOW)
#define IIC_RST_1  digitalWrite(IIC_RST,HIGH)

#define READ_SDA   digitalRead(IIC_SDA)

typedef struct
{
  uint8_t Touch;
  uint8_t TouchpointFlag;
  uint8_t TouchCount;

  uint8_t Touchkeytrackid[CT_MAX_TOUCH];
  uint16_t X[CT_MAX_TOUCH];
  uint16_t Y[CT_MAX_TOUCH];
  uint16_t S[CT_MAX_TOUCH];
} GT911_Dev;
GT911_Dev Dev_Now, Dev_Backup;
bool touched = 0;     //没有使用触摸中断，有触摸标志位touched = 1，否则touched = 0

uint8_t s_GT911_CfgParams[] =
{
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

void delay_us(unsigned int xus)  //1us
{
  for (; xus > 1; xus--);
}

void SDA_IN(void)
{
  pinMode(IIC_SDA, INPUT);
}

void SDA_OUT(void)
{
  pinMode(IIC_SDA, OUTPUT);
}

//初始化IIC
void IIC_Init(void)
{
  pinMode(IIC_SDA, OUTPUT);
  pinMode(IIC_SCL, OUTPUT);
  pinMode(IIC_RST, OUTPUT);
  IIC_SCL_1;
  IIC_SDA_1;
}

//产生IIC起始信号
void IIC_Start(void)
{
  SDA_OUT();
  IIC_SDA_1;
  IIC_SCL_1;
  delay_us(4);
  IIC_SDA_0; //START:when CLK is high,DATA change form high to low
  delay_us(4);
  IIC_SCL_0; //钳住I2C总线，准备发送或接收数据
}

//产生IIC停止信号
void IIC_Stop(void)
{
  SDA_OUT();
  IIC_SCL_0;
  IIC_SDA_0; //STOP:when CLK is high DATA change form low to high
  delay_us(4);
  IIC_SCL_1;
  IIC_SDA_1; //发送I2C总线结束信号
  delay_us(4);
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t IIC_Wait_Ack(void)
{
  uint8_t ucErrTime = 0;
  SDA_IN();      //SDA设置为输入
  IIC_SDA_1; delay_us(1);
  IIC_SCL_1; delay_us(1);
  while (READ_SDA)
  {
    ucErrTime++;
    if (ucErrTime > 250)
    {
      IIC_Stop();
      return 1;
    }
  }
  IIC_SCL_0; //时钟输出0
  return 0;
}

//产生ACK应答
void IIC_Ack(void)
{
  IIC_SCL_0;
  SDA_OUT();
  IIC_SDA_0;
  delay_us(2);
  IIC_SCL_1;
  delay_us(2);
  IIC_SCL_0;
}

//不产生ACK应答
void IIC_NAck(void)
{
  IIC_SCL_0;
  SDA_OUT();
  IIC_SDA_1;
  delay_us(2);
  IIC_SCL_1;
  delay_us(2);
  IIC_SCL_0;
}

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答
void IIC_Send_Byte(uint8_t txd)
{
  uint8_t t;
  SDA_OUT();
  IIC_SCL_0; //拉低时钟开始数据传输
  for (t = 0; t < 8; t++)
  {
    //IIC_SDA=(txd&0x80)>>7;
    if ((txd & 0x80) >> 7)
      IIC_SDA_1;
    else
      IIC_SDA_0;
    txd <<= 1;
    delay_us(2);   //对TEA5767这三个延时都是必须的
    IIC_SCL_1;
    delay_us(2);
    IIC_SCL_0;
    delay_us(2);
  }
}

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
uint8_t IIC_Read_Byte(unsigned char ack)
{
  unsigned char i, receive = 0;
  SDA_IN();//SDA设置为输入
  for (i = 0; i < 8; i++ )
  {
    IIC_SCL_0;
    delay_us(2);
    IIC_SCL_1;
    receive <<= 1;
    if (READ_SDA)receive++;
    delay_us(1);
  }
  if (!ack)
    IIC_NAck();//发送nACK
  else
    IIC_Ack(); //发送ACK
  return receive;
}

//reg:起始寄存器地址
//buf:数据缓缓存区
//len:写数据长度
//返回值:0,成功;1,失败.
uint8_t GT911_WR_Reg(uint16_t reg, uint8_t *buf, uint8_t len)
{
  uint8_t i;
  uint8_t ret = 0;
  IIC_Start();
  IIC_Send_Byte(GT_CMD_WR);       //发送写命令
  IIC_Wait_Ack();
  IIC_Send_Byte(reg >> 8);     //发送高8位地址
  IIC_Wait_Ack();
  IIC_Send_Byte(reg & 0XFF);     //发送低8位地址
  IIC_Wait_Ack();
  for (i = 0; i < len; i++)
  {
    IIC_Send_Byte(buf[i]);      //发数据
    ret = IIC_Wait_Ack();
    if (ret)break;
  }
  IIC_Stop();                    //产生一个停止条件
  return ret;
}

//reg:起始寄存器地址
//buf:数据缓缓存区
//len:读数据长度
void GT911_RD_Reg(uint16_t reg, uint8_t *buf, uint8_t len)
{
  uint8_t i;
  IIC_Start();
  IIC_Send_Byte(GT_CMD_WR);   //发送写命令
  IIC_Wait_Ack();
  IIC_Send_Byte(reg >> 8);     //发送高8位地址
  IIC_Wait_Ack();
  IIC_Send_Byte(reg & 0XFF);     //发送低8位地址
  IIC_Wait_Ack();
  IIC_Start();
  IIC_Send_Byte(GT_CMD_RD);   //发送读命令
  IIC_Wait_Ack();
  for (i = 0; i < len; i++)
  {
    buf[i] = IIC_Read_Byte(i == (len - 1) ? 0 : 1); //发数据
  }
  IIC_Stop();//产生一个停止条件
}

//发送配置参数
//mode:0,参数不保存到flash
//     1,参数保存到flash
uint8_t GT911_Send_Cfg(uint8_t mode)
{
  uint8_t buf[2];
  uint8_t i = 0;
  buf[0] = 0;
  buf[1] = mode;  //是否写入到GT911 FLASH?  即是否掉电保存
  GT911_WR_Reg(GT_CHECK_REG, buf, 2); //写入校验和,和配置更新标记
  return 0;
}

void GT911_Scan(void)
{
  uint8_t buf[41];
  uint8_t Clearbuf = 0;
  uint8_t i;
  static unsigned long lastDebugTime = 0;
  static unsigned long lastScanTime = 0;
  static const unsigned long SCAN_INTERVAL_MS = 20; // Limit scanning to 50Hz max
  
  // Throttle scanning to prevent excessive I2C communication
  if (millis() - lastScanTime < SCAN_INTERVAL_MS) {
    return;
  }
  lastScanTime = millis();
  
  if (1)
    // if (Dev_Now.Touch == 1)
  {
    Dev_Now.Touch = 0;
    GT911_RD_Reg(GT911_READ_XY_REG, buf, 1);

    // Debug: Print raw register value every 10 seconds (reduced frequency)
    if (millis() - lastDebugTime > 10000) {
      Serial.printf("GT911 Debug - Raw register value: 0x%02X\r\n", buf[0]);
      lastDebugTime = millis();
    }

    if ((buf[0] & 0x80) == 0x00)
    {
      touched = 0;
      GT911_WR_Reg(GT911_READ_XY_REG, (uint8_t *)&Clearbuf, 1);
      // Serial.printf("No touch\r\n");
      delay(1);
    }
    else
    {
      touched = 1;
      Dev_Now.TouchpointFlag = buf[0];
      Dev_Now.TouchCount = buf[0] & 0x0f;
      
      // Serial.printf("Touch detected! Flag: 0x%02X, Count: %d\r\n", Dev_Now.TouchpointFlag, Dev_Now.TouchCount);
      
      if (Dev_Now.TouchCount > 5)
      {
        touched = 0;
        GT911_WR_Reg(GT911_READ_XY_REG, (uint8_t *)&Clearbuf, 1);
        Serial.printf("Dev_Now.TouchCount > 5\r\n");
        return ;
      }
      GT911_RD_Reg(GT911_READ_XY_REG + 1, &buf[1], Dev_Now.TouchCount * 8);
      GT911_WR_Reg(GT911_READ_XY_REG, (uint8_t *)&Clearbuf, 1);

      Dev_Now.Touchkeytrackid[0] = buf[1];
      Dev_Now.X[0] = ((uint16_t)buf[3] << 8) + buf[2];
      Dev_Now.Y[0] = ((uint16_t)buf[5] << 8) + buf[4];
      Dev_Now.S[0] = ((uint16_t)buf[7] << 8) + buf[6];

      Dev_Now.Touchkeytrackid[1] = buf[9];
      Dev_Now.X[1] = ((uint16_t)buf[11] << 8) + buf[10];
      Dev_Now.Y[1] = ((uint16_t)buf[13] << 8) + buf[12];
      Dev_Now.S[1] = ((uint16_t)buf[15] << 8) + buf[14];

      Dev_Now.Touchkeytrackid[2] = buf[17];
      Dev_Now.X[2] = ((uint16_t)buf[19] << 8) + buf[18];
      Dev_Now.Y[2] = ((uint16_t)buf[21] << 8) + buf[20];
      Dev_Now.S[2] = ((uint16_t)buf[23] << 8) + buf[22];

      Dev_Now.Touchkeytrackid[3] = buf[25];
      Dev_Now.X[3] = ((uint16_t)buf[27] << 8) + buf[26];
      Dev_Now.Y[3] = ((uint16_t)buf[29] << 8) + buf[28];
      Dev_Now.S[3] = ((uint16_t)buf[31] << 8) + buf[30];

      Dev_Now.Touchkeytrackid[4] = buf[33];
      Dev_Now.X[4] = ((uint16_t)buf[35] << 8) + buf[34];
      Dev_Now.Y[4] = ((uint16_t)buf[37] << 8) + buf[36];
      Dev_Now.S[4] = ((uint16_t)buf[39] << 8) + buf[38];

      // Serial.printf("Touch coordinates: X=%d, Y=%d, Size=%d\r\n", Dev_Now.X[0], Dev_Now.Y[0], Dev_Now.S[0]);

      for (i = 0; i < Dev_Backup.TouchCount; i++)
      {
        if (Dev_Now.Y[i] < 0)Dev_Now.Y[i] = 0;
        if (Dev_Now.Y[i] > 480)Dev_Now.Y[i] = 480;
        if (Dev_Now.X[i] < 0)Dev_Now.X[i] = 0;
        if (Dev_Now.X[i] > 320)Dev_Now.X[i] = 320;
      }
      // Validate and process touch points
      bool validTouch = false;
      for (i = 0; i < Dev_Now.TouchCount; i++)
    {
        // Validate coordinates more strictly
        if (Dev_Now.Y[i] < 0 || Dev_Now.Y[i] > 480 || Dev_Now.X[i] < 0 || Dev_Now.X[i] > 320) {
            // Serial.printf("Invalid coordinates detected: X=%d, Y=%d - rejecting\r\n", Dev_Now.X[i], Dev_Now.Y[i]);
            continue;
        }
        
        // Additional validation for obviously wrong values
        if (Dev_Now.X[i] > 1000 || Dev_Now.Y[i] > 1000) {
            // Serial.printf("Corrupted coordinates detected: X=%d, Y=%d - rejecting\r\n", Dev_Now.X[i], Dev_Now.Y[i]);
            continue;
        }

        // Valid touch found
        Dev_Backup.X[i] = Dev_Now.X[i];
        Dev_Backup.Y[i] = Dev_Now.Y[i];
        Dev_Backup.TouchCount = Dev_Now.TouchCount;
        // Serial.printf("Touch validated: X=%d, Y=%d\r\n", Dev_Now.X[i], Dev_Now.Y[i]);
        validTouch = true;
      }
      
      // Set touched based on whether we have valid touches
      touched = validTouch;
     if(Dev_Now.TouchCount==0)
        {
            touched = 0;
        }  
    }
  }
}

uint8_t GT911_ReadStatue(void)
{
  uint8_t buf[4];
  GT911_RD_Reg(GT_PID_REG, (uint8_t *)&buf[0], 3); 
  GT911_RD_Reg(GT_CFGS_REG, (uint8_t *)&buf[3], 1);
  Serial.printf("TouchPad_ID:%d,%d,%d\r\nTouchPad_Config_Version:%2x\r\n", buf[0], buf[1], buf[2], buf[3]);
  return buf[3];
}

void GT911_Reset_Sequence()
{
  //此处RST引脚与屏幕RST共用，只需要初始化一次即可
  IIC_RST_0;
  delay(100);
  IIC_RST_0;
  delay(100);
  IIC_RST_1;
  delay(200);
}

void GT911_Int()
{
  uint8_t config_Checksum = 0, i;

  IIC_Init();
  GT911_Reset_Sequence();
  //debug
  GT911_RD_Reg(GT_CFGS_REG, (uint8_t *)&s_GT911_CfgParams[0], 186);

  for (i = 0; i < sizeof(s_GT911_CfgParams) - 2; i++)
  {
    config_Checksum += s_GT911_CfgParams[i];
  }

  if (s_GT911_CfgParams[184] == (((~config_Checksum) + 1) & 0xff))
  {
    Serial.printf("READ CONFIG SUCCESS!\r\n");
    Serial.printf("%d*%d\r\n", s_GT911_CfgParams[2] << 8 | s_GT911_CfgParams[1], s_GT911_CfgParams[4] << 8 | s_GT911_CfgParams[3]);

    if ((GT911_MAX_WIDTH != (s_GT911_CfgParams[2] << 8 | s_GT911_CfgParams[1])) || (GT911_MAX_HEIGHT != (s_GT911_CfgParams[4] << 8 | s_GT911_CfgParams[3])))
    {
      s_GT911_CfgParams[1] = GT911_MAX_WIDTH & 0xff;
      s_GT911_CfgParams[2] = GT911_MAX_WIDTH >> 8;
      s_GT911_CfgParams[3] = GT911_MAX_HEIGHT & 0xff;
      s_GT911_CfgParams[4] = GT911_MAX_HEIGHT >> 8;
      s_GT911_CfgParams[185] = 1;

      config_Checksum = 0;
      for (i = 0; i < sizeof(s_GT911_CfgParams) - 2; i++)
      {
        config_Checksum += s_GT911_CfgParams[i];
      }
      s_GT911_CfgParams[184] = (~config_Checksum) + 1;

      GT911_WR_Reg(GT_CFGS_REG, (uint8_t *)s_GT911_CfgParams, sizeof(s_GT911_CfgParams));
    }
  }
  GT911_ReadStatue();
}

// Display flush callback
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t*)&color_p->full, w * h, true);
    tft.endWrite();

    lv_disp_flush_ready(disp);
}

// Touchpad read callback
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
    static unsigned long lastTouchDebug = 0;
    static bool lastTouched = false;
    static unsigned long lastCallbackDebug = 0;
    static unsigned long lastTouchTime = 0;
    static const unsigned long TOUCH_DEBOUNCE_MS = 150; // Debounce time to prevent double clicks

    // Debug: Print when callback is called every 10 seconds (reduced frequency)
    if (millis() - lastCallbackDebug > 10000) {
        Serial.printf("my_touchpad_read callback called, touched=%d\r\n", touched);
        lastCallbackDebug = millis();
    }

    // Throttle touch scanning to prevent double clicks
    if (millis() - lastTouchTime < 50) { // 20 FPS max for touch scanning
        data->state = lastTouched ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
        return;
    }
    lastTouchTime = millis();

    GT911_Scan();
    
    if (!touched) {
        data->state = LV_INDEV_STATE_REL;
        if (lastTouched) {
            Serial.printf("Touch released\r\n");
            lastTouched = false;
        }
     } else {
         // Debounce touch to prevent double clicks
         static unsigned long lastValidTouch = 0;
         if (millis() - lastValidTouch < TOUCH_DEBOUNCE_MS) {
             data->state = LV_INDEV_STATE_REL;
             return;
         }
         lastValidTouch = millis();

         /*Set the coordinates - adjust based on actual touch coordinates*/
         // Based on user feedback: touch area is still above button, need to shift down more
         // Touch area should be on the button itself, not above it
         data->point.x = Dev_Now.X[0] + 110;  // Keep X offset
         data->point.y = Dev_Now.Y[0] + 80;   // Increase Y offset to shift touch down more to button
         data->state = LV_INDEV_STATE_PR;
        
         // Print LVGL touch data only when starting touch (reduced frequency)
         if (!lastTouched) {
             Serial.printf("LVGL Touch: X=%d, Y=%d, State=%d\r\n", data->point.x, data->point.y, data->state);
             // Check if touch is in button area (adjusted with larger Y offset)
             // With Y offset +80, button area shifts down more
             // Button area should be around X=85-235, Y=227-307
             if (data->point.x >= 85 && data->point.x <= 235 && data->point.y >= 227 && data->point.y <= 307) {
                 Serial.printf("Touch is in button area! Will trigger button click.\r\n");
             } else {
                 Serial.printf("Touch is OUTSIDE button area. Button area: X=85-235, Y=227-307\r\n");
                 Serial.printf("Raw touch: X=%d, Y=%d -> Adjusted: X=%d, Y=%d\r\n", Dev_Now.X[0], Dev_Now.Y[0], data->point.x, data->point.y);
             }
             Serial.printf("Touch started\r\n");
             lastTouched = true;
         }
    }
}

// Button click event handler
static void btn_event_cb(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    static unsigned long lastClickTime = 0;
    static const unsigned long CLICK_DEBOUNCE_MS = 200; // Prevent double clicks
    
    Serial.printf("Button event received: %d\r\n", code);
    
    // Only respond to CLICKED event and add debouncing
    if (code == LV_EVENT_CLICKED) {  // LV_EVENT_CLICKED = 1
        // Debounce to prevent double clicks
        if (millis() - lastClickTime < CLICK_DEBOUNCE_MS) {
            Serial.printf("Click ignored - too fast (debounce)\r\n");
            return;
        }
        lastClickTime = millis();
        
        counter++;
        lv_label_set_text_fmt(counter_label, "%d", counter);
        Serial.printf("Button clicked! Counter: %d\r\n", counter);
    } else {
        Serial.printf("Button event code: %d (ignored - only CLICKED events processed)\r\n", code);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 TFT Touch Test Starting...");

    // Initialize GT911 touch controller
    Serial.println("Initializing GT911 touch controller...");
    GT911_Int();
    Serial.println("GT911 initialization completed.");
    
    // Initialize TFT display
    tft.init();
    tft.setRotation(2);  // 180° rotation to match touch orientation
    tft.fillScreen(TFT_BLACK);

    // Initialize LVGL
    lv_init();

    // Initialize display buffer
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, 320 * 20);

    // Initialize display
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = 320;  // Rotated display: width becomes height
    disp_drv.ver_res = 480;  // Rotated display: height becomes width
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    // Initialize input device
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    // Create main screen
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x003a57), LV_PART_MAIN);

    // Create title label
    lv_obj_t *title_label = lv_label_create(scr);
    lv_label_set_text(title_label, "ESP32 TFT Test");
    lv_obj_set_style_text_color(title_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(title_label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 20);

    // Create counter label
    counter_label = lv_label_create(scr);
    lv_label_set_text_fmt(counter_label, "%d", counter);
    lv_obj_set_style_text_color(counter_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(counter_label, &lv_font_montserrat_48, LV_PART_MAIN);
    lv_obj_align(counter_label, LV_ALIGN_CENTER, 0, -80);

     // Create button
     lv_obj_t *btn = lv_btn_create(scr);
     lv_obj_set_size(btn, 150, 80);  // Made bigger for easier touch
     lv_obj_align(btn, LV_ALIGN_CENTER, 0, 40);
     lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_ALL, NULL);
    
    // Make button more visible with bright color
    lv_obj_set_style_bg_color(btn, lv_color_hex(0xFF0000), LV_PART_MAIN);
    
    // Debug: Print button position
    lv_area_t btn_area;
    lv_obj_get_coords(btn, &btn_area);
    Serial.printf("Button created at: X=%d, Y=%d, Width=%d, Height=%d\r\n", 
                  btn_area.x1, btn_area.y1, btn_area.x2 - btn_area.x1, btn_area.y2 - btn_area.y1);
    Serial.printf("Button area: X=%d-%d, Y=%d-%d\r\n", 
                  btn_area.x1, btn_area.x2, btn_area.y1, btn_area.y2);

    // Create button label
    lv_obj_t *btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Click Me!");
    lv_obj_set_style_text_color(btn_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_center(btn_label);

    // Create instructions label
    lv_obj_t *instr_label = lv_label_create(scr);
    lv_label_set_text(instr_label, "Touch the button to increment counter");
    lv_obj_set_style_text_color(instr_label, lv_color_hex(0x888888), LV_PART_MAIN);
    lv_obj_set_style_text_font(instr_label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(instr_label, LV_ALIGN_BOTTOM_MID, 0, -20);

    Serial.println("Setup completed successfully!");
}

void loop() {
    GT911_Scan();  // Scan for touch input
    lv_timer_handler();  // Handle LVGL tasks
    
    // Counter only increments on button click - no auto-increment
    
    delay(1);  // Minimal delay for better responsiveness
}