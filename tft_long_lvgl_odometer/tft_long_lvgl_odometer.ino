/*
  For Lilygo T-Display-S3-Long 
  nikthefix - 19th Dec 2023
  Modified driver code and example sketch using LVGL in 'sprite only mode' 

  Versions:
  LVGL 8.3.11 - latest at time of writing
  ESP32 Arduino 3.0.0-alpha3 - latest at time of writing
  Arduino IDE 2.2.1

  Notes:
  As the display doesn't implement a scan orientation hardware rotate - as far as I can see from the current datasheet - we need to use 
  a soft matrix rotation to get a landscape view without messing with LVGL. This is implemented in lcd_PushColors_rotated_90().
  You can then use lcd_setRotation(2) which IS hardware implemented, to flip the whole thing upside down if you need. If so then make
  sure you also flip the touch coordinates.
  Code has been stripped down to support QSPI display only.
  Use a fresh install of LVGL and NOT the lilygo supplied lib.

  Build Options:
  Board  ESP32-S3-Dev
  USB CDC On boot Enabled
  Flash Size 16MB
  Partition Scheme 16M Flash(3MB APP/9.9MB FATFS)
  PSRAM "OPI PSRAM"

  In lv_conf.h:
  Line 15 - #if 1 (Set it to "1" to enable content)
  Line 30 - #define LV_COLOR_16_SWAP 1
  Line 49 - #define LV_MEM_CUSTOM 1
  Line 88 - #define LV_TICK_CUSTOM 1
  Lines 354 to 384 - enable fonts

  Since ESP32 Arduino 3.0.0-alpha3 is still pretty funky with a lot of existing Arduino driver code it may be necessary to downgrade to V2.xx as the project expands - in the short term
*/

#include "AXS15231B.h"
#include <Arduino.h>
#include <lvgl.h>
#include "ui.h"
#include <Wire.h>



///////////// touch settings //////////////////////////
SemaphoreHandle_t xSemaphore = NULL;
uint8_t ALS_ADDRESS = 0x3B;
uint8_t read_touchpad_cmd[11] = {0xb5, 0xab, 0xa5, 0x5a, 0x0, 0x0, 0x0, 0x8};
#define TOUCH_IICSCL 10
#define TOUCH_IICSDA 15
#define TOUCH_INT 11
#define TOUCH_RES 16
#define AXS_TOUCH_ONE_POINT_LEN             6
#define AXS_TOUCH_BUF_HEAD_LEN              2
#define AXS_TOUCH_GESTURE_POS               0
#define AXS_TOUCH_POINT_NUM                 1
#define AXS_TOUCH_EVENT_POS                 2
#define AXS_TOUCH_X_H_POS                   2
#define AXS_TOUCH_X_L_POS                   3
#define AXS_TOUCH_ID_POS                    4
#define AXS_TOUCH_Y_H_POS                   4
#define AXS_TOUCH_Y_L_POS                   5
#define AXS_TOUCH_WEIGHT_POS                6
#define AXS_TOUCH_AREA_POS                  7
#define AXS_GET_POINT_NUM(buf) buf[AXS_TOUCH_POINT_NUM]
#define AXS_GET_GESTURE_TYPE(buf)  buf[AXS_TOUCH_GESTURE_POS]
#define AXS_GET_POINT_X(buf,point_index) (((uint16_t)(buf[AXS_TOUCH_ONE_POINT_LEN*point_index+AXS_TOUCH_X_H_POS] & 0x0F) <<8) + (uint16_t)buf[AXS_TOUCH_ONE_POINT_LEN*point_index+AXS_TOUCH_X_L_POS])
#define AXS_GET_POINT_Y(buf,point_index) (((uint16_t)(buf[AXS_TOUCH_ONE_POINT_LEN*point_index+AXS_TOUCH_Y_H_POS] & 0x0F) <<8) + (uint16_t)buf[AXS_TOUCH_ONE_POINT_LEN*point_index+AXS_TOUCH_Y_L_POS])
#define AXS_GET_POINT_EVENT(buf,point_index) (buf[AXS_TOUCH_ONE_POINT_LEN*point_index+AXS_TOUCH_EVENT_POS] >> 6)
/////////////////////////////////////////////////////////////



uint16_t old_millis;
uint16_t digit1_scroll = 0;
uint16_t digit2_scroll = 0;
uint16_t digit3_scroll = 0;
uint16_t digit4_scroll = 0;
uint16_t arc_val = 0;
uint32_t loop_count = 0;
uint16_t tally = 0;

#define bias 219


bool old_t_int = 0;


static const uint16_t screenWidth  = 640;
static const uint16_t screenHeight = 180;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf;
static lv_color_t *buf1;




/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
  
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );
 
    h = h & 0xFE;// make the refresh area height an even number - required for this display

    //Serial.printf("x %d     y %d     w %d     h %d \n", area->x1, area->y1, w, h);// debug
    
    lcd_PushColors_rotated_90(area->x1, area->y1 , w, h, (uint16_t *)&color_p->full);
    lv_disp_flush_ready(disp);
    
}

void my_rounder(lv_disp_drv_t * disp, lv_area_t * area)
{
   area->y1 = area->y1 & 0xFE; // round down the refresh area y-axis start and end points to next even number - required for this display
   area->y2 = area->y2 & 0xFE; //
}



void    my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
    //xSemaphoreTake(xSemaphore, portMAX_DELAY);
    uint8_t buff[20] = {0};

    Wire.beginTransmission(0x3B);
    Wire.write(read_touchpad_cmd, 8);
    Wire.endTransmission();
    Wire.requestFrom(0x3B, 8);
    while (!Wire.available())
        ;
    Wire.readBytes(buff, 8);

    uint16_t pointX;
    uint16_t pointY;
    uint16_t type = 0;

    type   = AXS_GET_GESTURE_TYPE(buff);
    pointX = AXS_GET_POINT_X(buff, 0);
    pointY = AXS_GET_POINT_Y(buff, 0);

    if (!type && (pointX || pointY)) {
        pointX = (640 - pointX);
        pointY = (180 - pointY);
        if (pointX > 640)
            pointX = 640;
        if (pointY > 180)
            pointY = 180;
        data->state   = LV_INDEV_STATE_PR;
        data->point.x = pointX;
        data->point.y = pointY;

    //Serial.printf("x = %d, y = %d\n", pointX, pointY); 

    } else {
        data->state = LV_INDEV_STATE_REL;
    }
    //xSemaphoreGive(xSemaphore);
}


    
void setup() {
    //xSemaphore = xSemaphoreCreateBinary();
    //xSemaphoreGive(xSemaphore);
    pinMode(TOUCH_INT, INPUT);
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, LOW);        // turn off backlight asap to minimise power on artifacts
    Serial.begin(115200); 
    pinMode(TOUCH_RES, OUTPUT);
    digitalWrite(TOUCH_RES, HIGH);delay(2);
    digitalWrite(TOUCH_RES, LOW);delay(10);
    digitalWrite(TOUCH_RES, HIGH);delay(2);
    Wire.begin(TOUCH_IICSDA, TOUCH_IICSCL);
    axs15231_init();
    //lcd_setRotation(2);             // 180 degree hardware rotate if you want reset / boot buttons at the bottom
    lcd_fill(0,0,180,640,0x00);       // clear screen using native portrait as it is faster
    digitalWrite(TFT_BL, HIGH);       // turn on backlight

   lv_init();

   size_t buffer_size = sizeof(lv_color_t) * screenWidth * screenHeight;

    buf = (lv_color_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    if (buf == NULL) {
      while (1) {
        Serial.println("buf NULL");
        delay(500);
      }
    }


    // buf1 = (lv_color_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    // if (buf1 == NULL) {
    //   while (1) {
    //     Serial.println("buf NULL");
    //     delay(500);
    //   }
    // }


   lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight  );

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.rounder_cb = my_rounder;
    //disp_drv.full_refresh = 1; //very slow but useful for debugging
    lv_disp_drv_register( &disp_drv );

    /*Initialize the (dummy) input device driver*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register( &indev_drv );

    ui_init();

    Serial.println( "Setup done" );
}


void loop() {     

    lv_timer_handler(); 
    delay(5);   
    
    lv_obj_set_y( ui_Label01, bias - digit1_scroll ); digit1_scroll++; if (digit1_scroll == 440) digit1_scroll = 0;
    if (tally % 10 == 0) {lv_obj_set_y( ui_Label02, bias - digit2_scroll ); digit2_scroll++; if (digit2_scroll == 440) digit2_scroll = 0;}
    if (tally % 100 == 0) {lv_obj_set_y( ui_Label03, bias - digit3_scroll ); digit3_scroll++; if (digit3_scroll == 440) digit3_scroll = 0;}
    if (tally % 1000 == 0) {lv_obj_set_y( ui_Label04, bias - digit4_scroll ); digit4_scroll++; if (digit4_scroll == 440) digit4_scroll = 0;}
    if (tally % 10000 == 0) {loop_count = 0; tally = 0;}
     
    lv_arc_set_value(ui_Arc1, arc_val++);
    if (arc_val > 440) arc_val = 0;
    loop_count++;
    tally = (loop_count / 44)+1;


///// for touch debug /////////////////////////////////

    bool t_int = digitalRead(TOUCH_INT);
    if (t_int != old_t_int) 
    {
    if(t_int == 0) Serial.println(millis());
    if(t_int == 1) Serial.println(millis());
    old_t_int = t_int;
    }

///////////////////////////////////////////////////////


}    


void check_for_memory_leaks() {

    Serial.print(F("Total heap  ")); Serial.println(ESP.getHeapSize());
    Serial.print(F("Free heap   ")); Serial.println(ESP.getFreeHeap());
    Serial.print(F("Total psram ")); Serial.println(ESP.getPsramSize());
    Serial.print(F("Free psram  ")); Serial.println(ESP.getFreePsram());
    Serial.println(F(" "));
}


