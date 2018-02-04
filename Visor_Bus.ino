
////////////////////////////////////////
///  V1.2 Visor
////////////////////////////////////////


#if defined ARDUINO_SAM_DUE
#include <FreeRTOS_ARM.h>
#else
#include <FreeRTOS_AVR.h>
#endif

#include <Wire.h>
#include <SPI.h>
#include <CleO.h>
#include "mcp_can_j1939_v2.h"
#include "can_ext_j1939_v2.h"
//#include <EEPROM.h>

#define Int_Can 20

int16_t tag;

struct trama_can {
//TRF1
uint16_t temperatura_transmision;
//EEC2
uint16_t Accelerator_Pedal_Position_1;
//EEC1
float engine_speed_rpm;
//VEP1
float Vehicle_Electrical_Power;
//ET1
uint16_t Engine_Coolant_Temperature;
//EFL_P1
uint16_t Engine_Oil_Pressure;
//CCSS
//DM01
bool ProtectWarninglamp;
bool AmberWarninglamp;
bool RedWarninglamp;
bool MalfunctionWarninglamp;
bool FlashProtectWarninglamp;
bool FlashAmberWarninglamp;
bool FlashRedWarninglamp;
bool FlashMalfunctionWarninglamp;
//Fuel
uint8_t Fuel_Level1;
uint8_t Fuel_Level2;
uint8_t Fuel_Level3;
} can_mega2560;

struct AMessage {
byte nPriority;
byte nSrcAddr;
byte nDestAddr;
byte nData[8];
int nDataLen;
long lPGN;
long time;
uint32_t cuenta = 0;
} xMessage, *pxMessage, *pxRxedMessage;

unsigned long tiempo_acumulado, tiempo_total;
int eeAddress;
int inicio = 27;
int match;

//Asignaciones
QueueHandle_t xQueue1;
TaskHandle_t xHandle_1, xHandle_2, xHandle_3;

const char* print_number(uint32_t Ax) {

static char buf[9];
sprintf(buf, "%d", Ax);
return buf;
}

void setup()
{
//analogReference(EXTERNAL);
Serial.begin(9600);

if (xTaskCreate(vPeriodicTask1, "CAN_RX", 200, NULL, tskIDLE_PRIORITY + 1, &xHandle_1) == pdPASS) { NULL; }
if (xTaskCreate(vPeriodicTask2, "Alive",200, NULL, tskIDLE_PRIORITY + 1, &xHandle_2) == pdPASS) { NULL; }
//if (xTaskCreate(vPeriodicTask3, "LCD", 200, NULL, tskIDLE_PRIORITY + 1, &xHandle_3) == pdPASS) { NULL; }

// Creamos la cola
xQueue1 = xQueueCreate(10, sizeof(struct AMessage *));
if (xQueue1 == NULL) Serial.println("Queue Fail !"); else Serial.println("Queue Created OK.");

// 11=250k @8MHz 10=250K @16MHz
// Initialize the CAN controller
if (canInitialize(11) == CAN_OK)
Serial.print("CAN Init OK.\n\r");
else
Serial.print("CAN Init Failed.\n\r");

// Initialize CleO - needs to be done only once
//noInterrupts();
CleO.begin();

// Get the handle for the image
int16_t im = CleO.LoadImageFile("Pictures/Begas2.jpg", 0);
// Start building a screen frame
CleO.Start();
// Draw a bitmap at (0, 0) using im handle
CleO.Bitmap(im, 0, 0);
// Display completed screen frame
CleO.Show();

delay(3000);

// Place your custom setup code here
attachInterrupt(digitalPinToInterrupt(Int_Can), recibir_can, FALLING);   //Can

vTaskStartScheduler();
for (;;);
}

void vPeriodicTask1()
{
TickType_t xLastWakeTime = xTaskGetTickCount();

static uint16_t RPM;
static int16_t x, y, dur;
static int key = 0;
static char temp[sizeof("Crono: %02d:%02d:%02d")];

#define offset_horizontal -40

for (;;)
{
       sprintf(temp, "Crono: %02d:%02d:%02d", (int)(millis() / 3600000) % 60, (int)(millis() / 60000) % 60, (int)(millis() / 1000) % 60);

noInterrupts();
CleO.Start();

CleO.SetBackgroundcolor(BLACK);

CleO.StringExt(FONT_SANS_3, 150, 220, WHITE, MM, 0, 0, temp);
CleO.StringExt(FONT_SANS_4, 400, 450, GOLD, MM, 0, 0,"(C) BEGAS MOTOR 2017" );

/*
switch (key)
{
case 0: CleO.StringExt(FONT_SANS_3, 150, 220, WHITE, MM, 0, 0,"x1" );
break;
case 1: CleO.StringExt(FONT_SANS_3, 150, 220, WHITE, MM, 0, 0, "x10");
break;
case 2: CleO.StringExt(FONT_SANS_3, 150, 220, WHITE, MM, 0, 0, "x100");
break;
case 3: CleO.StringExt(FONT_SANS_3, 150, 220, WHITE, MM, 0, 0, "x1000");
break;
default: key = 0;
break;
}
*/

CleO.Tag(85);
CleO.StringExt(FONT_SANS_7, 720+offset_horizontal, 140, GOLD, MM, 0, 0, print_number(can_mega2560.Engine_Oil_Pressure));
CleO.StringExt(FONT_SANS_3, 720+offset_horizontal, 80, LIGHT_GREEN, MM, 0, 0, "Engine Oil (kPa)");
CleO.Tag(86);
CleO.StringExt(FONT_SANS_7, 530+offset_horizontal, 140, GOLD, MM, 0, 0, print_number(can_mega2560.Vehicle_Electrical_Power));
CleO.StringExt(FONT_SANS_3, 530+offset_horizontal, 80, LIGHT_GREEN, MM, 0, 0, "Voltage (V)");
CleO.Tag(87);
CleO.StringExt(FONT_SANS_7, 340+offset_horizontal, 140, GOLD, MM, 0, 0, print_number(can_mega2560.temperatura_transmision));
CleO.StringExt(FONT_SANS_3, 340+offset_horizontal, 80, LIGHT_GREEN, MM, 0, 0, "Oil TX Temp (C)");
CleO.Tag(88);
CleO.StringExt(FONT_SANS_7, 150+offset_horizontal, 140, GOLD, MM, 0, 0, print_number(can_mega2560.engine_speed_rpm));
CleO.StringExt(FONT_SANS_3, 150+offset_horizontal, 80, LIGHT_GREEN, MM, 0, 0, "RPM (min-1)");
CleO.Tag(89);
CleO.StringExt(FONT_SANS_7, 150+offset_horizontal, 340, GOLD, MM, 0, 0, "---");
CleO.StringExt(FONT_SANS_3, 150+offset_horizontal, 280, LIGHT_GREEN, MM, 0, 0, "Engine Oil (L)");
CleO.Tag(90);
CleO.StringExt(FONT_SANS_7, 340+offset_horizontal, 340, GOLD, MM, 0, 0, print_number(can_mega2560.Engine_Coolant_Temperature));
CleO.StringExt(FONT_SANS_3, 340+offset_horizontal, 280, LIGHT_GREEN, MM, 0, 0, "Engine Temp (C)");
CleO.Tag(91);
CleO.StringExt(FONT_SANS_7, 530+offset_horizontal, 340, GOLD, MM, 0, 0, print_number(can_mega2560.Accelerator_Pedal_Position_1));
CleO.StringExt(FONT_SANS_3, 530+offset_horizontal, 280, LIGHT_GREEN, MM, 0, 0, "Acelarator (%)");
CleO.Tag(92);
CleO.StringExt(FONT_SANS_7, 720+offset_horizontal, 340, GOLD, MM, 0, 0, "---");
CleO.StringExt(FONT_SANS_3, 720+offset_horizontal, 280, LIGHT_GREEN, MM, 0, 0, "GLP level (%)");

//procesado toque pantalla
CleO.TouchCoordinates(x, y, dur, tag);

switch (tag)
       {
case 95:
key++;
break;
default:
break;
}

// Display completed screen frame
CleO.Show();
interrupts();

vTaskDelayUntil(&xLastWakeTime, (100 / portTICK_PERIOD_MS));

             }

       }


void vPeriodicTask2()
{
TickType_t xLastWakeTime = xTaskGetTickCount();
static char sString[80];

for (;;)
{
// Lectura cola CAN RX
if (xQueue1 != 0)
{
// Receive a message on the created queue.  Block for 10 ticks if a message is not immediately available
if (xQueueReceive(xQueue1, &(pxRxedMessage), (TickType_t)10))
{
             long lMessageID2 = ((long)pxRxedMessage->nPriority << 26) + ((long)pxRxedMessage->lPGN << 8) + (long)pxRxedMessage->nSrcAddr;

             switch (lMessageID2)
             {
             case 0x18FEF803:
             can_mega2560.temperatura_transmision = (uint16_t)(0.03125* (pxRxedMessage->nData[4] + 256 * pxRxedMessage->nData[5]) - 273);//ok
             break;
             case 0x0CF00400:
             can_mega2560.engine_speed_rpm = .125* pxRxedMessage->nData[3] + 32 * pxRxedMessage->nData[4];//ok
             break;
             case 0x0CF00300:
             can_mega2560.Accelerator_Pedal_Position_1 = .4* pxRxedMessage->nData[1];//ok
             break;
             case 0x18FEF700:
             can_mega2560.Vehicle_Electrical_Power= .05* pxRxedMessage->nData[4] + 12.75 * pxRxedMessage->nData[5];//ok
             break;
             case 0x18FEEE00:
             can_mega2560.Engine_Coolant_Temperature = pxRxedMessage->nData[0]-40;//ok
             break;
             case 0x18FEEF00:
             can_mega2560.Engine_Oil_Pressure = 4* pxRxedMessage->nData[3];//ok
             break;
             default:
             break;
}

ltoa(lMessageID2, sString, 16);
strupr(sString);
Serial.print("ID:");
Serial.print(sString);
Serial.print(" ");
sprintf(sString, "PGN:%05u P:%X S:%X D:%X \n\r", (int)pxRxedMessage->lPGN, pxRxedMessage->nPriority, pxRxedMessage->nSrcAddr, pxRxedMessage->nDestAddr);
Serial.print(sString);
Serial.print("Data: ");
for (int nIndex = 0; nIndex < pxRxedMessage->nDataLen; nIndex++)
{
sprintf(sString, "%X ", pxRxedMessage->nData[nIndex]);
Serial.print(sString);
} Serial.println("");

vTaskDelayUntil(&xLastWakeTime, (50 / portTICK_PERIOD_MS));
}
}//if xQueue1
}
}

void recibir_can() {

if (xQueue1 != NULL)
{
j1939Receive(&xMessage.lPGN, &xMessage.nPriority, &xMessage.nSrcAddr, &xMessage.nDestAddr, xMessage.nData, &xMessage.nDataLen);
pxMessage = &xMessage;
xQueueSend(xQueue1, (void *)&pxMessage, (TickType_t)0);
       }
}

/*
//LCD
void vPeriodicTask3()
{
       TickType_t xLastWakeTime = xTaskGetTickCount();
       static  int i;
       static unsigned long time;

       for (;;)
       {

             // Horometro
             while (i++ < 6) {

                    time = millis() - tiempo_total;
                    //mostrar_LCD_derecho((int)(((time / 1000) % 60) + 100 * ((time / 60000) % 60)), 2);
                    //mostrar_LCD_izquierdo((int)((time / 3600000) % 24) + 100 * (time / 86400000), 2);
                    delay(1000);

             } i = 0;

             EEPROM.put(eeAddress, (unsigned long)time + tiempo_acumulado);

             tiempo_total = millis();

             vTaskDelayUntil(&xLastWakeTime, (500 / portTICK_PERIOD_MS));
       }
}
*/

void loop() { }
