#include <EEPROMex.h>
#include <EEPROMVar.h>

#include <Servo.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_NeoPixel.h>

// Общие настройки
#define DELAY_BEFORE_LOAD 3000

// Распиновка
#define ESC_PIN     9
#define SLEFT_PIN   5	// левая серва
#define SRIGHT_PIN  6	// правая серва
#define BEEP_PIN    A0	// бипер
#define BEEP_SW_PIN 7	// выключатель бипера
#define MODE_SW_PIN 3
#define LED_PIN     A3	// адресуемые светодиоды
#define RF_PIN_CE   8
#define RF_PIN_CSN  10

// настройки светодиодов
#define LED_COUNT 2

#define COLOR_NONE  0x00000000
#define COLOR_WHITE 0x04000000
#define COLOR_RED   0x00040000
#define COLOR_GREEN 0x00000400
#define COLOR_BLUE  0x00000004

#define COLOR_QUEU_DELAY 250

// режимы
#define MODE_FLIGHT   0
#define MODE_SETTINGS 1

// Параметры радиоканала
#define RF_CHANNEL   100
#define RF_DATA_RATE RF24_2MBPS
#define RF_PA_LEVEL  RF24_PA_HIGH
// Адрес канала комманд
#define RF_CMD_CH 0xA4D3F4E1
// Адрес канала телеметрии
#define RF_TEL_CH 0xA4D3F4E2
// Тайм-аут для принятия данных
#define RECEIVE_TIMEOUT 100.0
// Период переотправки команд (мс)
#define RESEND_DELAY 50

// Биты протокола
#define PROTO_BIT_LAST           0b01000000
#define PROTO_BIT_FLIGHT         0b00100000
#define PROTO_BIT_ALL            PROTO_BIT_LAST | PROTO_BIT_FLIGHT

// команды протокола
#define PROTO_MSG_PING           0b00000001
#define PROTO_MSG_FLIGHT_DATA0   0b00000100
#define PROTO_MSG_FLIGHT_DATA1   0b00000101
#define PROTO_MSG_SETTINGS_REQ   0b00001000
#define PROTO_MSG_SETTING_PUSH   0b00001001
#define PROTO_MSG_SETTING_SAVE   0b00001010
#define PROTO_MSG_SETTINGS_RESET 0b00001100
#define PROTO_MSG_SETTINGS_WRITE 0b00001101

// Маски режимов
#define MODE_BIT_FAILSAFE 0b00000001
#define MODE_BIT_ARMING   0b00000010
#define MODE_BIT_BEEPER   0b00000100

// Настройки
#define ESC_PL_ZERO   0
#define ESC_PL_FS     1
#define ESC_PL_MIN    2
#define ESC_PL_MAX    3
#define SLEFT_PL_MIN  4
#define SLEFT_PL_MAX  5
#define SRIGHT_PL_MIN 6
#define SRIGHT_PL_MAX 7


// Парамметры приёма/передачи
#define SEND_POOL_SIZE    16
#define RECV_PACKET_MAX   16
#define RECEIVE_TIMEOUT  300
#define RECEIVE_STAT_MAX  32

// минимально и максимальнодопустимые значения настроек
#define SERVO_MIN_VALUE  700
#define SERVO_MAX_VALUE 2300

#define FD_THR    0
#define FD_SLEFT  1
#define FD_SRIGHT 2

#define RECEIVE_STAT_FAILSAFE 16
#define RECEIVE_STAT_DISARM   2

uint16_t settings_values[] = {700, 1100, 1100, 2100, 1700, 1000, 1000, 1800};

bool is_flight_mode = false;

// установленные режимы
uint8_t mode_bits = 0;
uint8_t mode_bits_prev = 1;

// передаваемое слово
uint32_t rf_trans = 0;
// принимаемое слово
uint32_t rf_recv  = 0;
// Пул отправляемых запросов
uint32_t send_pool[SEND_POOL_SIZE];
uint8_t  send_pool_size = 0;

// время последнего приёма и статистика приема
int receive_stat = 0;
int receive_stat_prev = 32;

// автоматический Failsafe
bool auto_failsafe = false;
// форсированный Failsafe
bool force_failsafe = false;

uint16_t flight_data[] = {
	settings_values[ESC_PL_ZERO],
	(settings_values[SLEFT_PL_MIN] + settings_values[SLEFT_PL_MAX]) / 2,
	(settings_values[SRIGHT_PL_MIN] + settings_values[SRIGHT_PL_MAX]) / 2,
};

Servo esc, sleft, sright;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRBW);
RF24 radio(RF_PIN_CE, RF_PIN_CSN);

void setup()
{
	for (int8_t setting = 0; setting < 8; setting++) {
		settings_values[setting] = min(SERVO_MAX_VALUE, max(SERVO_MIN_VALUE, EEPROM.readInt(setting * 2)));
	}

	// подготавливаем сервы
	sleft.attach(SLEFT_PIN);
	sleft.writeMicroseconds((settings_values[SLEFT_PL_MIN] + settings_values[SLEFT_PL_MAX]) / 2);
	sright.attach(SRIGHT_PIN);
	sright.writeMicroseconds((settings_values[SRIGHT_PL_MIN] + settings_values[SRIGHT_PL_MAX]) / 2);
	// иницируем контроллер мотора
	esc.attach(ESC_PIN);
	esc.writeMicroseconds(settings_values[ESC_PL_ZERO]);

	// инициализируем прочие пины
	pinMode(BEEP_PIN, OUTPUT);
	pinMode(BEEP_SW_PIN, INPUT_PULLUP);
	pinMode(MODE_SW_PIN, INPUT_PULLUP);

	is_flight_mode = !digitalRead(MODE_SW_PIN);
	if (is_flight_mode) {
		initFlightMode();
	} else {
		initSettingsMode();
	}

	// инициализируем приёмник
    radio.begin();
    radio.setChannel(RF_CHANNEL);
    radio.setDataRate(RF_DATA_RATE);
    radio.setPALevel(RF_PA_LEVEL);
    radio.openWritingPipe(RF_TEL_CH);
    radio.openReadingPipe(1, RF_CMD_CH);
	radio.startListening();
}

/**
 * Инициализация полётного режима
 */
void initFlightMode()
{
	if (!digitalRead(BEEP_SW_PIN)) {
		for (int i = 1; i < 5; i++) {
			digitalWrite(BEEP_PIN, HIGH);
			delay(100);
			digitalWrite(BEEP_PIN, LOW);
			delay((6 - i) * 10);
		}
	}

	// инициализируем и проверяем светодиоды
	pixels.begin();
    pixels.show();

	uint32_t color_queue[] = {
		COLOR_WHITE,
		COLOR_RED,
		COLOR_GREEN,
		COLOR_BLUE,
		COLOR_NONE,
	};

	pixels.setPixelColor(0, color_queue[0]);
	pixels.show();
	delay(COLOR_QUEU_DELAY);

	for (int i = 1; i < sizeof(color_queue) / sizeof(uint32_t); i++) {
		pixels.setPixelColor(0, color_queue[i]);
		pixels.setPixelColor(1, color_queue[i - 1]);
		pixels.show();
		delay(COLOR_QUEU_DELAY);
	}

	pixels.setPixelColor(0, COLOR_NONE);
	pixels.setPixelColor(1, COLOR_NONE);
	pixels.show();

	while (millis() <= DELAY_BEFORE_LOAD);

	pixels.setPixelColor(1, COLOR_GREEN);
	pixels.show();
}

/**
 * Инициализация режима настроек
 */
void initSettingsMode()
{
	// инициализируем и проверяем светодиоды
	pixels.begin();
    pixels.show();

	uint32_t color_queue[] = {
		COLOR_RED,
		COLOR_GREEN,
		COLOR_BLUE,
	};

	bool beep_enable = digitalRead(BEEP_SW_PIN);

	for (int i = 0; i < 3; i++) {
		pixels.setPixelColor(0, color_queue[i]);
		pixels.setPixelColor(1, color_queue[i]);
		pixels.show();
		if (!beep_enable) {
			digitalWrite(BEEP_PIN, HIGH);
		}
		delay(COLOR_QUEU_DELAY);
		pixels.setPixelColor(0, COLOR_WHITE);
		pixels.setPixelColor(1, COLOR_WHITE);
		pixels.show();
		digitalWrite(BEEP_PIN, LOW);
		delay(COLOR_QUEU_DELAY);
	}

	pixels.setPixelColor(0, COLOR_NONE);
	pixels.setPixelColor(1, COLOR_NONE);
	pixels.show();

	while (millis() <= DELAY_BEFORE_LOAD);
}

void loop(void)
{
	int8_t pack_num = 0;
    while(readRfData() && !onDataReceived() && (pack_num++ < RECV_PACKET_MAX));
    radio.stopListening();
	writeRfData();
	if (is_flight_mode) {
		flight();
	}
	radio.startListening();
    drawReceiveStat();    
}

bool readRfData()
{
	unsigned long start_listen_time = millis();
    bool data_received = false;

    do {
    	data_received = radio.available();
    } while (!data_received && ((millis() - start_listen_time) < RECEIVE_TIMEOUT));

	if (data_received) {
	    do {
	        data_received = radio.read(&rf_recv, sizeof(rf_recv));
	    } while (!data_received && (millis() - start_listen_time) < RECEIVE_TIMEOUT);
	}

    if (data_received) {
    	receive_stat++;
    	if (receive_stat > RECEIVE_STAT_MAX) receive_stat = RECEIVE_STAT_MAX;
    } else {
    	receive_stat --;
    	if (receive_stat < 0) receive_stat = 0;
    }
	return data_received;
}

bool addMessageToSendPool(uint32_t message)
{
	if (send_pool_size >= SEND_POOL_SIZE) {
		return false;
	}
	send_pool[send_pool_size] = message;
	send_pool_size++;
	return true;
}

bool onDataReceived()
{
	if (is_flight_mode) {
		decodeFlightCommand();
	} else {
		decodeSettingsCommand();
	}
	return ((rf_recv >> 24) & PROTO_BIT_LAST);
}

void writeRfData()
{
	if (send_pool_size == 0) {
		rf_trans = (uint32_t)(PROTO_MSG_PING | PROTO_BIT_LAST | (is_flight_mode ? PROTO_BIT_FLIGHT : 0)) << 24;
		radio.write(&rf_trans, sizeof(rf_trans));
		return;
	}

	do {
		send_pool_size--;
		rf_trans = send_pool[send_pool_size] | (is_flight_mode ? PROTO_BIT_FLIGHT : 0);
		if (send_pool_size == 0) {
			rf_trans |= (uint32_t)PROTO_BIT_LAST << 24;
		}
		radio.write(&rf_trans, sizeof(rf_trans));
	} while (send_pool_size > 0);
}

void drawReceiveStat()
{
	if ((receive_stat_prev >> 2) != (receive_stat >> 2)) {
		uint32_t color = ((uint32_t)(RECEIVE_STAT_MAX - receive_stat) << 16) | ((uint32_t)receive_stat << 8);
		pixels.setPixelColor(1, color);
		pixels.show();
		receive_stat_prev = receive_stat;
	}
}

void decodeFlightCommand()
{
	uint32_t cmd = (rf_recv >> 24) & ~((uint32_t)PROTO_BIT_ALL);
	switch (cmd) {
		case PROTO_MSG_FLIGHT_DATA0:
			decodeFlightData();
			break;

		case PROTO_MSG_FLIGHT_DATA1:
			decodeFlightMode();
			break;
	}
}

void decodeFlightData()
{
	flight_data[FD_THR] = map((rf_recv & 0xFF), 0, 0xFF, settings_values[ESC_PL_MIN], settings_values[ESC_PL_MAX]);

	float pch  = (uint32_t)(rf_recv >> 8) & 0xFF;
	float roll = (uint32_t)(rf_recv >> 16) & 0xFF;

	float sl = sqrt(2.0) / 2.0 * ((pch - 127.5) + (roll - 127.5)) + 127.5;
	float sr = sqrt(2.0) / 2.0 * ((pch - 127.5) - (roll - 127.5)) + 127.5;

	flight_data[FD_SLEFT]  = map(sl, 0, 0xFF, settings_values[SLEFT_PL_MIN], settings_values[SLEFT_PL_MAX]);
	flight_data[FD_SRIGHT] = map(sr, 0, 0xFF, settings_values[SRIGHT_PL_MIN], settings_values[SRIGHT_PL_MAX]);
}

void decodeFlightMode()
{
	mode_bits = rf_recv & 0xFF;
}

void flight()
{
	bool failsafe = (mode_bits & MODE_BIT_FAILSAFE) || (receive_stat <= RECEIVE_STAT_FAILSAFE);

	if (mode_bits & MODE_BIT_ARMING) {
		if (failsafe) {
			esc.writeMicroseconds(settings_values[(receive_stat < RECEIVE_STAT_DISARM) ? ESC_PL_ZERO : ESC_PL_FS]);
		} else {
			esc.writeMicroseconds(flight_data[FD_THR]);
		}
	} else {
		esc.writeMicroseconds(settings_values[ESC_PL_ZERO]);
	}

	sleft.writeMicroseconds(flight_data[FD_SLEFT]);
	sright.writeMicroseconds(flight_data[FD_SRIGHT]);

	if (mode_bits != mode_bits_prev) {
		if ((mode_bits_prev & MODE_BIT_BEEPER) != (mode_bits & MODE_BIT_BEEPER)) {
			digitalWrite(BEEP_PIN, mode_bits & MODE_BIT_BEEPER);
		}
		mode_bits_prev = mode_bits;
	}
}

void decodeSettingsCommand()
{
	uint32_t cmd = (rf_recv >> 24) & ~((uint32_t)PROTO_BIT_ALL);
	switch (cmd) {
		case PROTO_MSG_SETTINGS_REQ:
			decodeSettingsGet();
			break;

		case PROTO_MSG_SETTING_PUSH:
			decodeSettingPush();
			break;

		case PROTO_MSG_SETTING_SAVE:
			decodeSettingSave();
			break;

		case PROTO_MSG_SETTINGS_RESET:
			decodeSettingsReset();
			break;

		case PROTO_MSG_SETTINGS_WRITE:
			decodeSettingsWrite();
			break;
	}
}

void decodeSettingsGet()
{
	uint32_t settings = rf_recv & (uint32_t)0xFF;
	for (uint32_t setting = 0; setting < 8; setting++) {
		if ((settings >> setting) & 0b1) {
			uint32_t message = ((uint32_t)PROTO_MSG_SETTING_SAVE << 24) | (uint32_t)(setting << 16) | (uint32_t)settings_values[setting];
			addMessageToSendPool(message);
		}
	}
}

void decodeSettingPush()
{
	uint8_t setting = (rf_recv >> 16) & 0xFF;
	uint16_t setting_val = rf_recv & 0xFFFF; 
    switch (setting) {
    	case ESC_PL_ZERO:
    	case ESC_PL_FS:
    	case ESC_PL_MIN:
    	case ESC_PL_MAX:
    		esc.writeMicroseconds(setting_val);
    		break;
    	
    	case SLEFT_PL_MIN:
    	case SLEFT_PL_MAX:
    		sleft.writeMicroseconds(setting_val);
    		break;

    	case SRIGHT_PL_MIN:
    	case SRIGHT_PL_MAX:
    		sright.writeMicroseconds(setting_val);
    		break;
    }
	uint32_t message = ((uint32_t)PROTO_MSG_SETTING_PUSH << 24) | ((uint32_t)setting << 16) | (uint32_t)setting_val;
	addMessageToSendPool(message);
}

void decodeSettingSave()
{
    uint8_t setting = (rf_recv >> 16) & 0xFF;
	settings_values[setting] = rf_recv & 0xFFFF; 

    switch (setting) {
    	case ESC_PL_ZERO:
    	case ESC_PL_FS:
    	case ESC_PL_MIN:
    	case ESC_PL_MAX:
    		esc.writeMicroseconds(settings_values[ESC_PL_ZERO]);
    		break;
    	
    	case SLEFT_PL_MIN:
    	case SLEFT_PL_MAX:
    		sleft.writeMicroseconds((settings_values[SLEFT_PL_MIN] + settings_values[SLEFT_PL_MAX]) / 2);
    		break;

    	case SRIGHT_PL_MIN:
    	case SRIGHT_PL_MAX:
    		sright.writeMicroseconds((settings_values[SRIGHT_PL_MIN] + settings_values[SRIGHT_PL_MAX]) / 2);
    		break;
    }

 	uint32_t message = ((uint32_t)PROTO_MSG_SETTING_PUSH << 24) | ((uint32_t)setting << 16) | settings_values[setting];
	addMessageToSendPool(message);	   
}

void decodeSettingsReset()
{
	for (int8_t setting = 0; setting < 8; setting++) {
		settings_values[setting] = min(SERVO_MAX_VALUE, max(SERVO_MIN_VALUE, EEPROM.readInt(setting * 2)));
	}
 	uint32_t message = (uint32_t)PROTO_MSG_SETTINGS_RESET << 24;
	addMessageToSendPool(message);	
}

void decodeSettingsWrite()
{
	for (int8_t setting = 0; setting < 8; setting++) {
		EEPROM.updateInt(setting * 2, settings_values[setting]);
	}
 	uint32_t message = (uint32_t)PROTO_MSG_SETTINGS_WRITE << 24;
	addMessageToSendPool(message);
}


/**

TRANCIEVER						RECEIVER

# get data from 

	-> PROTO_MSG_SETTINGS_REQ ->
	<- PROTO_MSG_SETTING_SAVE <-
	.....
	<- PROTO_MSG_SETTING_SAVE <-

# push test data

	-> PROTO_MSG_SETTING_PUSH ->
	<- PROTO_MSG_SETTING_PUSH <-

# save test data

	-> PROTO_MSG_SETTING_SAVE ->
	<- PROTO_MSG_SETTING_SAVE <-

**/


