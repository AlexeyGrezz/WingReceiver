#include <Adafruit_GFX.h>
#include <gfxfont.h>

#include <Adafruit_ST7735.h>

#include <SPI.h>
#include <RF24.h>

// Пины, к которым подключены кнопки стиков
#define LEFT_BTN 1
#define RIGHT_BTN 0

// Пины, к которым подключены потенциометры стиков
#define STICK_ROLL A0
#define STICK_PCH  A1
#define STICK_YAW  A2
#define STICK_THRD A3
#define STICK_SENS A6
#define STICK_THRS A7

// Параметры стиков
#define STICK_POS_MIN    0
#define STICK_POS_MID  512
#define STICK_POS_MAX 1024
#define STICK_POS_LAG 	8

// Пины подключения передатчика
#define RF_PIN_CE  9
#define RF_PIN_CSN 10
// Параметры радиоканала
#define RF_CHANNEL   100
#define RF_DATA_RATE RF24_2MBPS
#define RF_PA_LEVEL  RF24_PA_HIGH
// Адрес канала комманд
#define RF_CMD_CH 0xA4D3F4E1
// Адрес канала телеметрии
#define RF_TEL_CH 0xA4D3F4E2

// Пины подключения дисплея
#define TFT_ROT 3
#define TFT_CS  4
#define TFT_RST 5
#define TFT_DC  6
#define TFT_LED 7

// Биты протокола
#define PROTO_BIT_LAST          0b01000000
#define PROTO_BIT_FLIGHT        0b00100000
#define PROTO_BIT_ALL           PROTO_BIT_LAST | PROTO_BIT_FLIGHT

// команды протокола
#define PROTO_MSG_PING          0b00000001
#define PROTO_MSG_FLIGHT_DATA0  0b00000100
#define PROTO_MSG_FLIGHT_DATA1  0b00000101
#define PROTO_MSG_SETTINGS_REQ  0b00001000
#define PROTO_MSG_SETTING_PUSH  0b00001001
#define PROTO_MSG_SETTING_SAVE  0b00001010
#define PROTO_MSG_SETTING_RESET 0b00001100
#define PROTO_MSG_SETTING_WRITE 0b00001101
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

// настрйока органов управления
#define CTRL_ANALOG_MAX        1024
#define CTRL_ACTION_TIMEOUT     200
#define CTRL_AXIS_VALUE         100
#define CTRL_AXIS_Y_LIMIT        50
#define CTRL_AXIS_X_SLOW_LIMIT   20
#define CTRL_AXIS_X_SLOW_VALUE   10
#define CTRL_AXIS_X_FAST_LIMIT   80
#define CTRL_AXIS_X_FAST_VALUE  100

// Позиция отображения описателя режима
#define MODE_TEXT_LEFT 5
#define MODE_TEXT_TOP  5

// Позиция счётчика пакетов
#define RECEIVE_STAT_LEFT 125
#define RECEIVE_STAT_TOP    7

// Настройки отображения линий настроек
#define FIRST_LINE_TOP  33
#define LINE_HEIGHT     20
#define COL_NAMES_LEFT  10
#define COL_VAL0_LEFT   65
#define COL_VAL1_LEFT  115
#define LINES_COUNT      5
#define BUTTONS_LINE_NUM 4

// Параметры дисплея
#define DISPLAY_W 160
#define DISPLAY_H 128
#define BADGE_W    32
#define BADGE_H    16
// Дополнительные цвета дисплея
#define ST7735_NAVY        0x000F
#define ST7735_DARKGREEN   0x03E0
#define ST7735_DARKCYAN    0x03EF
#define ST7735_MAROON      0x7800
#define ST7735_PURPLE      0x780F
#define ST7735_OLIVE       0x7BE0
#define ST7735_LIGHTGREY   0xC618
#define ST7735_DARKGREY    0x7BEF
#define ST7735_ORANGE      0xFD20
#define ST7735_GREENYELLOW 0xAFE5
#define ST7735_PINK        0xF81F
// Цвета интерфейса
#define BACKGROUND_COLOR       ST7735_BLACK
#define DEFAULT_TEXT_COLOR     ST7735_GREEN
#define WAIT_ACTION_TEXT_COLOR ST7735_ORANGE
#define DEFAULT_LINE_COLOR     ST7735_WHITE
#define SELECTED_LINE_COLOR    ST7735_ORANGE
#define SELECTED_COL_COLOR     ST7735_GREEN

// переменные для статистики принятых комманд
int8_t receive_stat = 0;
int8_t receive_stat_prev = -1;

// передаваемое слово
uint32_t rf_trans = 0;
// принимаемое слово
uint32_t rf_recv  = 0;

// Пул отправляемых запросов
uint32_t send_pool[SEND_POOL_SIZE];
uint8_t  send_pool_size = 0;

// режим работы канала
bool is_flight_mode = false;
// установленные режимы
uint16_t mode_bits = 0;

// Битовая маска тех настроек, которые надо синхронизировать с приёмником
uint8_t settings_to_sync = 0;
// Битовая маска тех настроек, которые надо сохранить на приёмнике
uint8_t settings_to_save = 0;

bool wait_write = false;
bool wait_reset = false;
bool control_lock = false;

// выбранное значение
int8_t selected_line = 0;
int8_t selected_column = 2;
int8_t selected_button = 2;

// время последнего действия
unsigned long last_btn_action_ts = 0;

uint16_t settings_values[]      = {0, 0, 0, 0, 0, 0, 0, 0};
uint16_t settings_prev_values[] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t line_col_setting_map[4][2] = {
	{ESC_PL_ZERO, ESC_PL_FS},
	{ESC_PL_MIN, ESC_PL_MAX},
	{SLEFT_PL_MIN, SLEFT_PL_MAX},
	{SRIGHT_PL_MIN, SRIGHT_PL_MAX}
};

const String line_settings_names[] = {
	"ESC FS",
	"ESC FL",
	"SERVO L",
	"SERVO R"
};

// Инициализация дополнительных устройств
Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);
RF24 radio(RF_PIN_CE, RF_PIN_CSN);

void setup()
{
    pinMode(STICK_ROLL, INPUT);
    pinMode(STICK_PCH,  INPUT);
    pinMode(STICK_YAW,  INPUT);
    pinMode(STICK_THRD, INPUT);

	pinMode(TFT_LED,    OUTPUT);
	pinMode(RIGHT_BTN,  INPUT_PULLUP);
	
	digitalWrite(TFT_LED, HIGH);

	initDisplay();

	radio.begin();
	radio.setChannel(RF_CHANNEL);
	radio.setDataRate(RF_DATA_RATE);
	radio.setPALevel(RF_PA_LEVEL);
	radio.openWritingPipe(RF_CMD_CH);
	radio.openReadingPipe(1, RF_TEL_CH);
	radio.startListening();
}

void loop(void)
{
	int8_t pack_num = 0;
    while(readRfData() && !onDataReceived() && (pack_num++ < RECV_PACKET_MAX));
    radio.stopListening();
    processControls();
    buildRequest();
	writeRfData();
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
    	receive_stat--;
    	if (receive_stat < 0) receive_stat = 0;
    }
	return data_received;
}

bool onDataReceived()
{
	uint32_t cmd = (rf_recv >> 24);
	boolean prev_flight_mode = is_flight_mode;
	is_flight_mode = (cmd & PROTO_BIT_FLIGHT) == PROTO_BIT_FLIGHT;
	cmd &= ~((uint32_t)PROTO_BIT_ALL);
	if (prev_flight_mode != is_flight_mode) {
		drawModeTitle();
	}
	switch (cmd) {
		case PROTO_MSG_SETTING_PUSH:
			decodeSettingPush();
			break;

		case PROTO_MSG_SETTING_SAVE:
			decodeSettingSave();
			break;

		case PROTO_MSG_SETTING_WRITE:
		case PROTO_MSG_SETTING_RESET:
			decodeSettingReset();
			break;
	}

	return ((rf_recv >> 24) & PROTO_BIT_LAST);
}

void decodeSettingPush()
{
	uint8_t setting = (rf_recv >> 16) & 0xFF;
	uint16_t setting_val = rf_recv & 0xFFFF; 
    if (settings_values[setting] == setting_val) {
        settings_to_sync &= ~(0b1 << setting);
    }
	for (int8_t line = 0; line < LINES_COUNT; line++) {
		drawSettingsLine(line);
	}
}

void decodeSettingSave()
{
    uint8_t setting = (rf_recv >> 16) & 0xFF;
	settings_values[setting] = rf_recv & 0xFFFF; 
    settings_to_save &= ~(0b1 << setting);
	for (int8_t line = 0; line < LINES_COUNT; line++) {
		drawSettingsLine(line);
	}
}

void decodeSettingReset()
{
	wait_reset = false;
	wait_write = false;
	for (int8_t setting = 0; setting < 8; setting++) {
		settings_values[setting] = 0;
	}
	selected_line = 2;
	selected_button = 2;
	for (int8_t line = 0; line < LINES_COUNT; line++) {
		drawSettingsLine(line, true, true);
	}
	drawModeTitle();
}

void processControls() 
{
	if (is_flight_mode) {
		processControlsFlight();
	} else {
		processControlsSettings();
	}
}

void processControlsFlight()
{
	uint32_t message;
	uint32_t thr  = map(analogRead(STICK_SENS), 0, CTRL_ANALOG_MAX, 0, 0xFF);
	uint32_t pch  = map(analogRead(STICK_PCH),  0, CTRL_ANALOG_MAX, 0, 0xFF);
	uint32_t roll = map(analogRead(STICK_ROLL), 0, CTRL_ANALOG_MAX, 0, 0xFF);

	message = ((uint32_t)PROTO_MSG_FLIGHT_DATA0 << 24) | (thr & 0xFF) | ((pch & 0xFF) << 8) | ((roll & 0xFF) << 16);
	addMessageToSendPool(message);

	uint8_t mode_bits = 0;
	// TODO: сделать нормальный, человеческий, безпасный арминг
	if (thr > 5) {
		mode_bits |= MODE_BIT_ARMING;
	}
	message = ((uint32_t)PROTO_MSG_FLIGHT_DATA1 << 24) | (uint32_t)mode_bits;
	addMessageToSendPool(message);
}

void processControlsSettings()
{
	// пока сохраняемся или сбрасываемся - ничего не делаем
	if (wait_write || wait_reset) {
		return;
	}
	if (control_lock && !digitalRead(RIGHT_BTN)) {
		return;
	}
	control_lock = false;

	if ((millis() - last_btn_action_ts) >= CTRL_ACTION_TIMEOUT) {
		int8_t axis_y = map(analogRead(STICK_PCH),  0, CTRL_ANALOG_MAX, -CTRL_AXIS_VALUE, CTRL_AXIS_VALUE);
		int8_t axis_x = map(analogRead(STICK_ROLL), 0, CTRL_ANALOG_MAX, -CTRL_AXIS_VALUE, CTRL_AXIS_VALUE);	
		// выбор линии настроек
		if (abs(axis_y) > CTRL_AXIS_Y_LIMIT && selected_column == 2) {
			last_btn_action_ts = millis();
			int8_t prev_selected_line = selected_line;
			selected_line -= axis_y / abs(axis_y);
			selected_line = max(0, min(LINES_COUNT - 1, selected_line));
			drawSettingsLine(prev_selected_line, true, true);
			drawSettingsLine(selected_line, true, true);
		}
		
		if (!digitalRead(RIGHT_BTN)) {
			if (selected_line == BUTTONS_LINE_NUM) {
				if (selected_button == 0) {
					wait_write = true;
					drawModeTitle();
				} else if (selected_button == 1) {
					wait_reset = true;
					drawModeTitle();
				}
			} else {
				last_btn_action_ts = millis();
				if (selected_column != 2) {
					uint8_t setting = line_col_setting_map[selected_line][selected_column];
					settings_to_save |= 0b1 << setting;
				}
				selected_column++;
				if (selected_column == 3) {
					selected_column = 0;
				}
			}
			control_lock = true;
			drawSettingsLine(selected_line, true);
		}

		if (abs(axis_x) > CTRL_AXIS_X_SLOW_LIMIT) {
			if (selected_line == BUTTONS_LINE_NUM) {
				selected_button += axis_x / abs(axis_x);
				selected_button = min(1, max(0, selected_button));
				drawSettingsLine(selected_line);
			} else if (selected_column != 2) {
				last_btn_action_ts = millis();
				uint32_t setting = line_col_setting_map[selected_line][selected_column];
				settings_values[setting] += ((abs(axis_x) > CTRL_AXIS_X_FAST_LIMIT) ? CTRL_AXIS_X_FAST_VALUE : CTRL_AXIS_X_SLOW_VALUE) * axis_x / abs(axis_x);
				settings_values[setting] = min(SERVO_MAX_VALUE, max(SERVO_MIN_VALUE, settings_values[setting]));
				drawSettingsLine(selected_line);
				settings_to_sync |= 0b1 << setting;
			}
		}
	}
}

void buildRequest()
{
	uint8_t settings_mask = 0;
	uint8_t setting;
	uint32_t message;
	for (setting = 0; setting < 8; setting++) {
	    uint8_t setting_mask = 0b1 << setting;
		if (settings_values[setting] == 0) {
			settings_mask |= setting_mask;
		}
		if ((settings_to_save & setting_mask) == setting_mask) {
	        message = ((uint32_t)PROTO_MSG_SETTING_SAVE << 24) | ((uint32_t)setting << 16) | (uint32_t)settings_values[setting];
	        addMessageToSendPool(message);
        }
		if ((settings_to_sync & setting_mask) == setting_mask) {
	        message = ((uint32_t)PROTO_MSG_SETTING_PUSH << 24) | ((uint32_t)setting << 16) | (uint32_t)settings_values[setting];
	        addMessageToSendPool(message);
        }
	}
    // Запрашиваем значения настроек, которые нам не известны
	if (settings_mask > 0) {
		message = ((uint32_t)PROTO_MSG_SETTINGS_REQ << 24) | settings_mask;
		addMessageToSendPool(message);
	}

	if (wait_reset) {
		message = ((uint32_t)PROTO_MSG_SETTING_RESET << 24);
		addMessageToSendPool(message);	
	}

	if (wait_write) {
		message = ((uint32_t)PROTO_MSG_SETTING_WRITE << 24);
		addMessageToSendPool(message);	
	}
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

void initDisplay()
{
	tft.initR(INITR_BLACKTAB);
	tft.setRotation(TFT_ROT);
	tft.fillScreen(BACKGROUND_COLOR);

	drawModeTitle();

	for (uint8_t line = 0; line < LINES_COUNT; line++) {
		drawSettingsLine(line, true, true);
	}
}

bool drawModeTitle()
{
	tft.setTextSize(2);
	tft.setTextColor(BACKGROUND_COLOR);
	tft.setCursor(MODE_TEXT_LEFT, MODE_TEXT_TOP);
    tft.println(is_flight_mode ? "SETTINGS" : "FLIGHT");
	tft.setTextColor((wait_write || wait_reset) ? WAIT_ACTION_TEXT_COLOR : DEFAULT_TEXT_COLOR);
	tft.setCursor(MODE_TEXT_LEFT, MODE_TEXT_TOP);
    tft.println(is_flight_mode ? "FLIGHT" : "SETTINGS");
}

void drawReceiveStat()
{
	if (receive_stat != receive_stat_prev) {
		tft.setTextSize(1);
		tft.setTextColor(BACKGROUND_COLOR);
		tft.setCursor(RECEIVE_STAT_LEFT, RECEIVE_STAT_TOP);
	    tft.println(receive_stat_prev);

		tft.setTextColor(DEFAULT_TEXT_COLOR);
		tft.setCursor(RECEIVE_STAT_LEFT, RECEIVE_STAT_TOP);
	    tft.println(receive_stat);
	    receive_stat_prev = receive_stat;		
	}
}

void drawSettingsLine(int8_t line, bool force_val = false, bool force_name = false)
{
	if (line == BUTTONS_LINE_NUM) {
		drawButtons(line);
		return;
	}
	tft.setTextSize(1);
	uint16_t line_color = (line == selected_line) ? SELECTED_LINE_COLOR : DEFAULT_LINE_COLOR;
	if (force_name) {
		tft.setTextColor(line_color);
		tft.setCursor(COL_NAMES_LEFT, FIRST_LINE_TOP + line * LINE_HEIGHT);
	    tft.println(line_settings_names[line]);
	}

	for (uint8_t column = 0; column < 2; column++) {
		uint8_t setting = line_col_setting_map[line][column];
		if (force_val || (settings_values[setting] != settings_prev_values[setting])) {
			uint8_t cursor_x = (column == 0) ? COL_VAL0_LEFT : COL_VAL1_LEFT;
			uint8_t cursor_y = FIRST_LINE_TOP + line * LINE_HEIGHT;
			tft.setTextColor(BACKGROUND_COLOR);
			tft.setCursor(cursor_x, cursor_y);
		    tft.println(settings_prev_values[setting]);

		    if (line == selected_line && column == selected_column) {
				tft.setTextColor(SELECTED_COL_COLOR);
		    } else {
				tft.setTextColor(line_color);
		    }
		    tft.setCursor(cursor_x, cursor_y);
		    tft.println(settings_values[setting]);
		    settings_prev_values[setting] = settings_values[setting];
		}
	}
}

void drawButtons(int8_t line)
{
	uint16_t line_color = (line == selected_line) ? SELECTED_LINE_COLOR : DEFAULT_LINE_COLOR;
	tft.setTextSize(1);

    if (line == selected_line && 0 == selected_button) {
		tft.setTextColor(SELECTED_COL_COLOR);
    } else {
		tft.setTextColor(line_color);
    }
	tft.setCursor(COL_VAL0_LEFT, FIRST_LINE_TOP + line * LINE_HEIGHT);
    tft.println("WRITE");

	if (line == selected_line && 1 == selected_button) {
		tft.setTextColor(SELECTED_COL_COLOR);
	} else {
		tft.setTextColor(line_color);
	}
	tft.setCursor(COL_VAL1_LEFT, FIRST_LINE_TOP + line * LINE_HEIGHT);
	tft.println("RESET");
}
