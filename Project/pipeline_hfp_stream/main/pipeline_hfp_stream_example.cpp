#include "Arduino.h"

#include "Device_Info.h"
#include <LCD_Screen.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//BLACK -> 화이트
//WHITE -> 파랑


//BLE
#include "BLE_Controller.h"

//ADF
extern "C"
{
    #include "audio_hal.h"
    #include <string.h>
    #include "esp_console.h"
    #include "esp_err.h"
    #include "nvs_flash.h"
    #include "esp_log.h"
    #include "esp_peripherals.h"
    #include "esp_bt_device.h"
    #include "esp_bt_main.h"
    #include "esp_gap_bt_api.h"
    #include "audio_pipeline.h"
    #include "audio_event_iface.h"
    #include "board.h"
    #include "hfp_stream.h"
    #include "audio_idf_version.h"
    #include "i2s_stream.h"

    static const char *TAG = FULL_DEVICE_NAME;

    static audio_element_handle_t hfp_in_stream, hfp_out_stream, i2s_stream_writer, i2s_stream_reader;
    static audio_pipeline_handle_t pipeline_in, pipeline_out;
    static audio_board_handle_t  board_handle;
    static int g_hfp_audio_rate = 16000;

    #define GPIO_SPEAKER_PIN GPIO_NUM_5  // 스피커 핀 (출력, 풀다운)
    #define GPIO_MIC_BUTTON_PIN GPIO_NUM_17 // 마이크 버튼 핀
    #define GPIO_BACKLIGHT_PIN GPIO_NUM_22     // 백라이트 핀 
    // 태스크 핸들러
    TaskHandle_t xMicButtonTaskHandle = NULL;
    TaskHandle_t xLoopTaskHandle = NULL;

    // GPIO 초기화 함수
    void initGPIO()
    {
        // GPIO 5번 스피커 설정 (출력, 풀다운)
        gpio_config_t io_conf_speaker = {
            .pin_bit_mask = (1ULL << GPIO_SPEAKER_PIN),  // 설정할 GPIO 핀 비트 마스크 (스피커)
            .mode = GPIO_MODE_OUTPUT,                    // GPIO 모드: 출력
            .pull_up_en = GPIO_PULLUP_DISABLE,           // 풀업 비활성화
            .pull_down_en = GPIO_PULLDOWN_ENABLE,        // 풀다운 활성화
            .intr_type = GPIO_INTR_DISABLE               // 인터럽트 비활성화
        };
        // GPIO 설정 적용
        gpio_config(&io_conf_speaker); // 스피커 핀 설정
        ESP_LOGI("GPIO", "GPIO 초기화 완료: 스피커(출력, 풀다운)");


        // GPIO 22번 백라이트 설정 (출력, 풀다운)
        gpio_config_t io_conf_backlight = {
            .pin_bit_mask = (1ULL << GPIO_BACKLIGHT_PIN),  // 설정할 GPIO 핀 비트 마스크 (스피커)
            .mode = GPIO_MODE_OUTPUT,                    // GPIO 모드: 출력
            .pull_up_en = GPIO_PULLUP_DISABLE,           // 풀업 비활성화
            .pull_down_en = GPIO_PULLDOWN_ENABLE,        // 풀다운 활성화
            .intr_type = GPIO_INTR_DISABLE               // 인터럽트 비활성화
        };
        // GPIO 설정 적용
        gpio_config(&io_conf_backlight); // 스피커 핀 설정
        ESP_LOGI("GPIO", "GPIO 초기화 완료: 백라이트(출력, 풀다운)");


        // GPIO 17번 마이크 버튼 설정 (입력)
        gpio_config_t io_conf_mic_button = {
            .pin_bit_mask = (1ULL << GPIO_MIC_BUTTON_PIN), // 마이크 버튼 핀 비트 마스크
            .mode = GPIO_MODE_INPUT,                       // GPIO 모드: 입력
            .pull_up_en = GPIO_PULLUP_DISABLE,             // 풀업 저항은 이미 하드웨어에 있음
            .pull_down_en = GPIO_PULLDOWN_DISABLE,         // 풀다운 비활성화
            .intr_type = GPIO_INTR_DISABLE                 // 인터럽트 비활성화
        };
        gpio_config(&io_conf_mic_button); // 마이크 버튼 핀 설정
        ESP_LOGI(TAG, "GPIO 초기화 완료: 마이크 버튼(입력)");
        
    }
    void loopTask(void *pvParameters) {
        while (1) {
            loopBLE();  // BLE 관련 루프 실행
          //  loopLCD();  // LCD 관련 루프 실행
            vTaskDelay(pdMS_TO_TICKS(100));  // 1초 대기 (1000ms)
        }
    }
    // 마이크 버튼 상태를 감시하는 Task
    void micButtonMonitorTask(void *arg)
    {
        int last_button_state = 1; // 초기 상태는 버튼이 눌리지 않은 상태로 가정 (1)
        
        while (1) {
            int current_button_state = gpio_get_level(GPIO_MIC_BUTTON_PIN);
            
            if (current_button_state == 0 && last_button_state == 1) {
                // 버튼이 눌려서 Low로 바뀌는 순간 (상태 변화 감지)
                ESP_LOGI(TAG, "마이크 버튼 눌림 - 쿠폰 지급");
                sendBLEMessage("/askMic");
            }
            
            // 버튼이 다시 High 상태로 돌아오면 상태를 초기화하여 다시 감지 가능하게 함
            if (current_button_state == 1 && last_button_state == 0) {
                ESP_LOGI(TAG, "마이크 버튼이 다시 올라감");
            }

            // 이전 버튼 상태 업데이트
            last_button_state = current_button_state;

            vTaskDelay(100 / portTICK_PERIOD_MS);  // 100ms 대기
        }
    }

    // 마이크 버튼 감시 Task 시작
    void startMicButtonMonitorTask()
    {
        if (xMicButtonTaskHandle == NULL) {
            xTaskCreate(micButtonMonitorTask, "micButtonMonitorTask", 2048, NULL, 5, &xMicButtonTaskHandle);
            ESP_LOGI(TAG, "마이크 버튼 감시 Task 시작");
        }
    }

    // 마이크 버튼 감시 Task 종료
    void stopMicButtonMonitorTask()
    {
        if (xMicButtonTaskHandle != NULL) {
            vTaskDelete(xMicButtonTaskHandle);
            xMicButtonTaskHandle = NULL;
            ESP_LOGI(TAG, "마이크 버튼 감시 Task 종료");
        }
    }
    // 루프 태스크를 시작하는 함수
    void startLoopTask() {
        if (xLoopTaskHandle == NULL) {
            xTaskCreate(loopTask, "LoopTask", 2048, NULL, 5, &xLoopTaskHandle);
            ESP_LOGI(TAG, "루프 태스크 시작됨");
        }
    }

    // 루프 태스크를 중지하는 함수
    void stopLoopTask() {
        if (xLoopTaskHandle != NULL) {
            vTaskDelete(xLoopTaskHandle);
            xLoopTaskHandle = NULL;
            ESP_LOGI(TAG, "루프 태스크 중지됨");
        }
    }


}
// 스피커 상태 설정 함수 (ESP)
void setSpeakerOn(bool b)
{
    if (b)
    {
        gpio_set_level(GPIO_SPEAKER_PIN, 1);  // 스피커 핀을 HIGH로 설정
        ESP_LOGI("GPIO", "스피커 ON (HIGH)");
    }
    else
    {
        gpio_set_level(GPIO_SPEAKER_PIN, 0);  // 스피커 핀을 LOW로 설정
        ESP_LOGI("GPIO", "스피커 OFF (LOW)");
    }
}
//ADF Functions
extern "C"
{   
    static void hfp_event_handler(esp_hf_client_cb_event_t event, esp_hf_client_cb_param_t *param);
    static void bt_app_hf_client_audio_open(hfp_data_enc_type_t type);
    static void bt_app_hf_client_audio_close(void);
    i2s_stream_cfg_t create_i2s_stream_cfg(audio_stream_type_t stream_type, i2s_port_t port, uint32_t rate, i2s_bits_per_sample_t bits) {
        i2s_stream_cfg_t cfg = {
            .type = stream_type,
            .i2s_port = port,
            .expand_src_bits = I2S_BITS_PER_SAMPLE_16BIT,
            .i2s_config = {
                .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX),
                .sample_rate = rate,
                .bits_per_sample = bits,
                .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
                .communication_format = I2S_COMM_FORMAT_STAND_I2S,
                .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2 | ESP_INTR_FLAG_IRAM,
                .dma_buf_count = 3,
                .dma_buf_len = 300,
                .use_apll = true,
                .tx_desc_auto_clear = true,
                .fixed_mclk = 0
            },
            .use_alc = true,
            .volume = 200,
            .out_rb_size = I2S_STREAM_RINGBUFFER_SIZE,
            .task_stack = I2S_STREAM_TASK_STACK,
            .task_core = I2S_STREAM_TASK_CORE,
            .task_prio = I2S_STREAM_TASK_PRIO,
            .stack_in_ext = false,
            .multi_out_num = 0,
            .uninstall_drv = true,
            .need_expand = false,
            .buffer_len = I2S_STREAM_BUF_SIZE,
        };
        return cfg;
    }
    void app_main(void);
}

// //BLE 
// void loop(){
//     loopBLE();  // Run BLE loop
//     loopLCD();  // Run BLE loop
//     ESP_LOGI(TAG, "[ 1 ] loop TEST");
// }
// 실제 루프 태스크 (while 문을 사용하는 태스크)

//ADF HFP 
static void hfp_event_handler(esp_hf_client_cb_event_t event, esp_hf_client_cb_param_t *param) {
    switch (event) {
        case ESP_HF_CLIENT_CONNECTION_STATE_EVT:
            ESP_LOGI(TAG, "HFP Client connection state changed: %d", param->conn_stat.state);  
            if (param->conn_stat.state == ESP_HF_CLIENT_CONNECTION_STATE_CONNECTED) {
                ESP_LOGI(TAG, "HFP Client is connected");
                setScreen(SCREEN_CONNECTED, 10);
            } else if (param->conn_stat.state == ESP_HF_CLIENT_CONNECTION_STATE_DISCONNECTED) {
                ESP_LOGI(TAG, "HFP Client is disconnected");
                setScreen(SCREEN_DISCONNECTED, 10);
            }
            break;
        case ESP_HF_CLIENT_AUDIO_STATE_EVT:
            ESP_LOGI(TAG, "HFP Client audio state changed: %d", param->audio_stat.state);
            break;
        case ESP_HF_CLIENT_BVRA_EVT:
            ESP_LOGI(TAG, "Voice recognition state changed: %d", param->bvra.value);
            break;
        default:
            ESP_LOGI(TAG, "Unhandled HFP event: %d", event);
            break;
    }
}

void set_max_volume(void){
    esp_err_t ret = esp_hf_client_volume_update(ESP_HF_VOLUME_CONTROL_TARGET_MIC, 15);
    if (ret == ESP_OK) {
        printf("게인 볼륨 업데이트 성공 \n");
    } else {
        printf("게인 볼륨 업데이트 실패2, Error code: %d\n", ret);
        // 추가적인 오류 처리 로직을 여기에 추가할 수 있습니다.
    }
    ret = esp_hf_client_volume_update(ESP_HF_VOLUME_CONTROL_TARGET_SPK, 30);
    if (ret == ESP_OK) {
        printf("게인 볼륨 업데이트 성공 \n");
    } else {
        printf("게인 볼륨 업데이트 실패, Error code: %d\n", ret);
        // 추가적인 오류 처리 로직을 여기에 추가할 수 있습니다.
    }
    // // ALC 볼륨 설정
    // ret = i2s_alc_volume_set(i2s_stream_reader, 64);
    // if (ret == ESP_OK) {
    //     printf("ALC 볼륨 설정 성공\n");
    // } else {
    //     printf("ALC 볼륨 설정 실패, 오류 코드: %d\n", ret);
    // }
    ESP_LOGI(TAG, "[ 2.1 ] audio hal 볼륨조절");
    int volume = 85;  // 0-100 범위, 여기서는 최대 값 100으로 설정
    ret = audio_hal_set_volume(board_handle->audio_hal, volume);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "볼륨 설정 실패: %d", ret);
    } else {
        int current_volume = 0;
        ret = audio_hal_get_volume(board_handle->audio_hal, &current_volume);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "볼륨 가져오기 실패: %d", ret);
        } else {
            ESP_LOGI(TAG, "볼륨 설정 성공: %d (현재 볼륨: %d)", ret, current_volume);
        }
    }
}

// HFP 오디오 스트림을 여는 함수
static void bt_app_hf_client_audio_open(hfp_data_enc_type_t type)
{
    ESP_LOGI(TAG, "HFP 오디오 열기 타입 = %d", type);
    
    setSpeakerOn(true);
    set_max_volume();
}

// 이 함수는 HFP 오디오 스트림을 닫습니다. 연결이 끊어졌을 때 호출됩니다.
static void bt_app_hf_client_audio_close(void)
{
    setSpeakerOn(false);
    ESP_LOGI(TAG, "HFP 오디오 닫기");
}

void setup(){
    Serial.begin(115200);
    initBLE();
    initLCD();
    Serial.println("GPIO 초기화 완료: 마이크(입력, 풀업 사용)");
}
void app_main(void)
{
    // NVS(Non-Volatile Storage)를 초기화합니다.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // 로그 레벨을 설정합니다.
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    ESP_LOGI(TAG, "[ 1 ] Bluetooth 초기화");
    // ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    // BLE 메모리를 해제합니다.
   // ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    // Bluetooth 컨트롤러를 초기화합니다.
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if (esp_bt_controller_init(&bt_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "컨트롤러 초기화 실패");
    }

    // Bluetooth 클래식 모드를 활성화합니다.
    if (esp_bt_controller_enable(ESP_BT_MODE_BTDM) != ESP_OK) {
        ESP_LOGE(TAG, "컨트롤러 활성화 실패");
    }

    // 블루드로이드(Bluedroid) 스택을 초기화합니다.
    if (esp_bluedroid_init() != ESP_OK) {
        ESP_LOGE(TAG, "블루드로이드 초기화 실패");
    }

    // 블루드로이드 스택을 활성화합니다.
    if (esp_bluedroid_enable() != ESP_OK) {
        ESP_LOGE(TAG, "블루드로이드 활성화 실패");
    }

    // Bluetooth 장치 이름을 설정합니다.
    esp_bt_dev_set_device_name(FULL_DEVICE_NAME);
    
    // HFP 오디오 스트림 열기 및 닫기 이벤트 콜백을 등록합니다.
    hfp_open_and_close_evt_cb_register(bt_app_hf_client_audio_open, bt_app_hf_client_audio_close);

    // HFP 서비스를 초기화합니다.
    hfp_service_init();

    // HFP 이벤트 핸들러 등록
    ESP_LOGI(TAG, "[ 1 ] HFP 이벤트 핸들러 등록");
  //  esp_hf_client_register_callback(hfp_event_handler);

#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0))
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
#else
    esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
#endif

    ESP_LOGI(TAG, "[ 2 ] 코덱 칩 시작");
    board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);

    ESP_LOGI(TAG, "[ 3 ] 재생을 위한 오디오 파이프라인 생성");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline_in = audio_pipeline_init(&pipeline_cfg);
    pipeline_out = audio_pipeline_init(&pipeline_cfg);

    ESP_LOGI(TAG, "[3.1] 코덱 칩에 데이터를 쓰고 코덱 칩에서 데이터를 읽기 위한 i2s 스트림 생성");
    i2s_stream_cfg_t i2s_cfg1 = create_i2s_stream_cfg(AUDIO_STREAM_READER, I2S_NUM_0, 44100, I2S_BITS_PER_SAMPLE_16BIT);

    #if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
        i2s_cfg1.chan_cfg.id = CODEC_ADC_I2S_PORT;
        i2s_cfg1.std_cfg.clk_cfg.sample_rate_hz = g_hfp_audio_rate;
        i2s_cfg1.std_cfg.slot_cfg.slot_mode = I2S_SLOT_MODE_MONO;
    #if defined(CONFIG_ESP_LYRAT_MINI_V1_1_BOARD) || defined(CONFIG_ESP_LYRATD_MSC_V2_1_BOARD) || defined(CONFIG_ESP_LYRATD_MSC_V2_2_BOARD)
        i2s_cfg1.std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
    #else
        i2s_cfg1.std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_RIGHT;
    #endif
    #else
        i2s_cfg1.i2s_config.sample_rate = g_hfp_audio_rate;
    #if defined(CONFIG_ESP_LYRAT_MINI_V1_1_BOARD) || defined(CONFIG_ESP_LYRATD_MSC_V2_1_BOARD) || defined(CONFIG_ESP_LYRATD_MSC_V2_2_BOARD)
        i2s_cfg1.i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    #else
//        i2s_cfg1.i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT;
    #endif
    #endif // (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
    //i2s_cfg1.out_rb_size = 8 * 1024; // 링 버퍼 크기 설정

    i2s_stream_cfg_t i2s_cfg2 = create_i2s_stream_cfg(AUDIO_STREAM_WRITER, I2S_NUM_0, 44100, I2S_BITS_PER_SAMPLE_16BIT);
    #if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
    i2s_cfg2.std_cfg.slot_cfg.slot_mode = I2S_SLOT_MODE_MONO;
    i2s_cfg2.std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
    i2s_cfg2.std_cfg.clk_cfg.sample_rate_hz = g_hfp_audio_rate;
    #else
    i2s_cfg2.i2s_config.sample_rate = g_hfp_audio_rate;
 //   i2s_cfg2.i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT;
    #endif
 //   i2s_cfg2.out_rb_size = 8 * 1024; // 링 버퍼 크기 설정
    i2s_stream_reader = i2s_stream_init(&i2s_cfg1);
    i2s_stream_writer = i2s_stream_init(&i2s_cfg2);

    ESP_LOGI(TAG, "[3.2] HFP 스트림 가져오기");
    hfp_stream_config_t hfp_config;
    hfp_config.type = OUTGOING_STREAM;
    hfp_out_stream = hfp_stream_init(&hfp_config);
    hfp_config.type = INCOMING_STREAM;
    hfp_in_stream = hfp_stream_init(&hfp_config);

    ESP_LOGI(TAG, "[3.3] 모든 요소를 오디오 파이프라인에 등록하기");
    audio_pipeline_register(pipeline_in, hfp_in_stream, "incoming");
    audio_pipeline_register(pipeline_in, i2s_stream_writer, "i2s_writer");
    audio_pipeline_register(pipeline_out, i2s_stream_reader, "i2s_reader");
    audio_pipeline_register(pipeline_out, hfp_out_stream, "outgoing");

    ESP_LOGI(TAG, "[3.4] 오디오 파이프라인 요소를 연결하기 [Bluetooth]-->hfp_in_stream-->i2s_stream_writer-->[codec_chip]");
    const char *link_in[2] = {"incoming", "i2s_writer"};
    audio_pipeline_link(pipeline_in, &link_in[0], 2);
    const char *link_out[2] = {"i2s_reader", "outgoing"};
    audio_pipeline_link(pipeline_out, &link_out[0], 2);

    // 여기에 링 버퍼 설정 코드를 추가합니다
    audio_element_set_output_ringbuf(hfp_in_stream, audio_element_get_input_ringbuf(i2s_stream_writer));
    audio_element_set_output_ringbuf(i2s_stream_reader, audio_element_get_input_ringbuf(hfp_out_stream));


    ESP_LOGI(TAG, "[ 4 ] 주변 장치 초기화");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

    ESP_LOGI(TAG, "[ 5 ] 이벤트 리스너 설정");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

    ESP_LOGI(TAG, "[5.1] 파이프라인의 모든 요소에서 이벤트 수신 대기");
    audio_pipeline_set_listener(pipeline_in, evt);
    audio_pipeline_set_listener(pipeline_out, evt);

    ESP_LOGI(TAG, "[5.2] 주변 장치에서 이벤트 수신 대기");
    audio_event_iface_set_listener(esp_periph_set_get_event_iface(set), evt);


    ESP_LOGI(TAG, "[ 6 ] 모든 파이프라인 이벤트 수신 대기");
    g_hfp_audio_rate = 16000;
    i2s_stream_set_clk(i2s_stream_reader, g_hfp_audio_rate, 16, 1);
    i2s_stream_set_clk(i2s_stream_writer, g_hfp_audio_rate, 16, 1);

    // 파이프라인을 실행합니다.
    ESP_LOGI(TAG, "입력 파이프라인 실행");
    audio_pipeline_run(pipeline_in);
    ESP_LOGI(TAG, "출력 파이프라인 실행");
    audio_pipeline_run(pipeline_out);
    
    set_max_volume();

    // 명령어 직접 실행 및 예외 처리
    esp_err_t ret = esp_console_run("vu 0 15", NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to execute command: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Command executed successfully");
    }

    initGPIO();    
  //  xTaskCreate(gpio_task_btnDown, "gpio_task_btnDown", 2048, NULL, 10, NULL); // 이벤트 처리 태스크 생성

   //스피커 조절
    setSpeakerOn(false);
    startMicButtonMonitorTask();
    startLoopTask();

    setup();
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    while (1) {
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[ * ] 이벤트 인터페이스 오류 : %d", ret);
            continue;
        }
        /* 마지막 파이프라인 요소(i2s_stream_writer)가 중지 이벤트를 수신할 때 중지 */
        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *) i2s_stream_writer
            && msg.cmd == AEL_MSG_CMD_REPORT_STATUS && (int) msg.data == AEL_STATUS_STATE_STOPPED) {
            ESP_LOGW(TAG, "[ * ] 중지 이벤트 수신");
            break;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);  // 10ms 대기
    }

    ESP_LOGI(TAG, "[ 7 ] 오디오 파이프라인 중지");
    audio_pipeline_stop(pipeline_in);
    audio_pipeline_wait_for_stop(pipeline_in);
    audio_pipeline_terminate(pipeline_in);
    audio_pipeline_stop(pipeline_out);
    audio_pipeline_wait_for_stop(pipeline_out);
    audio_pipeline_terminate(pipeline_out);

    audio_pipeline_unregister(pipeline_in, hfp_in_stream);
    audio_pipeline_unregister(pipeline_in, i2s_stream_writer);
    audio_pipeline_unregister(pipeline_out, i2s_stream_reader);
    audio_pipeline_unregister(pipeline_out, hfp_out_stream);

    /* 리스너를 제거하기 전에 파이프라인을 종료합니다 */
    audio_pipeline_remove_listener(pipeline_in);
    audio_pipeline_remove_listener(pipeline_out);

    /* 리스너를 제거하기 전에 모든 주변 장치를 중지합니다 */
    esp_periph_set_stop_all(set);
    audio_event_iface_remove_listener(esp_periph_set_get_event_iface(set), evt);

    /* audio_pipeline_remove_listener 및 audio_event_iface_remove_listener가 호출된 후 이벤트 인터페이스를 제거합니다 */
    audio_event_iface_destroy(evt);

    /* 모든 리소스를 해제합니다 */
    audio_pipeline_deinit(pipeline_in);
    audio_pipeline_deinit(pipeline_out);
    audio_element_deinit(hfp_in_stream);
    audio_element_deinit(i2s_stream_writer);
    audio_element_deinit(i2s_stream_reader);
    audio_element_deinit(hfp_out_stream);
    esp_periph_set_destroy(set);

    // 내 함수 해제
    stopLoopTask();
    stopMicButtonMonitorTask();
}
