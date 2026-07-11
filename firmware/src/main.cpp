#include <CAN/CAN_Frames.h>
#include <stm32f0xx_hal.h>

#include <cstdint>

namespace {

constexpr uint32_t FOINSE_STATUS_MAGIC = 0x464F494EU; // FOIN
constexpr uint32_t FOINSE_STATUS_VERSION = 5U;
constexpr uint8_t ADC_AVERAGE_SAMPLES = 8U;
constexpr uint32_t ADC_REFERENCE_MV = 3300U;
constexpr int32_t ACS71240_ZERO_MV = 1650;
constexpr int32_t ACS71240_SENSITIVITY_MV_PER_A = 132;
constexpr uint32_t CAN_HEARTBEAT_PERIOD_MS = 1000U;
constexpr uint32_t CAN_POWER_PERIOD_MS = 200U;
constexpr uint32_t CAN_BUS_RECOVERY_PERIOD_MS = 250U;
constexpr uint8_t CAN_TX_QUEUE_LEN = 8U;
constexpr uint8_t CAN_TX_DRAIN_BUDGET = 3U;
constexpr uint8_t POWER_MAIN_FLAG_CURRENT_VALID = 0x01U;
constexpr uint8_t POWER_SERVO_FLAG_CURRENT_VALID = 0x02U;

ADC_HandleTypeDef hadc{};
CAN_HandleTypeDef hcan{};
IWDG_HandleTypeDef hiwdg{};

struct FoinseStatus {
    uint32_t magic;
    uint32_t version;
    uint32_t uptime_ms;
    uint32_t loop_count;
    uint32_t adc_ok;
    uint32_t fault;
    uint32_t sense1_raw;
    uint32_t sense2_raw;
    uint32_t sense1_mv;
    uint32_t sense2_mv;
    int32_t sense1_current_ma;
    int32_t sense2_current_ma;
    uint32_t clock_hz;
    uint32_t clock_source;
    uint32_t can_init_ok;
    uint32_t can_bus_off;
    uint32_t can_error;
    uint32_t can_tx_count;
    uint32_t can_rx_count;
    uint32_t can_tx_drops;
    uint32_t can_tx_queue_depth;
    uint32_t heartbeat_tx_count;
    uint32_t power_tx_count;
    uint32_t can_esr;
    uint32_t sense1_valid;
    uint32_t sense2_valid;
    uint32_t adc_error_count;
    uint32_t watchdog_init_ok;
    uint32_t watchdog_refresh_count;
    uint32_t reset_flags;
};
static_assert(sizeof(FoinseStatus) == 120U, "FoinseStatus wire contract changed");

bool can_ready = false;
uint32_t can_tx_count = 0U;
uint32_t can_rx_count = 0U;
uint32_t can_tx_drops = 0U;
uint32_t heartbeat_tx_count = 0U;
uint32_t power_tx_count = 0U;
uint32_t last_heartbeat_ms = 0U;
uint32_t last_power_ms = 0U;
uint32_t last_bus_recovery_ms = 0U;
CAN_Frame tx_queue[CAN_TX_QUEUE_LEN]{};
uint8_t tx_head = 0U;
uint8_t tx_tail = 0U;
uint8_t tx_count = 0U;
bool sense1_valid = false;
bool sense2_valid = false;
uint32_t adc_error_count = 0U;

bool init_watchdog()
{
#if defined(__HAL_DBGMCU_FREEZE_IWDG)
    __HAL_DBGMCU_FREEZE_IWDG();
#endif
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload = 2499U;
    hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
    return HAL_IWDG_Init(&hiwdg) == HAL_OK;
}

} // namespace

extern "C" {

struct OgmaBoardIdentity {
    uint32_t magic;
    uint16_t schema_version;
    uint16_t struct_size;
    uint32_t board_id;
    uint32_t capabilities;
    uint32_t firmware_version;
    uint32_t firmware_build;
    uint32_t reserved0;
    uint32_t reserved1;
};

__attribute__((used)) volatile OgmaBoardIdentity ogma_board_identity{
    0x4F474944U,
    1U,
    sizeof(OgmaBoardIdentity),
    0x06U,
    0x21U,
    20260707U,
    0U,
    0U,
    0U,
};

volatile FoinseStatus foinse_status{};

}

void SystemClock_Config();
void Error_Handler();
bool MX_CAN_Init();

extern "C" void SysTick_Handler()
{
    HAL_IncTick();
}

uint32_t raw_to_mv(uint32_t raw)
{
    return (raw * ADC_REFERENCE_MV) / 4095U;
}

int32_t mv_to_current_ma(uint32_t mv)
{
    return ((static_cast<int32_t>(mv) - ACS71240_ZERO_MV) * 1000) / ACS71240_SENSITIVITY_MV_PER_A;
}

bool can_bus_off()
{
    return hcan.Instance != nullptr && (hcan.Instance->ESR & CAN_ESR_BOFF) != 0U;
}

uint32_t can_error()
{
    return hcan.Instance == nullptr ? HAL_CAN_ERROR_PARAM : HAL_CAN_GetError(&hcan);
}

bool can_send_now(CAN_Frame& frame)
{
    if (!can_ready || can_bus_off() || frame.id > 0x7FFU || frame.dlc > 8U) {
        return false;
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U) {
        return false;
    }

    CAN_TxHeaderTypeDef header{};
    header.StdId = frame.id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = frame.dlc;
    header.TransmitGlobalTime = DISABLE;

    uint32_t mailbox = 0U;
    return HAL_CAN_AddTxMessage(&hcan, &header, frame.data, &mailbox) == HAL_OK;
}

bool queue_can_frame(const CAN_Frame& frame)
{
    if (tx_count >= CAN_TX_QUEUE_LEN) {
        tx_head = static_cast<uint8_t>((tx_head + 1U) % CAN_TX_QUEUE_LEN);
        --tx_count;
        ++can_tx_drops;
    }

    tx_queue[tx_tail] = frame;
    tx_tail = static_cast<uint8_t>((tx_tail + 1U) % CAN_TX_QUEUE_LEN);
    ++tx_count;
    return true;
}

bool send_can_frame(CAN_Frame& frame)
{
    if (!can_ready) {
        return false;
    }
    if (tx_count != 0U) {
        return queue_can_frame(frame);
    }
    if (can_send_now(frame)) {
        ++can_tx_count;
        return true;
    }
    return queue_can_frame(frame);
}

void flush_can_queue()
{
    if (!can_ready) {
        return;
    }

    for (uint8_t sent = 0U; sent < CAN_TX_DRAIN_BUDGET && tx_count > 0U; ++sent) {
        CAN_Frame& frame = tx_queue[tx_head];
        if (!can_send_now(frame)) {
            return;
        }
        tx_head = static_cast<uint8_t>((tx_head + 1U) % CAN_TX_QUEUE_LEN);
        --tx_count;
        ++can_tx_count;
    }
}

void service_can_rx()
{
    if (!can_ready) {
        return;
    }

    if (__HAL_CAN_GET_FLAG(&hcan, CAN_FLAG_FOV0) != RESET) {
        __HAL_CAN_CLEAR_FLAG(&hcan, CAN_FLAG_FOV0);
    }

    while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0U) {
        CAN_RxHeaderTypeDef header{};
        uint8_t data[8]{};
        if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
            return;
        }
        ++can_rx_count;
    }
}

void service_can_bus(uint32_t now_ms)
{
    if (!can_ready || !can_bus_off()) {
        return;
    }
    if ((now_ms - last_bus_recovery_ms) < CAN_BUS_RECOVERY_PERIOD_MS) {
        return;
    }
    (void)HAL_CAN_Stop(&hcan);
    HAL_CAN_ResetError(&hcan);
    (void)HAL_CAN_Start(&hcan);
    last_bus_recovery_ms = now_ms;
}

void send_heartbeat(uint32_t now_ms)
{
    uint8_t err = 0U;
    if (can_bus_off()) {
        err |= CAN_HEARTBEAT_ERR_BUS_OFF;
    }
    if (can_error() != 0U) {
        err |= CAN_HEARTBEAT_ERR_CAN_ERROR;
    }
    if (can_tx_drops != 0U) {
        err |= CAN_HEARTBEAT_ERR_TX_DROP;
    }

    HEARTBEAT_Payload payload{
        static_cast<uint8_t>(NODE_FOINSE),
        0U,
        err,
        static_cast<uint8_t>((now_ms / 1000U) & 0xFFU),
    };
    CAN_Frame frame = pack_frame(CAN_ID_HEARTBEAT_NODE(NODE_FOINSE), payload);
    if (send_can_frame(frame)) {
        ++heartbeat_tx_count;
    }
}

uint16_t clamp_u16_ma(int32_t value)
{
    if (value <= 0) {
        return 0U;
    }
    if (value > 65535) {
        return 65535U;
    }
    return static_cast<uint16_t>(value);
}

void send_power_status()
{
    POWER_MAIN_Payload main_payload{
        0U,
        clamp_u16_ma(foinse_status.sense1_current_ma),
        0U,
        static_cast<uint8_t>(sense1_valid ? POWER_MAIN_FLAG_CURRENT_VALID : 0U),
        0U,
    };
    CAN_Frame main_frame = pack_frame(CAN_ID_POWER_MAIN, main_payload);
    bool sent_any = send_can_frame(main_frame);

    POWER_SERVO_Payload servo_payload{
        0U,
        clamp_u16_ma(foinse_status.sense2_current_ma),
        0,
        static_cast<uint8_t>(sense2_valid ? POWER_SERVO_FLAG_CURRENT_VALID : 0U),
        0U,
    };
    CAN_Frame servo_frame = pack_frame(CAN_ID_POWER_SERVO, servo_payload);
    sent_any = send_can_frame(servo_frame) || sent_any;
    if (sent_any) {
        ++power_tx_count;
    }
}

void update_can_status()
{
    foinse_status.clock_hz = SystemCoreClock;
    foinse_status.can_init_ok = can_ready ? 1U : 0U;
    foinse_status.can_bus_off = can_ready && can_bus_off() ? 1U : 0U;
    foinse_status.can_error = can_ready ? can_error() : 0U;
    foinse_status.can_tx_count = can_tx_count;
    foinse_status.can_rx_count = can_rx_count;
    foinse_status.can_tx_drops = can_tx_drops;
    foinse_status.can_tx_queue_depth = tx_count;
    foinse_status.heartbeat_tx_count = heartbeat_tx_count;
    foinse_status.power_tx_count = power_tx_count;
    foinse_status.can_esr = hcan.Instance != nullptr ? hcan.Instance->ESR : 0U;
    foinse_status.sense1_valid = sense1_valid ? 1U : 0U;
    foinse_status.sense2_valid = sense2_valid ? 1U : 0U;
    foinse_status.adc_error_count = adc_error_count;
}

void MX_GPIO_Init()
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio{};
    gpio.Pin = GPIO_PIN_3 | GPIO_PIN_4;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
}

void MX_ADC1_Init()
{
    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc.Instance = ADC1;
    hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc.Init.Resolution = ADC_RESOLUTION_12B;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
    hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc.Init.LowPowerAutoWait = DISABLE;
    hadc.Init.LowPowerAutoPowerOff = DISABLE;
    hadc.Init.ContinuousConvMode = DISABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.DMAContinuousRequests = DISABLE;
    hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;

    if (HAL_ADC_Init(&hadc) != HAL_OK) {
        foinse_status.fault = 1U;
        Error_Handler();
    }

    if (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK) {
        foinse_status.fault = 2U;
        Error_Handler();
    }
}

bool read_adc_channel(uint32_t channel, uint32_t* value)
{
    if (value == nullptr) {
        return false;
    }
    ADC1->CHSELR = 0U;

    ADC_ChannelConfTypeDef config{};
    config.Channel = channel;
    config.Rank = ADC_RANK_CHANNEL_NUMBER;
    config.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;

    if (HAL_ADC_ConfigChannel(&hadc, &config) != HAL_OK) {
        foinse_status.adc_ok = 0U;
        foinse_status.fault = 3U;
        ++adc_error_count;
        return false;
    }

    if (HAL_ADC_Start(&hadc) != HAL_OK) {
        foinse_status.adc_ok = 0U;
        foinse_status.fault = 4U;
        ++adc_error_count;
        return false;
    }

    if (HAL_ADC_PollForConversion(&hadc, 10U) != HAL_OK) {
        (void)HAL_ADC_Stop(&hadc);
        foinse_status.adc_ok = 0U;
        foinse_status.fault = 5U;
        ++adc_error_count;
        return false;
    }

    *value = HAL_ADC_GetValue(&hadc);
    (void)HAL_ADC_Stop(&hadc);
    return true;
}

bool read_adc_average(uint32_t channel, uint32_t* average)
{
    if (average == nullptr) {
        return false;
    }
    uint32_t sum = 0U;
    for (uint8_t sample = 0U; sample < ADC_AVERAGE_SAMPLES; ++sample) {
        uint32_t value = 0U;
        if (!read_adc_channel(channel, &value)) {
            return false;
        }
        sum += value;
    }
    *average = (sum + (ADC_AVERAGE_SAMPLES / 2U)) / ADC_AVERAGE_SAMPLES;
    return true;
}

int main()
{
    (void)ogma_board_identity.magic;
    foinse_status.magic = FOINSE_STATUS_MAGIC;
    foinse_status.version = FOINSE_STATUS_VERSION;
    foinse_status.reset_flags = RCC->CSR;
    __HAL_RCC_CLEAR_RESET_FLAGS();

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    can_ready = MX_CAN_Init() && HAL_CAN_Start(&hcan) == HAL_OK;
    foinse_status.watchdog_init_ok = init_watchdog() ? 1U : 0U;
    if (foinse_status.watchdog_init_ok == 0U) {
        Error_Handler();
    }

    last_heartbeat_ms = HAL_GetTick();
    last_power_ms = last_heartbeat_ms;
    last_bus_recovery_ms = last_heartbeat_ms;

    while (true) {
        foinse_status.uptime_ms = HAL_GetTick();
        foinse_status.loop_count++;

        uint32_t sense1_raw = 0U;
        uint32_t sense2_raw = 0U;
        sense1_valid = read_adc_average(ADC_CHANNEL_3, &sense1_raw);
        sense2_valid = read_adc_average(ADC_CHANNEL_4, &sense2_raw);
        foinse_status.adc_ok = sense1_valid && sense2_valid ? 1U : 0U;
        if (sense1_valid) {
            foinse_status.sense1_raw = sense1_raw;
            foinse_status.sense1_mv = raw_to_mv(sense1_raw);
            foinse_status.sense1_current_ma = mv_to_current_ma(foinse_status.sense1_mv);
        }
        if (sense2_valid) {
            foinse_status.sense2_raw = sense2_raw;
            foinse_status.sense2_mv = raw_to_mv(sense2_raw);
            foinse_status.sense2_current_ma = mv_to_current_ma(foinse_status.sense2_mv);
        }

        const uint32_t now_ms = foinse_status.uptime_ms;
        service_can_bus(now_ms);
        flush_can_queue();
        service_can_rx();
        if ((now_ms - last_power_ms) >= CAN_POWER_PERIOD_MS) {
            send_power_status();
            last_power_ms = now_ms;
        }
        if ((now_ms - last_heartbeat_ms) >= CAN_HEARTBEAT_PERIOD_MS) {
            send_heartbeat(now_ms);
            last_heartbeat_ms = now_ms;
        }
        update_can_status();

        if (HAL_IWDG_Refresh(&hiwdg) == HAL_OK) {
            ++foinse_status.watchdog_refresh_count;
        }

        HAL_Delay(50U);
    }
}

bool configure_hse_48mhz()
{
    RCC_OscInitTypeDef osc{};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PREDIV = RCC_PREDIV_DIV1;
    osc.PLL.PLLMUL = RCC_PLL_MUL6;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        return false;
    }

    RCC_ClkInitTypeDef clk{};
    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) != HAL_OK) {
        return false;
    }

    SystemCoreClockUpdate();
    foinse_status.clock_source = 1U;
    return true;
}

bool configure_hsi48()
{
#if defined(RCC_OSCILLATORTYPE_HSI48)
    RCC_OscInitTypeDef osc{};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    osc.HSI48State = RCC_HSI48_ON;
    osc.PLL.PLLState = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        return false;
    }

    RCC_ClkInitTypeDef clk{};
    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) != HAL_OK) {
        return false;
    }

    SystemCoreClockUpdate();
    foinse_status.clock_source = 2U;
    return true;
#else
    return false;
#endif
}

void SystemClock_Config()
{
    if (configure_hse_48mhz() || configure_hsi48()) {
        return;
    }

    Error_Handler();
}

bool MX_CAN_Init()
{
    hcan.Instance = CAN;
    hcan.Init.Prescaler = 6U;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = ENABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = ENABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan) != HAL_OK) {
        return false;
    }

    CAN_FilterTypeDef filter{};
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;
    return HAL_CAN_ConfigFilter(&hcan, &filter) == HAL_OK;
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* handle)
{
    if (handle->Instance != CAN) {
        return;
    }

    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio{};
    gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF4_CAN;
    HAL_GPIO_Init(GPIOA, &gpio);
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* handle)
{
    if (handle->Instance != CAN) {
        return;
    }

    __HAL_RCC_CAN1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
}

void Error_Handler()
{
    __disable_irq();
    while (true) {}
}
