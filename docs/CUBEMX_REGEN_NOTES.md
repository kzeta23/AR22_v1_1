# CubeMX 재생성 후 수동 조치 (Post-Regeneration Checklist)

STM32CubeIDE/CubeMX로 `.ioc`를 다시 생성(Generate Code)하면, 생성기가 USER CODE 밖의
수동 변경을 되돌리거나 일부 코드를 다시 삽입한다. **재생성 직후 아래를 반드시 확인·수정한다.**

## 1. ⚠️ IWDG 이른 시작 제거 (부트 루프 방지) — 필수
CubeMX는 `main()`의 "Initialize all configured peripherals" 블록에 `MX_IWDG_Init();`를
자동 삽입한다(예: `MX_TIM5_Init();` 다음). 이 위치는 **POST/EEPROM 로드 전**이라,
워치독이 긴 부팅 구간(`init_display`의 스위치 대기, `power_on_selftest` ~3.5초, EEPROM 로드)
도중 ~4초에 리셋 → **POST 무한 반복("셀프테스트에서 멈춤")** 이 된다.

조치:
- 초기화 블록의 `MX_IWDG_Init();`를 **삭제**한다.
- 워치독은 메인 루프 직전(USER CODE BEGIN WHILE, EEPROM 로드 후)의 `MX_IWDG_Init();`로만
  시작한다. (킥은 0.2초 표시 블록에서만 호출 → "실제 진행 시에만" 갱신)

## 2. USER CODE 보존분 점검
대부분의 커스터마이즈는 USER CODE 구간이라 보존되지만, 재생성 후 다음이 살아있는지 확인:
- `_write()` → **USART1**(printf 경로, 차단 전송)
- 부저: **TIM5_CH2 PWM**(`buzzer_on/off`), 워치독 **0.2초 킥**
- GM 카운트 ISR 스왑: `gmCountLow <- htim8(PA0)`, `gmCountHigh <- htim1(PE7)`
- EEPROM 바이트 단위 R/W: 알람 5B@0, c/f 2B@26
- OLED 밝기: `SSD1322_API_set_contrast(0x80)`, 계조값(메인 15 / 알람 7 / 점멸 3)
- EEPROM 검증·기본값(`DEFAULT_ALARM_THRESHOLD`), 데드타임 분모 클램프
- ISR 공유 변수 `volatile`

## 3. 수동 추가 HAL 드라이버·설정 유지
다음이 .ioc/패키지 설정과 일치하는지 확인:
- 드라이버: `stm32f4xx_hal_i2c(.c/.h)`, `stm32f4xx_hal_i2c_ex(.c/.h)`, `stm32f4xx_hal_iwdg(.c/.h)`
- `stm32f4xx_hal_conf.h`: `HAL_I2C_MODULE_ENABLED`, `HAL_IWDG_MODULE_ENABLED`

## 4. 재생성 후 실기 확인 (필수)
빌드 → 플래싱 → **POST 통과 후 정상 동작**(USART1 연속 로그, 리셋/부트루프 없음) 확인.
GM 채널을 만진 경우 `CntL`(저선량/민감 튜브)·`CntH`(고선량 튜브)가 뒤바뀌지 않았는지 함께 확인.
