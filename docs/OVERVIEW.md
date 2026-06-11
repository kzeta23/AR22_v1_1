# AR22 LCU 펌웨어 기능 개요 (OVERVIEW)

> AR22 LCU(Local Control Unit) — 방사선(감마선) 검출·표시·경보 펌웨어
> 대상 MCU: **STM32F446VET6** (Cortex-M4 @100MHz, HSE 8MHz, FPU)

이 펌웨어는 이전 제품에서 검증된 코드를 **AR22 업그레이드 제품에 재사용**하는 것을 목표로 하며,
검증된 기존 동작을 최대한 보존한다. (작업 규칙: 루트의 `CLAUDE.md` 참조)

---

## 1. 제품 개요
2개의 GM(Geiger-Müller) 튜브로 **저선량·고선량 이중 레인지**를 측정하고, OLED에 선량을 표시하며,
설정한 임계값을 초과하면 부저·램프로 경보한다. 설정값(알람 임계값, 변환계수)은 EEPROM에 보존된다.

- **측정 범위: 0.1 µSv/h ~ 100 mSv/h** (시스템 사양)

## 2. 측정 체인 (핵심)
| 단계 | 내용 |
|---|---|
| 펄스 카운트 | 저선량 GM → TIM1_ETR(PE7), 고선량 GM → TIM8_ETR(PA0). 외부 펄스를 하드웨어 카운터로 누적 |
| 1초 샘플링 | TIM4 1ms 인터럽트 → 1초마다 카운터를 읽고 0으로 리셋 (`HAL_TIM_PeriodElapsedCallback`) |
| 적응형 EMA 필터 | `process_dose_ema()` — 지수이동평균. 변화가 크면 α↑(빠른 응답), 안정되면 α↓(노이즈 억제). 저/고 독립 |
| 선량 환산 | 저선량: `dose = filtered × conv_factor_low` (+ 데드타임 보정 `n/(1-nτ)`)<br>고선량: `dose = filtered × conv_factor_high` |
| 변환계수 | 저 ≈ 0.601 (LND7128), 고 ≈ 48 (LND71631) — EEPROM 저장·교정 가능 |

## 3. 이중 레인지 & 표시
- **전체 측정 범위**: **0.1 µSv/h ~ 100 mSv/h** (시스템 사양)
- **LOW 레인지**: µSv/h ~ 약 1 mSv/h
- **HIGH 레인지**: 1 ~ 100+ mSv/h, 150 mSv/h 초과 시 **"OVERLOAD"**
- **자동 전환**: 기준 1000 µSv/h, ±200 히스테리시스 (`process_range_check`)
- **NO COUNT FAIL**: 선량 < 0.05 상태가 300초 지속 시 고장 표시
- 디스플레이: SSD1322 **OLED 256×64** (SPI4 + DMA), 값 크기에 따라 폰트/단위 자동 전환(µSv/h ↔ mSv/h)

## 4. 경보 기능 (`process_alarm`)
- `alarmThreshold` (5자리, µSv/h) — EEPROM 저장
- 선량 ≥ 임계값 → **부저 ON + 적색 램프 ON**
- **ACK** 처리: 60초간 부저 음소거(램프 유지) 후 복귀
- 레인지별(저/고) 각각 판정

## 5. 사용자 인터페이스 (스위치 4개, EXTI)
| 스위치 | 핀 | 기능 |
|---|---|---|
| SET | PE12 | 알람설정 진입/저장, C/F 설정 저장 |
| SHIFT | PE11 | 설정 자리 이동 / C/F 저·고 전환 |
| UP / DOWN | PE10 / PE9 | 값 증가 / 감소 |

- **부팅 시 모드 진입**: SHIFT 누른 채 부팅 → 데이터 모니터 모드, SET 누른 채 부팅 → C/F 교정 모드
- 알람 임계값(5자리)·변환계수(저/고) 편집 후 EEPROM에 저장

## 6. 저장소 (EEPROM 24C01 / I2C)
- 용량 128바이트(0x00~0x7F), 8바이트 페이지, I2C 주소 0xA0
- 알람 데이터: 주소 **0~4**
- 변환계수: 주소 **26(저), 27(고)**
- 신뢰성을 위해 읽기/쓰기 2회 수행
- 드라이버: `MyLib/EEPROM_I2C/`

## 7. 통신 / 로그
- `printf` → **USART1 (115200 bps, STLink VCP)** 로 출력
- `data_monitor_ble()` — 1초마다 `L<선량>` / `H<선량>` 형식 송신
- USART2(9600, 구 블루투스)는 현재 미사용

## 8. 파워온 셀프테스트 (`power_on_selftest`)
부팅 시 1회 실행:
1. 디스플레이 전체 점등(죽은 픽셀/라인 확인)
2. LED1 + LED2 점멸(3회) → LAMP_RED 점멸(3회)
3. 부저 2회 비프(TIM5 PWM)
4. EEPROM 존재 확인 + 미사용 셀(0x7F)에 쓰기/읽기/검증 후 원복 → 화면에 `EEPROM OK/FAIL`
   (FAIL 시 긴 비프 3회 경고 후 계속 진행)

## 9. 메인 루프 구조
```
while(1):
  process_switch()        // 항상: 버튼 처리
  매 1.0초: process_alarm → process_range_check → process_dose_ema → data_monitor_ble
  매 0.2초: 화면 갱신 (정상 / 데이터모니터 / C-F 중 택1)
```

## 10. 주변장치 맵
| 기능 | 자원 | 핀 |
|---|---|---|
| 저선량 카운트 | TIM1_ETR | PE7 |
| 고선량 카운트 | TIM8_ETR | PA0 |
| 타임베이스 | TIM4 (1ms IT) | — |
| OLED | SPI4 + DMA2_Stream1 | PE2(SCL)/PE6(SI), PE4(CS)/PB9(DC)/PE0(RST) |
| EEPROM | I2C1 | PB6(SCL)/PB7(SDA), PB5(WP) |
| 부저 | TIM5_CH2 PWM (~3.7kHz) | PA1 |
| 디버그 로그 | USART1 | PA9(TX)/PA10(RX) |
| 스위치 | EXTI9_5 / EXTI15_10 | PE9~PE12 |
| LED / 램프 | GPIO | PA2(LED1), PA3(LED2), PD3(LAMP_RED) |
| (미사용) ETH SPI | SPI1 | PA5~PA7, PA4(CS) |

## 11. 빌드 / 플래싱
STM32CubeIDE 없이 ARM GNU Toolchain으로 빌드한다.
- VS Code: **Ctrl+Shift+B** = `Build + Flash` (`.vscode/tasks.json`)
- 스크립트: `tools/build.ps1`(빌드), `tools/flash.ps1`(플래싱, 다중 ST-Link 중 타깃 자동 선택)
- 산출물: `build/AR22_LCU_v1_1.elf/.hex/.bin/.map`
- 코어 옵션: Cortex-M4, FPU `fpv4-sp-d16`(hard), `-mthumb`, 정의 `STM32F446xx`/`USE_HAL_DRIVER`, 링커 `STM32F446VETX_FLASH.ld`

## 12. 변경 이력 (원본 베이스라인 대비)
- EEPROM **SPI(M95010) → I2C(24C01)** 마이그레이션
- `printf` 로그 출력 **USART2 → USART1**
- 부저 구동 **GPIO on/off → TIM5_CH2 PWM 톤**
- **파워온 셀프테스트** 추가 (디스플레이/LED/부저/EEPROM)
- `.ioc`에서 제거된 핀 정리: GM_SW_HI/LO, LAMP_GREEN, SW_ACK, PC6(TIM3)
