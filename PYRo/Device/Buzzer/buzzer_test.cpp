#include "buzzer_test.h"
#include "pyro_buzzer_player.h"
#include "pyro_buzzer_task.h"
#include "pyro_buzzer_notes.h"

// ============================================================
// 测试 1：初始化
// ============================================================
void test_buzzer_init(void) {
    BuzzerPlayer::getInstance().init();
}

// ============================================================
// 测试 2：阻塞式 Beep
// ============================================================
void test_buzzer_beep(void) {
    BuzzerPlayer::getInstance().beep(440, 100, 300);
}

// ============================================================
// 测试 3：音阶上行
// ============================================================
void test_buzzer_melody_scale(void) {
    BuzzerTask* task = new BuzzerTask();

    task->add_event(0,    BUZZER_GET_FREQ(BUZZER_NOTE_C4), 100);
    task->add_event(200,  BUZZER_GET_FREQ(BUZZER_NOTE_D4), 100);
    task->add_event(400,  BUZZER_GET_FREQ(BUZZER_NOTE_E4), 100);
    task->add_event(600,  BUZZER_GET_FREQ(BUZZER_NOTE_F4), 100);
    task->add_event(800,  BUZZER_GET_FREQ(BUZZER_NOTE_G4), 100);
    task->add_event(1000, BUZZER_GET_FREQ(BUZZER_NOTE_A4), 100);
    task->add_event(1200, BUZZER_GET_FREQ(BUZZER_NOTE_B4), 100);
    task->add_event(1400, BUZZER_GET_FREQ(BUZZER_NOTE_C5), 100);
    task->add_event(1600, 0, 0);
    task->set_duration(1600);

    BuzzerPlayer::getInstance().loadTask(task);
}

// ============================================================
// 测试 4：Bird 旋律（使用 RTTTL 解析）
// ============================================================
// void test_buzzer_melody_twinkle(void) {
//     const char* rtttl =
//         "Bird:d=4,o=5,b=100:"
//         "16g,16g,16a,16a,16e,16e,8g,"
//         "16g,16g,16a,16a,16e,16e,8g,"
//         "16g,16g,16a,16a,16c6,16c6,8b,"
//         "8b,8a,8g,8f,"
//         "16f,16f,16g,16g,16d,16d,8f,"
//         "16f,16f,16g,16g,16d,16d,8f,"
//         "16f,16f,16g,16g,16a,16b,8c6,"
//         "8a,8g,8e,c";

//     BuzzerTask* task = BuzzerTask::from_rtttl(rtttl);
//     if (task) {
//         BuzzerPlayer::getInstance().loadTask(task);
//     }
// }
void test_buzzer_melody_twinkle(void) {
    // 定义节奏：哔, 哔哔哔, 哔哔哔哔哔s
    uint8_t pattern[] = {1, 3, 5, 7};
    size_t len = sizeof(pattern) / sizeof(pattern[0]);
    uint32_t gap_ms = 200;  // 段间间隔 200ms

    BuzzerTask* task = BuzzerTask::from_rhythm(pattern, len, gap_ms, 440); // 频率 440Hz
    if (task) {
        BuzzerPlayer::getInstance().loadTask(task);
    }
}