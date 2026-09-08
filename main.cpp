#include <windows.h>
#include <conio.h>
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <ctime>

#include "rocket.h"
#include "pid_controller.h"
#include "reaction_wheel.h"

// GCC 6.x 구버전 대응: ENABLE_VIRTUAL_TERMINAL_PROCESSING 직접 정의
#ifndef ENABLE_VIRTUAL_TERMINAL_PROCESSING
#define ENABLE_VIRTUAL_TERMINAL_PROCESSING 0x0004
#endif

// ─── 콘솔 유틸 ──────────────────────────────────────────────────────
void enableANSI() {
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD mode = 0;
    GetConsoleMode(h, &mode);
    SetConsoleMode(h, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
}

void hideCursor() {
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    CONSOLE_CURSOR_INFO ci = { 1, FALSE };
    SetConsoleCursorInfo(h, &ci);
}

void moveTo(int x, int y) {
    COORD c = { (SHORT)x, (SHORT)y };
    SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), c);
}

// ─── 롤 인디케이터 (원 + 점) ─────────────────────────────────────────
void drawRollIndicator(int cx, int cy, double rollDeg) {
    const double PI = 3.14159265;
    const int    R  = 5;
    const double AX = 2.1;

    // 원 그리기
    for (int a = 0; a < 360; a += 12) {
        double rad = a * PI / 180.0;
        int x = cx + (int)(R * AX * cos(rad));
        int y = cy + (int)(R * sin(rad));
        moveTo(x, y);
        printf("\033[36m.\033[0m");
    }

    // 0도 기준 (위쪽)
    moveTo(cx, cy - R - 1);
    printf("\033[32m^\033[0m");

    // 롤 지시자 점
    double rad = (rollDeg - 90.0) * PI / 180.0;
    int dx = cx + (int)(R * AX * cos(rad));
    int dy = cy + (int)(R * sin(rad));
    moveTo(dx, dy);
    if (fabs(rollDeg) < 5.0)       printf("\033[32m@\033[0m");
    else if (fabs(rollDeg) < 15.0) printf("\033[33m@\033[0m");
    else                            printf("\033[31m@\033[0m");

    // 중심
    moveTo(cx, cy);
    printf("\033[37m+\033[0m");

    // 롤 각도 텍스트
    moveTo(cx - 7, cy + R + 2);
    printf("\033[33mRoll: %+6.1f deg\033[0m", rollDeg);
}

// ─── ASCII 로켓 ──────────────────────────────────────────────────────
void drawRocket(int sx, int sy) {
    const char* lines[] = {
        "    /\\    ",
        "   /  \\   ",
        "  | ** |  ",
        "  |    |  ",
        "  | RW |  ",
        "  |    |  ",
        " /|    |\\ ",
        "/  ----  \\",
        "   |  |   ",
        "  _|  |_  ",
        " /______\\ ",
        "   [==]   ",
    };
    for (int i = 0; i < 12; i++) {
        moveTo(sx, sy + i);
        printf("\033[37m%s\033[0m", lines[i]);
    }
}

// ─── 텔레메트리 패널 ─────────────────────────────────────────────────
void drawTelemetry(int px, int py, const RocketState& rs,
                   double rwRPM, double pidOut, double maxRPM) {
    moveTo(px, py);
    printf("\033[36;1m=== TELEMETRY ===================\033[0m");
    moveTo(px, py+1);
    printf("\033[90m---------------------------------\033[0m");

    moveTo(px, py+2);
    printf("\033[37mAltitude :\033[0m \033[32m%8.1f m   \033[0m", rs.altitude);
    moveTo(px, py+3);
    printf("\033[37mVelocity :\033[0m \033[36m%+8.1f m/s\033[0m", rs.velocity);
    moveTo(px, py+4);
    printf("\033[37mTime     :\033[0m \033[90m%8.1f s   \033[0m", rs.time);

    moveTo(px, py+5);
    printf("\033[90m---------------------------------\033[0m");

    const char* rc = (fabs(rs.roll) < 5) ? "\033[32m"
                   : (fabs(rs.roll) < 15) ? "\033[33m" : "\033[31m";
    moveTo(px, py+6);
    printf("\033[37mRoll Ang :\033[0m %s%+8.2f deg \033[0m", rc, rs.roll);
    moveTo(px, py+7);
    printf("\033[37mRoll Rate:\033[0m \033[33m%+8.2f deg/s\033[0m", rs.rollRate);
    moveTo(px, py+8);
    printf("\033[37mWind     :\033[0m \033[90m%+8.2f d/s2 \033[0m", rs.windDisturbance);

    moveTo(px, py+9);
    printf("\033[90m---------------------------------\033[0m");

    const char* rc2 = (fabs(rwRPM) < maxRPM * 0.7) ? "\033[36m" : "\033[31m";
    moveTo(px, py+10);
    printf("\033[37mRW RPM   :\033[0m %s%+8.0f rpm \033[0m", rc2, rwRPM);
    moveTo(px, py+11);
    printf("\033[37mPID Out  :\033[0m \033[35m%+8.3f     \033[0m", pidOut);

    // 부하 바
    moveTo(px, py+12);
    printf("\033[37mRW Load  :\033[0m [");
    float load = (float)(fabs(rwRPM) / maxRPM);
    int filled = (int)(load * 20);
    for (int i = 0; i < 20; i++) {
        if (i < filled) printf(load < 0.7f ? "\033[32m#\033[0m" : "\033[31m#\033[0m");
        else            printf("\033[90m-\033[0m");
    }
    printf("\033[37m]%3.0f%%\033[0m", load * 100.0f);
}

// ─── main ────────────────────────────────────────────────────────────
int main() {
    srand((unsigned)time(NULL));
    enableANSI();
    hideCursor();
    system("cls");

    // 헤더
    printf("\033[34;1m+------------------------------------------------------------------+\033[0m\n");
    printf("\033[34;1m|\033[0m\033[37;1m    ROCKET REACTION WHEEL SIMULATION  (C++ / PID Controller)     \033[0m\033[34;1m|\033[0m\n");
    printf("\033[34;1m+------------------------+-----------+---------------------------------+\033[0m\n");
    printf("\033[34;1m|\033[0m \033[36mROLL INDICATOR          \033[34;1m|\033[0m  \033[36mROCKET   \033[34;1m|\033[0m  \033[36mTELEMETRY               \033[34;1m|\033[0m\n");
    printf("\033[34;1m+------------------------+-----------+---------------------------------+\033[0m\n");

    Rocket        rocket;
    PIDController pid(2.5, 0.05, 1.2);
    ReactionWheel rw(8000.0, 0.01);

    double pidOutput = 0.0;
    bool   paused    = false;
    const double DT  = 0.033;

    while (true) {
        // 키 입력
        if (_kbhit()) {
            char ch = (char)_getch();
            if (ch == ' ')              paused = !paused;
            if (ch == 'r' || ch == 'R') { rocket = Rocket(); pid.reset(); }
            if (ch == 'q' || ch == 'Q') break;
        }

        // 시뮬레이션 업데이트
        if (!paused && rocket.isFlying()) {
            const RocketState& rs = rocket.getState();
            pidOutput = pid.compute(0.0, rs.roll, DT);
            rw.update(pidOutput, DT);
            rocket.update(DT, rw.getTorque());
        }

        // 화면 그리기
        drawRollIndicator(12, 11, rocket.getState().roll);
        drawRocket(27, 5);
        drawTelemetry(40, 5, rocket.getState(), rw.getRPM(), pidOutput, rw.getMaxRPM());

        // 하단 상태
        moveTo(2, 20);
        if (!rocket.isFlying()) {
            printf("\033[31;1m-- FLIGHT ENDED --  R: restart, Q: quit              \033[0m");
        } else if (paused) {
            printf("\033[33;1mPAUSED  (SPACE: resume, R: restart, Q: quit)         \033[0m");
        } else {
            printf("\033[90mRunning...  SPACE: pause  R: restart  Q: quit        \033[0m");
        }

        Sleep((DWORD)(DT * 1000.0));
    }

    moveTo(0, 22);
    printf("\033[0m\n");
    return 0;
}
