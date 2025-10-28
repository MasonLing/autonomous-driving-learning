#include<iostream>
#include<graphics.h>
#include<conio.h>
#include <windows.h>
#include <math.h>
#include <string>

using namespace std;

// 检测两个矩形是否碰撞的函数
bool checkCollision(int rect1X, int rect1Y, int rect1Width, int rect1Height,
    int rect2X, int rect2Y, int rect2Width, int rect2Height)
{
    if (rect1X < rect2X + rect2Width &&
        rect1X + rect1Width > rect2X &&
        rect1Y < rect2Y + rect2Height &&
        rect1Y + rect1Height > rect2Y)
    {
        return true;
    }
    return false;
}

// 计算两点之间距离的函数
double distance(int x1, int y1, int x2, int y2)
{
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// 绘制信息面板函数 - 移动到窗口底部
void drawInfoPanel(int carX, int carY, bool collision, bool autoMode)
{
    // 首先清除底部信息面板区域，避免重影
    setfillcolor(WHITE);
    solidrectangle(0, 500, 800, 600);

    // 设置文本颜色和背景
    settextcolor(BLUE);
    setbkmode(TRANSPARENT);

    // 设置文本字体和大小
    settextstyle(14, 0, _T("Arial"));

    // 在窗口底部绘制创作者信息
    outtextxy(15, 500, _T("Smart Car Simulator v1.0"));
    outtextxy(15, 520, _T("Creator: MasonLing"));

    // 绘制分隔线
    setlinecolor(LIGHTGRAY);
    line(0, 540, 800, 540);

    // 绘制功能说明
    settextcolor(DARKGRAY);
    settextstyle(15, 0, _T("Arial"));
    outtextxy(10, 550, _T("Controls: Arrow Keys to Move"));
    outtextxy(250, 550, _T("A: Auto Mode"));
    outtextxy(400, 550, _T("M: Manual Mode"));
    outtextxy(550, 550, _T("ESC: Exit"));

    // 绘制状态信息
    TCHAR posInfo[50];
    _stprintf_s(posInfo, _T("Position: (%d, %d)"), carX, carY);
    outtextxy(10, 570, posInfo);

    // 清除模式显示区域，避免重影
    setfillcolor(WHITE);
    solidrectangle(200, 570, 400, 590);

    if (autoMode)
    {
        outtextxy(200, 570, _T("Mode: Auto Navigation"));
    }
    else
    {
        outtextxy(200, 570, _T("Mode: Manual Control"));
    }

    // 清除碰撞状态显示区域，避免重影
    setfillcolor(WHITE);
    solidrectangle(400, 570, 550, 590);

    if (collision)
    {
        settextcolor(RED);
        outtextxy(400, 570, _T("Collision: YES!"));
    }
    else
    {
        settextcolor(GREEN);
        outtextxy(400, 570, _T("Collision: No"));
    }

    // 绘制颜色说明
    settextcolor(DARKGRAY);
    outtextxy(550, 570, _T("Blue:Manual Yellow:Auto Red:Collision"));
}

int main()
{
    // 初始化图形窗口
    initgraph(800, 600);

    // 设置背景色为白色
    setbkcolor(WHITE);

    // 启用双缓冲绘图模式
    SetWorkingImage();
    BeginBatchDraw();

    // 小车初始位置和速度
    int carX = 100;
    int carY = 200;
    int speed = 5;

    // 目标位置
    int targetX = 600;
    int targetY = 400;

    // 定义多个障碍物
    int obstacles[3][4] = {
        {400, 250, 80, 80},
        {200, 100, 60, 60},
        {500, 400, 100, 40}
    };

    // 自动模式标志
    bool autoMode = false;

    // 碰撞标志
    bool collision = false;

    // 记录上一帧的状态
    bool lastCollision = false;
    int lastCarX = carX;
    int lastCarY = carY;
    bool lastAutoMode = autoMode;

    // === 绘制初始场景 ===
    cleardevice();

    // 绘制目标点
    setfillcolor(GREEN);
    solidcircle(targetX, targetY, 10);

    // 绘制所有障碍物
    setfillcolor(RED);
    for (int i = 0; i < 3; i++)
    {
        solidrectangle(obstacles[i][0], obstacles[i][1],
            obstacles[i][0] + obstacles[i][2],
            obstacles[i][1] + obstacles[i][3]);
    }

    // 绘制初始小车
    setfillcolor(BLUE);
    solidrectangle(carX, carY, carX + 200, carY + 100);

    setfillcolor(BLACK);
    solidcircle(carX + 30, carY + 110, 20);
    solidcircle(carX + 170, carY + 110, 20);

    setfillcolor(LIGHTBLUE);
    solidrectangle(carX + 80, carY + 20, carX + 180, carY + 60);

    // 绘制信息面板
    drawInfoPanel(carX, carY, collision, autoMode);

    // 将内存中的图像一次性显示到屏幕
    FlushBatchDraw();

    // === 主循环 ===
    while (true)
    {
        // 检测按键
        if (GetAsyncKeyState(VK_LEFT) & 0x8000)
        {
            carX -= speed;
            autoMode = false;
        }
        if (GetAsyncKeyState(VK_RIGHT) & 0x8000)
        {
            carX += speed;
            autoMode = false;
        }
        if (GetAsyncKeyState(VK_UP) & 0x8000)
        {
            carY -= speed;
            autoMode = false;
        }
        if (GetAsyncKeyState(VK_DOWN) & 0x8000)
        {
            carY += speed;
            autoMode = false;
        }
        if (GetAsyncKeyState('A') & 0x8000)
        {
            autoMode = true;
            Sleep(100);
        }
        if (GetAsyncKeyState('M') & 0x8000)
        {
            autoMode = false;
            Sleep(100);
        }
        if (GetAsyncKeyState(VK_ESCAPE) & 0x8000)
        {
            break;
        }

        // 自动避障逻辑
        if (autoMode)
        {
            // 计算从当前位置指向目标点的向量
            int dx = targetX - carX;
            int dy = targetY - carY;

            // 归一化向量
            double length = sqrt(dx * dx + dy * dy);
            if (length > 0)
            {
                dx = (int)(dx / length * speed);
                dy = (int)(dy / length * speed);
            }

            // 尝试移动到新位置
            int newCarX = carX + dx;
            int newCarY = carY + dy;

            // 检查是否会碰撞
            bool willCollide = false;
            for (int i = 0; i < 3; i++)
            {
                if (checkCollision(newCarX, newCarY, 200, 100,
                    obstacles[i][0], obstacles[i][1],
                    obstacles[i][2], obstacles[i][3]))
                {
                    willCollide = true;
                    break;
                }
            }

            // 根据碰撞检测结果决定是否移动
            if (!willCollide)
            {
                carX = newCarX;
                carY = newCarY;
            }
            else
            {
                // 如果会碰撞，尝试绕开
                carX += dy;
                carY -= dx;
            }
        }

        // 边界检查 - 调整边界，确保小车不会进入信息面板区域
        if (carX < 0) carX = 0;
        if (carX > 600) carX = 600;
        if (carY < 0) carY = 0;
        if (carY > 470) carY = 470; // 调整边界，避免进入信息面板区域

        // 碰撞检测
        collision = false;
        for (int i = 0; i < 3; i++)
        {
            if (checkCollision(carX, carY, 200, 100,
                obstacles[i][0], obstacles[i][1],
                obstacles[i][2], obstacles[i][3]))
            {
                collision = true;
                break;
            }
        }

        // 判断是否需要重绘
        bool needRedraw = (carX != lastCarX || carY != lastCarY ||
            collision != lastCollision || autoMode != lastAutoMode);

        if (needRedraw)
        {
            // 擦除上一帧的小车
            setfillcolor(WHITE);
            solidrectangle(lastCarX, lastCarY, lastCarX + 200, lastCarY + 100);
            setfillcolor(WHITE);
            solidcircle(lastCarX + 30, lastCarY + 110, 25);
            solidcircle(lastCarX + 170, lastCarY + 110, 25);
            setfillcolor(WHITE);
            solidrectangle(lastCarX + 80, lastCarY + 20, lastCarX + 180, lastCarY + 60);

            // 重绘被小车覆盖的障碍物和目标点
            setfillcolor(RED);
            for (int i = 0; i < 3; i++)
            {
                if (checkCollision(lastCarX, lastCarY, 200, 130,
                    obstacles[i][0], obstacles[i][1],
                    obstacles[i][2], obstacles[i][3]))
                {
                    solidrectangle(obstacles[i][0], obstacles[i][1],
                        obstacles[i][0] + obstacles[i][2],
                        obstacles[i][1] + obstacles[i][3]);
                }
            }

            // 重绘被覆盖的目标点
            if (checkCollision(lastCarX, lastCarY, 200, 130,
                targetX - 10, targetY - 10, 20, 20))
            {
                setfillcolor(GREEN);
                solidcircle(targetX, targetY, 10);
            }

            // 绘制新位置的小车
            if (collision)
            {
                setfillcolor(RED);
            }
            else if (autoMode)
            {
                setfillcolor(YELLOW);
            }
            else
            {
                setfillcolor(BLUE);
            }

            solidrectangle(carX, carY, carX + 200, carY + 100);

            setfillcolor(BLACK);
            solidcircle(carX + 30, carY + 110, 20);
            solidcircle(carX + 170, carY + 110, 20);

            setfillcolor(LIGHTBLUE);
            solidrectangle(carX + 80, carY + 20, carX + 180, carY + 60);

            // 更新记录的状态
            lastCarX = carX;
            lastCarY = carY;
            lastCollision = collision;
            lastAutoMode = autoMode;

            // 绘制信息面板 - 总是在底部固定位置
            drawInfoPanel(carX, carY, collision, autoMode);

            // 将内存中的图像显示到屏幕
            FlushBatchDraw();
        }

        // 控制循环速度
        Sleep(20);
    }

    // 程序结束前的清理工作
    EndBatchDraw();
    closegraph();
    return 0;
}