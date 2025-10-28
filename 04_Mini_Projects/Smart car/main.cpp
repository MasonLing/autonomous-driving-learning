#include<iostream>
#include<graphics.h>
#include<conio.h>
#include <windows.h>
#include <math.h>
#include <string>

using namespace std;

// ������������Ƿ���ײ�ĺ���
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

// ��������֮�����ĺ���
double distance(int x1, int y1, int x2, int y2)
{
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// ������Ϣ��庯�� - �ƶ������ڵײ�
void drawInfoPanel(int carX, int carY, bool collision, bool autoMode)
{
    // ��������ײ���Ϣ������򣬱�����Ӱ
    setfillcolor(WHITE);
    solidrectangle(0, 500, 800, 600);

    // �����ı���ɫ�ͱ���
    settextcolor(BLUE);
    setbkmode(TRANSPARENT);

    // �����ı�����ʹ�С
    settextstyle(14, 0, _T("Arial"));

    // �ڴ��ڵײ����ƴ�������Ϣ
    outtextxy(15, 500, _T("Smart Car Simulator v1.0"));
    outtextxy(15, 520, _T("Creator: MasonLing"));

    // ���Ʒָ���
    setlinecolor(LIGHTGRAY);
    line(0, 540, 800, 540);

    // ���ƹ���˵��
    settextcolor(DARKGRAY);
    settextstyle(15, 0, _T("Arial"));
    outtextxy(10, 550, _T("Controls: Arrow Keys to Move"));
    outtextxy(250, 550, _T("A: Auto Mode"));
    outtextxy(400, 550, _T("M: Manual Mode"));
    outtextxy(550, 550, _T("ESC: Exit"));

    // ����״̬��Ϣ
    TCHAR posInfo[50];
    _stprintf_s(posInfo, _T("Position: (%d, %d)"), carX, carY);
    outtextxy(10, 570, posInfo);

    // ���ģʽ��ʾ���򣬱�����Ӱ
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

    // �����ײ״̬��ʾ���򣬱�����Ӱ
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

    // ������ɫ˵��
    settextcolor(DARKGRAY);
    outtextxy(550, 570, _T("Blue:Manual Yellow:Auto Red:Collision"));
}

int main()
{
    // ��ʼ��ͼ�δ���
    initgraph(800, 600);

    // ���ñ���ɫΪ��ɫ
    setbkcolor(WHITE);

    // ����˫�����ͼģʽ
    SetWorkingImage();
    BeginBatchDraw();

    // С����ʼλ�ú��ٶ�
    int carX = 100;
    int carY = 200;
    int speed = 5;

    // Ŀ��λ��
    int targetX = 600;
    int targetY = 400;

    // �������ϰ���
    int obstacles[3][4] = {
        {400, 250, 80, 80},
        {200, 100, 60, 60},
        {500, 400, 100, 40}
    };

    // �Զ�ģʽ��־
    bool autoMode = false;

    // ��ײ��־
    bool collision = false;

    // ��¼��һ֡��״̬
    bool lastCollision = false;
    int lastCarX = carX;
    int lastCarY = carY;
    bool lastAutoMode = autoMode;

    // === ���Ƴ�ʼ���� ===
    cleardevice();

    // ����Ŀ���
    setfillcolor(GREEN);
    solidcircle(targetX, targetY, 10);

    // ���������ϰ���
    setfillcolor(RED);
    for (int i = 0; i < 3; i++)
    {
        solidrectangle(obstacles[i][0], obstacles[i][1],
            obstacles[i][0] + obstacles[i][2],
            obstacles[i][1] + obstacles[i][3]);
    }

    // ���Ƴ�ʼС��
    setfillcolor(BLUE);
    solidrectangle(carX, carY, carX + 200, carY + 100);

    setfillcolor(BLACK);
    solidcircle(carX + 30, carY + 110, 20);
    solidcircle(carX + 170, carY + 110, 20);

    setfillcolor(LIGHTBLUE);
    solidrectangle(carX + 80, carY + 20, carX + 180, carY + 60);

    // ������Ϣ���
    drawInfoPanel(carX, carY, collision, autoMode);

    // ���ڴ��е�ͼ��һ������ʾ����Ļ
    FlushBatchDraw();

    // === ��ѭ�� ===
    while (true)
    {
        // ��ⰴ��
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

        // �Զ������߼�
        if (autoMode)
        {
            // ����ӵ�ǰλ��ָ��Ŀ��������
            int dx = targetX - carX;
            int dy = targetY - carY;

            // ��һ������
            double length = sqrt(dx * dx + dy * dy);
            if (length > 0)
            {
                dx = (int)(dx / length * speed);
                dy = (int)(dy / length * speed);
            }

            // �����ƶ�����λ��
            int newCarX = carX + dx;
            int newCarY = carY + dy;

            // ����Ƿ����ײ
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

            // ������ײ����������Ƿ��ƶ�
            if (!willCollide)
            {
                carX = newCarX;
                carY = newCarY;
            }
            else
            {
                // �������ײ�������ƿ�
                carX += dy;
                carY -= dx;
            }
        }

        // �߽��� - �����߽磬ȷ��С�����������Ϣ�������
        if (carX < 0) carX = 0;
        if (carX > 600) carX = 600;
        if (carY < 0) carY = 0;
        if (carY > 470) carY = 470; // �����߽磬���������Ϣ�������

        // ��ײ���
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

        // �ж��Ƿ���Ҫ�ػ�
        bool needRedraw = (carX != lastCarX || carY != lastCarY ||
            collision != lastCollision || autoMode != lastAutoMode);

        if (needRedraw)
        {
            // ������һ֡��С��
            setfillcolor(WHITE);
            solidrectangle(lastCarX, lastCarY, lastCarX + 200, lastCarY + 100);
            setfillcolor(WHITE);
            solidcircle(lastCarX + 30, lastCarY + 110, 25);
            solidcircle(lastCarX + 170, lastCarY + 110, 25);
            setfillcolor(WHITE);
            solidrectangle(lastCarX + 80, lastCarY + 20, lastCarX + 180, lastCarY + 60);

            // �ػ汻С�����ǵ��ϰ����Ŀ���
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

            // �ػ汻���ǵ�Ŀ���
            if (checkCollision(lastCarX, lastCarY, 200, 130,
                targetX - 10, targetY - 10, 20, 20))
            {
                setfillcolor(GREEN);
                solidcircle(targetX, targetY, 10);
            }

            // ������λ�õ�С��
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

            // ���¼�¼��״̬
            lastCarX = carX;
            lastCarY = carY;
            lastCollision = collision;
            lastAutoMode = autoMode;

            // ������Ϣ��� - �����ڵײ��̶�λ��
            drawInfoPanel(carX, carY, collision, autoMode);

            // ���ڴ��е�ͼ����ʾ����Ļ
            FlushBatchDraw();
        }

        // ����ѭ���ٶ�
        Sleep(20);
    }

    // �������ǰ��������
    EndBatchDraw();
    closegraph();
    return 0;
}