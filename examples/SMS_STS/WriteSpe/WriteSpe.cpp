#include <iostream> 
#include <math.h> 
#include <vector> 
#include <string>
#include <sstream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include "SCServo.h"

using namespace std;

// 假设这是你的舵机通信库
SMS_STS sms_sts;

// 1. 舵机参数结构体
struct ServoParam {
    double minDegree; // 物理最小角度
    double maxDegree; // 物理最大角度
    int minPos;      // 对应的最小脉冲值
    int maxPos;      // 对应的最大脉冲值
    
    // 位置转角度
    double posToDegree(int pos) {
        if (pos <= minPos) return minDegree;
        if (pos >= maxPos) return maxDegree;
        return minDegree + (double)(pos - minPos) * (maxDegree - minDegree) / (maxPos - minPos);
    }
    
    // 角度转位置
    int degreeToPos(double degree) {
        if (degree <= minDegree) return minPos;
        if (degree >= maxDegree) return maxPos;
        return (int)(minPos + (degree - minDegree) * (maxPos - minPos) / (maxDegree - minDegree));
    }
};

// 2. 机械臂控制器类
class ArmController {
private:
    double L1, L2; // 连杆长度
    int armId;     // 机械臂编号 (1-4)
    
    // 两个关节的参数
    ServoParam joint1Param;
    ServoParam joint2Param;
    
    // 用于存储计算结果的临时变量 (弧度)
    double theta1_rad = 0.0;
    double theta2_rad = 0.0;  // 这是θ₂，关节1与关节2的夹角
    double theta3_rad = 0.0; 
    
public:
    // 构造函数
    ArmController(int id, double length1, double length2, ServoParam param1, ServoParam param2)
        : armId(id), L1(length1), L2(length2), joint1Param(param1), joint2Param(param2) {}
    
    // 逆运动学求解核心算法
    bool calculateIK(double x, double y, int& pos1, int& pos2, bool verbose = false) {
        // 计算距离
        double dist_sq = x*x + y*y;
        double dist = sqrt(dist_sq);
        
        // 1. 检查是否在工作空间内
        double min_dist = fabs(L1 - L2);
        double max_dist = L1 + L2;
        if (dist > max_dist || dist < min_dist) {
            if (verbose) {
                cout << "  [机械臂" << armId << "] 超出工作空间范围! 距离=" << dist 
                     << "mm, 范围:[" << min_dist << ", " << max_dist << "]" << endl;
            }
            return false;
        }
        
        // 2. 计算θ₂ (关节2相对于关节1的夹角)
        double cos_theta2 = (dist_sq - L1*L1 - L2*L2) / (2 * L1 * L2);
        double cos_theta3 = (-dist_sq + L1*L1 + L2*L2) / (2 * L1 * L2);
        
        // 防止浮点数精度误差
        if (cos_theta2 > 1.0) cos_theta2 = 1.0;
        if (cos_theta2 < -1.0) cos_theta2 = -1.0;
        
        // θ₂是关节1与关节2的夹角，使用肘部向下的解
        theta2_rad = acos(cos_theta2);
        theta3_rad = acos(cos_theta3);
        
        // 3. 计算θ₁ (关节1相对于x轴的角度)
        double target_angle = atan2(y, x);
        double alpha = atan2(L2 * sin(theta2_rad), L1 + L2 * cos(theta2_rad));
        theta1_rad = target_angle - alpha;
        
        // 4. 将弧度转换为角度
        double theta1_deg = theta1_rad * 180.0 / M_PI;
        double theta2_deg = theta2_rad * 180.0 / M_PI;
        double theta3_deg = theta3_rad * 180.0 / M_PI;
        
        if (verbose) {
            // 调试输出
            cout << "  [机械臂" << armId << "] 计算详情：" << endl;
            cout << "    目标坐标: (" << x << ", " << y << ")" << endl;
            cout << "    目标角度: " << (atan2(y, x) * 180.0 / M_PI) << "度" << endl;
            cout << "    关节2角度: " << theta2_deg << "度" << endl;
            cout << "    关节1角度: " << theta1_deg << "度" << endl;
        }
        
        // 5. 检查角度是否在关节范围内
        if (theta1_deg < joint1Param.minDegree || theta1_deg > joint1Param.maxDegree) {
            if (verbose) {
                cout << "  [机械臂" << armId << "] 关节1角度超出范围: " << theta1_deg << "度" 
                     << " 范围: [" << joint1Param.minDegree << ", " 
                     << joint1Param.maxDegree << "]" << endl;
            }
            return false;
        }
        
        if (theta3_deg < joint2Param.minDegree || theta3_deg > joint2Param.maxDegree) {
            if (verbose) {
                cout << "  [机械臂" << armId << "] 关节3角度超出范围: " << theta3_deg << "度"
                     << " 范围: [" << joint2Param.minDegree << ", " 
                     << joint2Param.maxDegree << "]" << endl;
            }
            return false;
        }
        
        // 6. 转换为舵机脉冲值
        pos1 = joint1Param.degreeToPos(theta1_deg);
        pos2 = joint2Param.degreeToPos(theta3_deg);
        
        return true;
    }
    
    // 移动机械臂到指定坐标
    bool moveTo(double x, double y, int speed = 2400, int time = 50, bool verbose = false) {
        int pos1, pos2;
        
        if (verbose) {
            cout << "\n尝试移动机械臂" << armId << "到 (" << x << ", " << y << ")..." << endl;
        }
        
        if (calculateIK(x, y, pos1, pos2, verbose)) {
            // 计算舵机ID
            int servo1_id = (armId - 1) * 2 + 1;
            int servo2_id = (armId - 1) * 2 + 2;
            
            // 发送指令给舵机
            sms_sts.WritePosEx(servo1_id, pos1, speed, time);
            sms_sts.WritePosEx(servo2_id, pos2, speed, time);
            
            if (verbose) {
                // 获取计算出的角度用于显示
                double theta1_deg, theta2_deg;
                getAngles(theta1_deg, theta2_deg);
                
                cout << "  [机械臂" << armId << "] 移动成功!" << endl;
                cout << "  舵机位置: (" << pos1 << ", " << pos2 << ")" << endl;
                cout << "  关节角度: (" << theta1_deg << "°, " << theta2_deg << "°)" << endl;
            }
            
            return true;
        } else {
            if (verbose) {
                cout << "  [机械臂" << armId << "] 移动失败: 目标位置不可达!" << endl;
            }
            return false;
        }
    }
    
    // 获取当前计算出的角度（用于调试）
    void getAngles(double& theta1_deg, double& theta2_deg) {
        theta1_deg = theta1_rad * 180.0 / M_PI;
        theta2_deg = theta2_rad * 180.0 / M_PI;
    }
    
    // 打印机械臂参数
    void printInfo() {
        cout << "机械臂 " << armId << " 配置: ";
        cout << "L1=" << L1 << "mm, L2=" << L2 << "mm" << endl;
        cout << "  关节1范围: " << joint1Param.minDegree << " 到 " 
                  << joint1Param.maxDegree << " 度" << endl;
        cout << "  关节2范围: " << joint2Param.minDegree << " 到 " 
                  << joint2Param.maxDegree << " 度" << endl;
    }
    
    // 获取机械臂ID
    int getId() const { return armId; }
    
    // 移动到初始位置
    bool moveToHome(int speed = 2400, int time = 50) {
        // 初始位置设为 (300, 300)
        return moveTo(300, 300, speed, time, true);
    }
};

// 键盘控制类
class KeyboardController {
private:
    vector<ArmController> arms;
    vector<pair<double, double>> currentPos; // 每个机械臂的当前位置
    int selectedArm; // 当前选中的机械臂: 1-4为单独控制，5为控制所有
    double stepSize; // 移动步长
    int speed, time; // 移动速度和时长
    
    // 终端原始设置
    struct termios oldt, newt;
    
    // 设置终端为非阻塞模式
    void setNonBlockingMode(bool enable) {
        if (enable) {
            tcgetattr(STDIN_FILENO, &oldt);
            newt = oldt;
            newt.c_lflag &= ~(ICANON | ECHO);
            tcsetattr(STDIN_FILENO, TCSANOW, &newt);
            
            int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
            fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
        } else {
            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        }
    }
    
    // 清屏
    void clearScreen() {
        cout << "\033[2J\033[1;1H"; // ANSI转义序列
    }
    
    // 显示控制界面
    void displayInterface() {
        clearScreen();
        
        cout << "=========================================" << endl;
        cout << "        机械臂键盘控制程序" << endl;
        cout << "=========================================" << endl;
        cout << endl;
        
        // 显示当前控制状态
        cout << "当前控制: ";
        if (selectedArm >= 1 && selectedArm <= 4) {
            cout << "单独控制机械臂 " << selectedArm << " ";
        } else if (selectedArm == 5) {
            cout << "同时控制所有机械臂 ";
        }
        cout << "   步长: " << stepSize << "mm" << endl;
        
        cout << "速度: " << speed << "   时长: " << time << "ms" << endl;
        cout << endl;
        
        // 显示机械臂位置
        cout << "机械臂当前位置:" << endl;
        cout << "┌──────┬────────────┬────────────┐" << endl;
        cout << "│ 编号 │    X(mm)   │    Y(mm)   │" << endl;
        cout << "├──────┼────────────┼────────────┤" << endl;
        for (int i = 0; i < 4; i++) {
            cout << "│  " << (i+1) << "   │";
            cout.width(10);
            cout << currentPos[i].first << "  │";
            cout.width(10);
            cout << currentPos[i].second << "  │" << endl;
        }
        cout << "└──────┴────────────┴────────────┘" << endl;
        cout << endl;
        
        // 显示控制说明
        cout << "键盘控制:" << endl;
        cout << "──────────────────────────────────────" << endl;
        cout << "W/S/A/D: 上/下/左/右移动" << endl;
        cout << "I/K/J/L: 斜向移动(上/下/左/右斜)" << endl;
        cout << "1-4: 选择机械臂1-4单独控制" << endl;
        cout << "5: 同时控制所有机械臂" << endl;
        cout << "+/-: 增加/减小步长" << endl;
        cout << "[: 减小速度  ]: 增加速度" << endl;
        cout << "<: 减小时长  >: 增加时长" << endl;
        cout << "H: 回到初始位置(300, 300)" << endl;
        cout << "R: 重置所有机械臂到初始位置" << endl;
        cout << "Space: 重新显示界面" << endl;
        cout << "Q: 退出程序" << endl;
        cout << "=========================================" << endl;
    }
    
    // 移动机械臂
    void moveArm(int armIndex, double deltaX, double deltaY) {
        double newX = currentPos[armIndex].first + deltaX;
        double newY = currentPos[armIndex].second + deltaY;
        
        if (arms[armIndex].moveTo(newX, newY, speed, time, false)) {
            currentPos[armIndex] = make_pair(newX, newY);
        } else {
            cout << "\a"; // 蜂鸣声提示错误
        }
    }
    
public:
    KeyboardController() : selectedArm(1), stepSize(1.0), speed(2400), time(50) {
        // 初始化四个机械臂的参数
        ServoParam p1_j1 = {-127.5, 48.0, 2000, 0};
        ServoParam p1_j2 = {6.5, 182.3, 0, 2000};
        
        ServoParam p2_j1 = {-127.5, 48.0, 0, 2000};
        ServoParam p2_j2 = {6.5, 182.3, 2000, 0};
        
        ServoParam p3_j1 = {-90.0, 85.8, 2000, 0};
        ServoParam p3_j2 = {6.5, 182.3, 0, 2000};
        
        ServoParam p4_j1 = {-90.0, 85.8, 0, 2000};
        ServoParam p4_j2 = {6.5, 182.3, 2000, 0};
        
        // 创建四个机械臂控制器
        arms.emplace_back(1, 130, 370, p1_j1, p1_j2);
        arms.emplace_back(2, 130, 370, p2_j1, p2_j2);
        arms.emplace_back(3, 130, 370, p3_j1, p3_j2);
        arms.emplace_back(4, 130, 370, p4_j1, p4_j2);
        
        // 初始化当前位置
        for (int i = 0; i < 4; i++) {
            currentPos.push_back(make_pair(300.0, 300.0)); // 初始位置
        }
        
        // 设置非阻塞键盘输入
        setNonBlockingMode(true);
    }
    
    ~KeyboardController() {
        // 恢复终端设置
        setNonBlockingMode(false);
    }
    
    // 运行键盘控制
    void run() {
        bool running = true;
        char input;
        
        // 先移动到初始位置
        for (int i = 0; i < 4; i++) {
            arms[i].moveToHome(speed, time);
        }
        
        usleep(2000000); // 等待2秒
        
        displayInterface();
        
        while (running) {
            if (read(STDIN_FILENO, &input, 1) > 0) {
                switch (input) {
                    // 移动控制
                    case 'w':
                    case 'W':
                        if (selectedArm >= 1 && selectedArm <= 4) {
                            moveArm(selectedArm-1, 0, stepSize);
                        } else if (selectedArm == 5) {
                            for (int i = 0; i < 4; i++) {
                                moveArm(i, 0, stepSize);
                            }
                        }
                        break;
                        
                    case 's':
                    case 'S':
                        if (selectedArm >= 1 && selectedArm <= 4) {
                            moveArm(selectedArm-1, 0, -stepSize);
                        } else if (selectedArm == 5) {
                            for (int i = 0; i < 4; i++) {
                                moveArm(i, 0, -stepSize);
                            }
                        }
                        break;
                        
                    case 'a':
                    case 'A':
                        if (selectedArm >= 1 && selectedArm <= 4) {
                            moveArm(selectedArm-1, -stepSize, 0);
                        } else if (selectedArm == 5) {
                            for (int i = 0; i < 4; i++) {
                                moveArm(i, -stepSize, 0);
                            }
                        }
                        break;
                        
                    case 'd':
                    case 'D':
                        if (selectedArm >= 1 && selectedArm <= 4) {
                            moveArm(selectedArm-1, stepSize, 0);
                        } else if (selectedArm == 5) {
                            for (int i = 0; i < 4; i++) {
                                moveArm(i, stepSize, 0);
                            }
                        }
                        break;
                        
                    // 斜向移动
                    case 'i':
                    case 'I':
                        if (selectedArm >= 1 && selectedArm <= 4) {
                            moveArm(selectedArm-1, stepSize, stepSize);
                        } else if (selectedArm == 5) {
                            for (int i = 0; i < 4; i++) {
                                moveArm(i, stepSize, stepSize);
                            }
                        }
                        break;
                        
                    case 'k':
                    case 'K':
                        if (selectedArm >= 1 && selectedArm <= 4) {
                            moveArm(selectedArm-1, -stepSize, -stepSize);
                        } else if (selectedArm == 5) {
                            for (int i = 0; i < 4; i++) {
                                moveArm(i, -stepSize, -stepSize);
                            }
                        }
                        break;
                        
                    case 'j':
                    case 'J':
                        if (selectedArm >= 1 && selectedArm <= 4) {
                            moveArm(selectedArm-1, -stepSize, stepSize);
                        } else if (selectedArm == 5) {
                            for (int i = 0; i < 4; i++) {
                                moveArm(i, -stepSize, stepSize);
                            }
                        }
                        break;
                        
                    case 'l':
                    case 'L':
                        if (selectedArm >= 1 && selectedArm <= 4) {
                            moveArm(selectedArm-1, stepSize, -stepSize);
                        } else if (selectedArm == 5) {
                            for (int i = 0; i < 4; i++) {
                                moveArm(i, stepSize, -stepSize);
                            }
                        }
                        break;
                    
                    // 选择机械臂
                    case '1':
                        selectedArm = 1;
                        break;
                        
                    case '2':
                        selectedArm = 2;
                        break;
                        
                    case '3':
                        selectedArm = 3;
                        break;
                        
                    case '4':
                        selectedArm = 4;
                        break;
                        
                    case '5':
                        selectedArm = 5;
                        break;
                    
                    // 步长调整
                    case '+':
                        stepSize *= 2.0;
                        if (stepSize > 100.0) stepSize = 100.0;
                        break;
                        
                    case '-':
                        stepSize /= 2.0;
                        if (stepSize < 0.1) stepSize = 0.1;
                        break;
                    
                    // 速度调整
                    case '[':
                        speed -= 200;
                        if (speed < 100) speed = 100;
                        break;
                        
                    case ']':
                        speed += 200;
                        if (speed > 3000) speed = 3000;
                        break;
                    
                    // 时间调整
                    case '<':
                        time -= 5;
                        if (time < 10) time = 10;
                        break;
                        
                    case '>':
                        time += 5;
                        if (time > 200) time = 200;
                        break;
                    
                    // 特殊功能
                    case 'h':
                    case 'H':
                        if (selectedArm >= 1 && selectedArm <= 4) {
                            arms[selectedArm-1].moveToHome(speed, time);
                            currentPos[selectedArm-1] = make_pair(300.0, 300.0);
                        } else if (selectedArm == 5) {
                            for (int i = 0; i < 4; i++) {
                                arms[i].moveToHome(speed, time);
                                currentPos[i] = make_pair(300.0, 300.0);
                            }
                        }
                        break;
                        
                    case 'r':
                    case 'R':
                        for (int i = 0; i < 4; i++) {
                            arms[i].moveToHome(speed, time);
                            currentPos[i] = make_pair(300.0, 300.0);
                        }
                        break;
                    
                    // 刷新界面
                    case ' ':
                        break;
                    
                    // 退出
                    case 'q':
                    case 'Q':
                        running = false;
                        cout << "退出程序..." << endl;
                        break;
                }
                
                if (running) {
                    displayInterface();
                }
            }
            
            usleep(10000); // 10ms延迟，减少CPU使用率
        }
    }
};

// ==================== 主程序 ====================
int main() {
    // 1. 初始化舵机通信
    cout << "正在初始化串口通信..." << endl;
    if(!sms_sts.begin(1000000, "/dev/ttyUSB0")) {
        cout << "串口初始化失败! 请检查设备连接。" << endl;
        return -1;
    }
    cout << "串口初始化成功！" << endl;
    
    // 2. 创建并运行键盘控制器
    KeyboardController controller;
    
    cout << "\n系统初始化完成，共4个机械臂!" << endl;
    cout << "坐标系: x轴向右, y轴向前" << endl;
    cout << "注意：只使用肘部向下解，超出范围则报错。" << endl;
    
    // 显示初始提示
    cout << "\n==========================" << endl;
    cout << "  系统就绪，可以开始控制" << endl;
    cout << "  建议工作空间范围:" << endl;
    cout << "  - 最近距离: " << fabs(130-370) << "mm" << endl;
    cout << "  - 最远距离: " << (130+370) << "mm" << endl;
    cout << "==========================" << endl;
    cout << "按任意键开始键盘控制..." << endl;
    getchar(); // 等待按键
    
    // 运行键盘控制
    controller.run();
    
    // 3. 关闭串口
    sms_sts.end();
    cout << "程序结束" << endl;
    
    return 0;
}