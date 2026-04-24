#include <iostream> 
#include <math.h> 
#include <vector> 
#include <string>
#include <sstream>
#include "SCServo.h" 
#include <unistd.h>  // 添加usleep的头文件

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
    bool calculateIK(double x, double y, int& pos1, int& pos2) {
        // 计算距离
        double dist_sq = x*x + y*y;
        double dist = sqrt(dist_sq);
        
        // 1. 检查是否在工作空间内
        double min_dist = fabs(L1 - L2);
        double max_dist = L1 + L2;
        if (dist > max_dist || dist < min_dist) {
            cout << "  [机械臂" << armId << "] 超出工作空间范围! 距离=" << dist 
                      << "mm, 范围:[" << min_dist << ", " << max_dist << "]" << endl;
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
        // 调试输出
        cout << "  [机械臂" << armId << "] 计算详情：" << endl;
        cout << "    目标坐标: (" << x << ", " << y << ")" << endl;
        cout << "    目标角度: " << (atan2(y, x) * 180.0 / M_PI) << "度" << endl;
        cout << "    关节2角度: " << theta2_deg << "度" << endl;
        cout << "    关节1角度: " << theta1_deg << "度" << endl;
        
        // 5. 检查角度是否在关节范围内
        if (theta1_deg < joint1Param.minDegree || theta1_deg > joint1Param.maxDegree) {
            cout << "  [机械臂" << armId << "] 关节1角度超出范围: " << theta1_deg << "度" 
                      << " 范围: [" << joint1Param.minDegree << ", " 
                      << joint1Param.maxDegree << "]" << endl;
            return false;
        }
        
        if (theta2_deg < joint2Param.minDegree || theta2_deg > joint2Param.maxDegree) {
            cout << "  [机械臂" << armId << "] 关节2角度超出范围: " << theta2_deg << "度"
                      << " 范围: [" << joint2Param.minDegree << ", " 
                      << joint2Param.maxDegree << "]" << endl;
            return false;
        }
        
        // 6. 转换为舵机脉冲值
        pos1 = joint1Param.degreeToPos(theta1_deg);
        //pos2 = joint2Param.degreeToPos(theta2_deg);
        pos2 = joint2Param.degreeToPos(theta3_deg);
        
        return true;
    }
    
    // 移动机械臂到指定坐标
    bool moveTo(double x, double y, int speed = 2400, int time = 50) {
        int pos1, pos2;
        
        cout << "\n尝试移动机械臂" << armId << "到 (" << x << ", " << y << ")..." << endl;
        
        if (calculateIK(x, y, pos1, pos2)) {
            // 计算舵机ID
            int servo1_id = (armId - 1) * 2 + 1;
            int servo2_id = (armId - 1) * 2 + 2;
            
            // 发送指令给舵机
            sms_sts.WritePosEx(servo1_id, pos1, speed, time);
            sms_sts.WritePosEx(servo2_id, pos2, speed, time);
            
            // 获取计算出的角度用于显示
            double theta1_deg, theta2_deg;
            getAngles(theta1_deg, theta2_deg);
            
            cout << "  [机械臂" << armId << "] 移动成功!" << endl;
            cout << "  舵机位置: (" << pos1 << ", " << pos2 << ")" << endl;
            cout << "  关节角度: (" << theta1_deg << "°, " << theta2_deg << "°)" << endl;
            
            return true;
        } else {
            cout << "  [机械臂" << armId << "] 移动失败: 目标位置不可达!" << endl;
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
    void moveToHome(int speed = 2400, int time = 50) {
        // 初始位置设为 (300, 300)
        moveTo(300, 300, speed, time);
    }
};

// 终端交互界面类
class TerminalInterface {
private:
    vector<ArmController> arms;
    
    // 显示菜单
    void showMenu() {
        cout << "\n==========================" << endl;
        cout << "  机械臂控制终端" << endl;
        cout << "==========================" << endl;
        cout << "1. 选择机械臂并移动" << endl;
        cout << "2. 同时移动所有机械臂" << endl;
        cout << "3. 显示机械臂信息" << endl;
        cout << "4. 移动到初始位置" << endl;
        cout << "5. 退出" << endl;
        cout << "==========================" << endl;
        cout << "请选择操作 (1-5): ";
    }
    
    // 选择机械臂
    int selectArm() {
        int armId;
        while (true) {
            cout << "\n请选择机械臂 (1-4, 0返回主菜单): ";
            string input;
            getline(cin, input);
            
            if (input == "0") return 0;
            
            try {
                armId = stoi(input);
                if (armId >= 1 && armId <= 4) {
                    return armId;
                } else {
                    cout << "无效的选择，请输入1-4之间的数字。" << endl;
                }
            } catch (...) {
                cout << "无效的输入，请输入数字。" << endl;
            }
        }
    }
    
    // 获取坐标
    bool getCoordinates(double& x, double& y) {
        cout << "\n请输入目标坐标 (格式: x y): ";
        string input;
        getline(cin, input);
        
        stringstream ss(input);
        if (ss >> x >> y) {
            return true;
        } else {
            cout << "坐标格式错误，请重新输入。" << endl;
            return false;
        }
    }
    
    // 获取移动速度和时间
    void getMovementParams(int& speed, int& time) {
        cout << "使用默认速度2400，时间50ms? (y/n): ";
        string input;
        getline(cin, input);
        
        if (input == "y" || input == "Y" || input.empty()) {
            speed = 2400;
            time = 50;
        } else {
            cout << "请输入速度 (默认2400): ";
            getline(cin, input);
            speed = input.empty() ? 2400 : stoi(input);
            
            cout << "请输入时间(ms) (默认50): ";
            getline(cin, input);
            time = input.empty() ? 50 : stoi(input);
        }
    }
    
public:
    TerminalInterface() {
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
    }
    
    // 运行终端界面
    void run() {
        int choice = 0;
        
        do {
            showMenu();
            string input;
            getline(cin, input);
            
            try {
                choice = stoi(input);
            } catch (...) {
                cout << "无效的输入，请输入1-5之间的数字。" << endl;
                continue;
            }
            
            switch (choice) {
                case 1: { // 选择机械臂并移动
                    int armId = selectArm();
                    if (armId == 0) break;
                    
                    double x, y;
                    if (getCoordinates(x, y)) {
                        int speed, time;
                        getMovementParams(speed, time);
                        arms[armId-1].moveTo(x, y, speed, time);
                    }
                    break;
                }
                    
                case 2: { // 同时移动所有机械臂
                    double x, y;
                    if (getCoordinates(x, y)) {
                        int speed, time;
                        getMovementParams(speed, time);
                        
                        cout << "\n移动所有机械臂到 (" << x << ", " << y << ")..." << endl;
                        bool allSuccess = true;
                        for (auto& arm : arms) {
                            if (!arm.moveTo(x, y, speed, time)) {
                                allSuccess = false;
                            }
                        }
                        
                        if (allSuccess) {
                            cout << "\n所有机械臂都成功移动到目标位置！" << endl;
                        } else {
                            cout << "\n部分机械臂移动失败。" << endl;
                        }
                    }
                    break;
                }
                    
                case 3: // 显示机械臂信息
                    cout << "\n=== 机械臂配置信息 ===" << endl;
                    for (auto& arm : arms) {
                        arm.printInfo();
                        cout << endl;
                    }
                    break;
                    
                case 4: { // 移动到初始位置
                    cout << "\n移动所有机械臂到初始位置..." << endl;
                    int speed, time;
                    getMovementParams(speed, time);
                    
                    for (auto& arm : arms) {
                        arm.moveToHome(speed, time);
                    }
                    break;
                }
                    
                case 5: // 退出
                    cout << "程序退出。" << endl;
                    break;
                    
                default:
                    cout << "无效的选择，请输入1-5之间的数字。" << endl;
            }
            
        } while (choice != 5);
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
    
    // 2. 创建并运行终端界面
    TerminalInterface terminal;
    
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
    
    // 运行终端交互界面
    terminal.run();
    
    // 3. 关闭串口
    sms_sts.end();
    cout << "程序结束" << endl;
    
    return 0;
}