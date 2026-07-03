/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo; https://bjsstukeji.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习).
 *            The code ADAPTS the corresponding hardware circuit board(智能汽车-ICAR),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file manualControl.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 手动接管控制线程实现
 * @version 0.1
 * @date 2025-12-29
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/manualControl.hpp"
#include "utils/tools.hpp"
#include <unistd.h>

ManualControlThread::ManualControlThread() {
    serverSocket = -1;
    clientSocket = -1;
    running = false;
    connected = false;
}

ManualControlThread::~ManualControlThread() {
    stop();
}

void ManualControlThread::start() {
    running = true;
    lastContact = std::chrono::steady_clock::now();

    // Create server socket
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket < 0)
    {
        perror("[Manual] socket failed");
        running = false;
        return;
    }

    // 允许端口立即重用，防止程序重启后TIME_WAIT导致bind失败
    int opt = 1;
    if (setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0)
    {
        perror("[Manual] setsockopt SO_REUSEADDR failed");
    }

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(8080);
    serverAddr.sin_addr.s_addr = INADDR_ANY;

    if (bind(serverSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0)
    {
        perror("[Manual] bind failed");
        close(serverSocket);
        serverSocket = -1;
        running = false;
        return;
    }

    if (listen(serverSocket, 1) < 0)
    {
        perror("[Manual] listen failed");
        close(serverSocket);
        serverSocket = -1;
        running = false;
        return;
    }

    thread = std::thread(&ManualControlThread::run, this);
}

void ManualControlThread::stop() {
    running = false;

    // Close sockets
    if (clientSocket >= 0) {
        close(clientSocket);
        clientSocket = -1;
    }
    if (serverSocket >= 0) {
        close(serverSocket);
        serverSocket = -1;
    }

    // Wait for thread to finish
    if (thread.joinable()) {
        thread.join();
    }
}

void ManualControlThread::sendImage(cv::Mat &img) {
    if (!img.empty()) {
        cout << "[Manual] Received image to send: size=" << img.cols << "x" << img.rows << endl;
        std::lock_guard<std::mutex> lock(mtxImg);
        image = img.clone();
        hasImage = true;
        cvImg.notify_one();
    }
}

void ManualControlThread::updateVehicleState(float speed, float steering) {
    std::lock_guard<std::mutex> lock(mtxState);
    vehicleState.speed = speed;
    vehicleState.steering = steering;
    vehicleState.emergency = false;
}

bool ManualControlThread::isManualControl() {
    return manualControl.forward || manualControl.backward ||
           manualControl.left || manualControl.right;
}

void ManualControlThread::applyManualControl(float *speed, uint16_t *steering) {
    cout << "[Manual] Applying manual control - Forward:" << manualControl.forward
         << ", Backward:" << manualControl.backward
         << ", Left:" << manualControl.left
         << ", Right:" << manualControl.right << endl;

    if (manualControl.emergencyStop) {
        *speed = 0;
        *steering = PWMSERVOMID; // 中间位置
        return;
    }

    // Speed control
    if (manualControl.forward) {
        *speed = 0.3f;  // Forward speed
    } else if (manualControl.backward) {
        *speed = -0.3f; // Backward speed
    } else {
        *speed = 0.0f;  // Stop
    }

    // Steering control
    if (manualControl.left) {
        *steering = PWMSERVOMID - 300; // 左转
    } else if (manualControl.right) {
        *steering = PWMSERVOMID + 300; // 右转
    } else {
        *steering = PWMSERVOMID;  // 直行
    }
}

bool ManualControlThread::checkForReturnKey() {
    bool ret = manualControl.returnAuto;
    if (ret) {
        manualControl.returnAuto = false;
    }
    return ret;
}

void ManualControlThread::disconnectClient() {
    if (clientSocket >= 0) {
        close(clientSocket);
        clientSocket = -1;
    }
    connected = false;
}

void ManualControlThread::run() {
    while (running) {
        // 如果socket未成功初始化（start失败），等待重试
        if (serverSocket < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // Wait for client connection
        addrSize = sizeof(clientAddr);
        clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddr, &addrSize);

        if (clientSocket < 0) {
            continue;
        }

        connected = true;
        lastContact = std::chrono::steady_clock::now();  // 重置超时计时（防止旧连接的lastContact导致立即超时）
        // 禁用Nagle算法，降低控制延迟
        int flag = 1;
        setsockopt(clientSocket, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
        printf("[Manual] Remote control connected\n");

        // Handle client connection
        handleClientConnection();

        connected = false;
        if (clientSocket >= 0) {
            close(clientSocket);
            clientSocket = -1;
        }
    }
}

void ManualControlThread::handleClientConnection() {
    // Reset manual control
    manualControl.forward = false;
    manualControl.backward = false;
    manualControl.left = false;
    manualControl.right = false;
    manualControl.emergencyStop = false;
    manualControl.returnAuto = false;

    // Start receiving commands thread
    std::thread cmdThread(&ManualControlThread::receiveCommands, this);
    cmdThread.detach();

    // Start timeout check thread
    std::thread timeoutThread([this]() {
        while (running && connected) {
            checkTimeout();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });
    timeoutThread.detach();

    // Send images and state
    while (running && connected) {
        // Send vehicle state (higher frequency for state)
        std::lock_guard<std::mutex> lock(mtxState);
        std::string state = "STATE:" + std::to_string(vehicleState.speed) +
                          "," + std::to_string(vehicleState.steering) + "\n";

        // 发送状态数据（带错误检查）
        int stateSent = send(clientSocket, state.c_str(), state.length(), 0);
        if (stateSent < 0) {
            cerr << "[Manual] Error sending state data" << endl;
            break;
        }

        // Send image if available (every frame)
        if (hasImage) {
            std::lock_guard<std::mutex> imgLock(mtxImg);
            if (!image.empty()) {
                try {
                    // 提高JPEG质量
                    std::vector<uchar> buf;
                    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 30};  // 30%质量
                    cv::imencode(".jpg", image, buf, params);

                    // 限制图像大小（如果太大，压缩质量会自动降低）
                    if (buf.size() > 50000) {  // 50KB限制
                        buf.resize(50000);
                    }

                    std::string header = "IMAGE:" + std::to_string(buf.size()) + "\n";
                    cout << "[Manual] Sending image: size=" << buf.size() << endl;

                    // 发送图像头
                    send(clientSocket, header.c_str(), header.length(), 0);
                    // 发送图像数据
                    send(clientSocket, buf.data(), buf.size(), 0);
                } catch (const cv::Exception& e) {
                    cerr << "[Manual] Image encode error: " << e.what() << endl;
                }
            }
            hasImage = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(16));  // 约60Hz
    }
}

void ManualControlThread::receiveCommands() {
    std::string lineBuf;  // 粘包缓冲区
    char rawBuf[1024];
    while (running && connected) {
        int bytes = recv(clientSocket, rawBuf, 1023, 0);
        if (bytes <= 0) {
            break;
        }

        rawBuf[bytes] = '\0';
        lineBuf += std::string(rawBuf);

        // Update contact time
        lastContact = std::chrono::steady_clock::now();

        // 按换行符分割处理（解决TCP粘包）
        size_t pos;
        while ((pos = lineBuf.find('\n')) != std::string::npos) {
            std::string cmd = lineBuf.substr(0, pos + 1);  // 包含\n
            lineBuf.erase(0, pos + 1);

            processCommand(cmd);

            if (manualControl.returnAuto) {
                printf("[Manual] Return to auto command received\n");
                goto done;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
done:
    lineBuf.clear();
}

void ManualControlThread::processCommand(const std::string &cmd) {
    // 处理特殊命令
    if (cmd == "RETURN\n") {
        manualControl.returnAuto = true;
        controlChanged = true;
        printf("[Manual] Return to auto command received\n");
        return;
    }
    if (cmd == "STOP\n") {
        manualControl.forward = false;
        manualControl.backward = false;
        manualControl.left = false;
        manualControl.right = false;
        manualControl.emergencyStop = true;
        manualControl.returnAuto = false;
        controlChanged = true;
        printf("[Manual] Stop command received\n");
        return;
    }

    // 组合命令解析（如"WA\n"=前进+左转，"WD\n"=前进+右转）
    manualControl.forward = false;
    manualControl.backward = false;
    manualControl.left = false;
    manualControl.right = false;
    manualControl.emergencyStop = false;
    manualControl.returnAuto = false;
    for (char c : cmd) {
        switch (c) {
            case 'W': manualControl.forward = true; break;
            case 'S': manualControl.backward = true; break;
            case 'A': manualControl.left = true; break;
            case 'D': manualControl.right = true; break;
        }
    }
    controlChanged = true;
    printf("[Manual] Command received: %s", cmd.c_str());
}

void ManualControlThread::checkTimeout() {
    if (connected) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - lastContact).count();

        if (elapsed > 30000) { // 30 seconds timeout
            printf("[Manual] Connection timeout, returning to auto mode\n");
            emergencyStop();
            connected = false;
        }
    }
}

void ManualControlThread::emergencyStop() {
    std::lock_guard<std::mutex> lock(mtxState);
    vehicleState.speed = 0;
    vehicleState.steering = 0;
    vehicleState.emergency = true;
}