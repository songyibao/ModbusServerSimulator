//
// Created by root on 3/8/25.
//

#include "main.h"
#include "modbus.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <unordered_map>
#include <sys/socket.h>
#include <unistd.h>
#include <signal.h>
using namespace std;

// 定义寄存器类型枚举
enum RegisterType {
    COIL,
    HOLDING_REGISTER
};

// 存储寄存器信息的结构体
struct Register {
    RegisterType type;
    int address;
    uint16_t value;
};

class ModbusServer {
private:
    modbus_t* ctx;
    modbus_mapping_t* mapping;
    unordered_map<int, Register*> coil_map;
    unordered_map<int, Register*> holding_reg_map;
    int server_socket;
    string host;
    int port;

public:
    ModbusServer(const string& host, int port):host(host),port(port) {
        // 创建Modbus TCP上下文
        ctx = modbus_new_tcp(host.c_str(), port);
        if (!ctx) {
            throw runtime_error("Failed to create Modbus context");
        }

        // 初始化映射结构（初始大小设为0）
        mapping = modbus_mapping_new(0, 0, 0, 0);
        if (!mapping) {
            modbus_free(ctx);
            throw runtime_error("Failed to create Modbus mapping");
        }
        server_socket = -1;
    }

    ~ModbusServer() {
        if (mapping) modbus_mapping_free(mapping);
        if (ctx) modbus_free(ctx);
        if (server_socket != -1) close(server_socket);

        // 可选：如果担心 new Register 的内存泄漏，可以在此清理：
        for (auto &kv : coil_map) {
            delete kv.second;
        }
        for (auto &kv : holding_reg_map) {
            delete kv.second;
        }
    }

    static uint16_t modbus_get_short_from_byte(uint8_t *query) {
        // 解析起始地址（16位大端字节序）
        uint16_t addr = (query[0] << 8) | query[1];
        return addr;
    }

    // 从CSV文件加载寄存器配置
    void loadConfig(const string& filename) {
        ifstream file(filename);
        if (!file) throw runtime_error("Cannot open CSV file");

        string line;
        int max_coil = -1;
        int max_holding = -1;

        // 第一次读取：仅用来确定最大地址
        while (getline(file, line)) {
            istringstream ss(line);
            string type, addr, val;
            getline(ss, type, ',');
            getline(ss, addr, ',');
            getline(ss, val, ',');

            if (type == "COIL") {
                max_coil = max(max_coil, stoi(addr));
            } else if (type == "HOLDING") {
                max_holding = max(max_holding, stoi(addr));
            }
        }

        // 重新分配映射内存
        modbus_mapping_free(mapping);
        mapping = modbus_mapping_new(
            /* nb_bits          = */ max_coil >= 0 ? max_coil + 1 : 0,
            /* nb_input_bits    = */ 0,
            /* nb_registers     = */ max_holding >= 0 ? max_holding + 1 : 0,
            /* nb_input_registers = */ 0
        );

        // 回到文件开头，进行第二次读取
        file.clear();
        file.seekg(0);

        while (getline(file, line)) {
            istringstream ss(line);
            string type, addr, val;
            getline(ss, type, ',');
            getline(ss, addr, ',');
            getline(ss, val, ',');

            int address = stoi(addr);
            uint16_t value = stoi(val);

            if (type == "COIL") {
                mapping->tab_bits[address] = value ? 1 : 0;
                coil_map[address] = new Register{COIL, address, value};
            } else if (type == "HOLDING") {
                mapping->tab_registers[address] = value;
                holding_reg_map[address] = new Register{HOLDING_REGISTER, address, value};
            }
        }

        // ===== 在此打印所有寄存器点位信息 =====
        cout << "===== 已加载寄存器配置 =====" << endl;

        cout << "COIL 列表：" << endl;
        for (auto &kv : coil_map) {
            cout << "  Address: " << kv.first << ", Value: " << kv.second->value << endl;
        }

        cout << "HOLDING REGISTER 列表：" << endl;
        for (auto &kv : holding_reg_map) {
            cout << "  Address: " << kv.first << ", Value: " << kv.second->value << endl;
        }
        cout << "========================================" << endl;
    }

    // 启动服务器
    void start() {
        server_socket = modbus_tcp_listen(ctx, 5);
        if (server_socket == -1) {
            throw runtime_error("Listen failed");
        }

        cout << "Modbus server started on port " << port << endl;

        while (true) {
            modbus_tcp_accept(ctx, &server_socket);

            // 处理客户端请求
            uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
            while (true) {
                int rc = modbus_receive(ctx, query);
                if (rc == -1) break;

                // 使用自定义回复处理
                if (rc > 0) {
                    processRequest(query, rc);
                }
            }
            // cout << "Client disconnected" << endl;
        }
    }

private:
    // 自定义请求处理逻辑
    void processRequest(uint8_t* query, int query_len) {
        int function = query[modbus_get_header_length(ctx)];
        // cout << "Function: " << function << endl;
        try {
            switch (function) {
                case MODBUS_FC_READ_COILS:
                    handleReadCoils(query, query_len);
                    break;
                case MODBUS_FC_WRITE_SINGLE_COIL:
                    handleWriteCoil(query, query_len);
                    break;
                case MODBUS_FC_READ_HOLDING_REGISTERS:
                    handleReadRegisters(query, query_len);
                    break;
                case MODBUS_FC_WRITE_SINGLE_REGISTER:
                    handleWriteRegister(query, query_len);
                    break;
                default:
                    modbus_reply_exception(ctx, query, MODBUS_EXCEPTION_ILLEGAL_FUNCTION);
            }
        } catch (const exception& e) {
            cerr << "Error processing request: " << e.what() << endl;
        }
    }

    // 处理线圈读取请求
    void handleReadCoils(uint8_t* query, int query_len) {
        int addr = modbus_get_short_from_byte(query + modbus_get_header_length(ctx) + 1);
        int nb = modbus_get_short_from_byte(query + modbus_get_header_length(ctx) + 3);

        if (addr + nb > mapping->nb_bits) {
            modbus_reply_exception(ctx, query, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
            return;
        }

        modbus_reply(ctx, query, query_len, mapping);
    }

    // 处理保持寄存器读取请求
    void handleReadRegisters(uint8_t* query, int query_len) {
        // 计算请求数据起始偏移量（跳过MBAP头）
        const int header_len = modbus_get_header_length(ctx);
        const int data_offset = header_len + 1; // 功能码后开始是地址

        // 解析起始地址（16位大端字节序）
        uint16_t addr = (query[data_offset] << 8) | query[data_offset + 1];

        // 解析寄存器数量（16位大端字节序）
        uint16_t nb = (query[data_offset + 2] << 8) | query[data_offset + 3];
        // cout<< "addr: " << addr << " nb: " << nb << endl;
        // 地址范围校验
        if (addr + nb > mapping->nb_registers) {
            modbus_reply_exception(ctx, query, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
            return;
        }

        // 生成正常响应
        modbus_reply(ctx, query, query_len, mapping);
    }

    // 处理线圈写入请求
    void handleWriteCoil(uint8_t* query, int query_len) {
        int addr = modbus_get_short_from_byte(query + modbus_get_header_length(ctx) + 1);
        uint16_t value = modbus_get_short_from_byte(query + modbus_get_header_length(ctx) + 3);

        if (addr >= mapping->nb_bits) {
            modbus_reply_exception(ctx, query, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
            return;
        }

        mapping->tab_bits[addr] = value ? 1 : 0;
        if (coil_map.count(addr)) {
            coil_map[addr]->value = value;
        }
        modbus_reply(ctx, query, query_len, mapping);
    }

    // 处理寄存器写入请求
    void handleWriteRegister(uint8_t* query, int query_len) {
        int addr = modbus_get_short_from_byte(query + modbus_get_header_length(ctx) + 1);
        uint16_t value = modbus_get_short_from_byte(query + modbus_get_header_length(ctx) + 3);

        if (addr >= mapping->nb_registers) {
            modbus_reply_exception(ctx, query, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
            return;
        }

        mapping->tab_registers[addr] = value;
        if (holding_reg_map.count(addr)) {
            holding_reg_map[addr]->value = value;
        }
        modbus_reply(ctx, query, query_len, mapping);
    }
};
void handle_signal(int sig) {
    // 如果需要，可以在这里打印信息或执行其他操作
    if (sig == SIGINT) {
        std::cout << "Received SIGINT, exiting gracefully...\n";
        exit(0);
    } else if (sig == SIGTSTP) {
        std::cout << "Received SIGTSTP (Ctrl+Z), but ignoring to prevent port lock.\n";
    }

    // 在退出之前执行其他清理工作（如果需要）
    // 例如，关闭打开的文件，释放其他资源等

    // 然后退出程序
    exit(0);
}
int main() {
    signal(SIGINT, handle_signal);
    signal(SIGTSTP, handle_signal);
    try {
        ModbusServer server("0.0.0.0", 502);
        server.loadConfig("/software/modbus_server/register.csv");
        server.start();
    } catch (const exception& e) {
        cerr << "Server error: " << e.what() << endl;
        return 1;
    }

    return 0;
}
