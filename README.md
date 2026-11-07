# SLE Reliable Link

基于 BearPi-Pico H3863 / WS63 官方 `sle_uart` 样例实现的可靠遥测与控制链路综合项目。

项目在原始 UART 透传路径上增加了应用层帧协议、CRC16、序号检测、命令应答、周期遥测、异常输入测试和断线业务恢复，用于“深入学习星闪 SLE”系列第十篇综合实践。

## 功能

- Client 通过 SSAP Write Request 下发控制命令；
- Server 通过 SSAP Notification 返回命令 ACK 和周期遥测；
- 应用层帧包含 Magic、版本、类型、序号、长度、Payload 和 CRC16；
- 检测遥测丢失、重复、乱序、CRC 错误和内容错误；
- 支持 START、STOP、SET_PERIOD、GET_STATUS、RESET_STATS、PING；
- 启动时执行 CRC 和协议编解码自检；
- 支持功能闭环、1000 帧连续测试、非法输入测试和五轮恢复测试；
- Server 复位后自动重新扫描、连接、配对和发现服务；
- 配对失败时提供陈旧配对记录清理与限次重连兜底。

本项目没有调用 WS63/H3863 当前 SDK 中尚未实现的专用低时延接口。

## 目录

```text
application/samples/products/sle_uart/
├── CMakeLists.txt
├── Kconfig
├── board.json
├── sle_uart.c
├── sle_reliable_link/
│   ├── sle_rl_protocol.c
│   └── sle_rl_protocol.h
├── sle_uart_client/
│   ├── sle_uart_client.c
│   └── sle_uart_client.h
└── sle_uart_server/
    ├── sle_uart_server.c
    ├── sle_uart_server.h
    ├── sle_uart_server_adv.c
    └── sle_uart_server_adv.h
```

## 应用层帧

所有多字节字段采用小端编码。

```text
+--------+---------+------+-------+--------+---------+---------+-------+
| Magic  | Version | Type | Flags | Seq    | Length  | Payload | CRC16 |
| 2 Byte | 1 Byte  | 1 B  | 1 B   | 4 Byte | 2 Byte  | N Byte  | 2 B   |
+--------+---------+------+-------+--------+---------+---------+-------+
```

- Magic：`0x53 0x4C`（`SL`）
- Version：`1`
- Payload 最大长度：48 字节
- CRC：CRC16-CCITT-FALSE，初值 `0xFFFF`，多项式 `0x1021`
- CRC 覆盖范围：Version 至 Payload
- 标准测试向量：`123456789` → `0x29B1`

CRC16 只用于错误检测，不提供加密或消息认证。

## 使用方法

### 1. 准备官方 SDK

先准备 BearPi-Pico H3863 官方 SDK 和官方 Ubuntu 编译环境。本仓库只保存修改后的样例目录，不包含完整 SDK、编译器和二进制工具。

### 2. 覆盖样例目录

将仓库中的：

```text
application/samples/products/sle_uart
```

复制到官方 SDK 的同名路径：

```text
<bearpi-pico_h3863>/application/samples/products/sle_uart
```

建议先备份原始 `sle_uart` 目录。

### 3. 配置 Server

进入官方图形化配置界面：

```text
Application
→ Enable the Sample of products
→ Support SLE UART sample
→ SLE UART Sample Configuration
→ Select sle uart type
→ Enable SLE UART Server sample
```

保存后编译：

```bash
python build.py -c ws63-liteos-app
```

将生成的固件烧录到 Server 开发板。

### 4. 配置 Client

再次打开配置界面，把 `Select sle uart type` 改为：

```text
Enable SLE UART Client sample
```

保存、编译并烧录到 Client 开发板。

### 5. 选择测试模式

测试模式位于：

```c
application/samples/products/sle_uart/sle_uart_client/sle_uart_client.c
```

相关宏：

```c
#define SLE_RL_TEST_MODE_FUNCTIONAL 1
#define SLE_RL_TEST_MODE_RECOVERY   2
#define SLE_RL_TEST_MODE            SLE_RL_TEST_MODE_RECOVERY
```

- `SLE_RL_TEST_MODE_FUNCTIONAL`：功能闭环、非法输入和连续 1000 帧测试；
- `SLE_RL_TEST_MODE_RECOVERY`：五轮业务恢复测试，每轮接收并校验 100 帧。

切换宏后需要重新编译并烧录 Client。

## 已验证结果

测试平台：两块 BearPi-Pico H3863，Server 为带外置天线的开发板，Client 为另一块开发板。

| 项目 | 本次结果 |
|---|---|
| 协议自检 | Server / Client 均 PASS |
| 功能闭环 | 20 帧，丢失、重复、CRC 和内容错误均为 0 |
| 连续可靠性 | 1000 / 1000，未检测到丢失、重复、乱序、CRC 或内容错误 |
| 非法输入 | CRC、Version、Length、未知命令、越界参数共 5 项均符合预期 |
| 业务恢复 | 5 / 5 轮 PASS，每轮 100 帧，共 500 帧 |

这些结果只描述本次开发板、SDK、固件和实验环境，不是 SLE 或 H3863 的官方性能规格，也不能证明所有射频环境下零丢包。

## 重要边界

- `ssaps_notify_indicate()` 提交成功不等于对端应用已经消费数据，本项目以 Client 解码后的统计作为业务证据；
- `sle_remove_paired_remote_device()` 返回 0 只表示异步请求成功发起，不等于删除已经完成；
- 恢复测试的“服务就绪”终点是服务发现完成，不是第一帧业务到达；
- 公开资料中没有核实配对状态 `0x8000600f` 的符号名称，源码只记录原始状态值；
- 启动日志中的 Flash、MAC 初始化和重复 enable 异常不作为业务成功证据。

## 上游与许可证

本项目基于 BearPi-Pico H3863 SDK 中的 HiSilicon SLE UART Sample 修改。原始源码文件保留其版权声明。

仓库沿用上游 Apache License 2.0，详见 [LICENSE](LICENSE)。

## 参考资料

- [WS63 SLE UART Bridge 示例](https://docs.hisilicon.com/projects/fbb_ws63/master/samples/connectivity/sle/data-comm/uart-bridge.html)
- [WS63 SSAP Server API](https://docs.hisilicon.com/projects/fbb_ws63/master/api-reference/middleware/sle/ssap/server/server.html)
- [WS63 SLE Connection API](https://docs.hisilicon.com/projects/fbb_ws63/master/api-reference/middleware/sle/connection/connection.html)
- [WS63 SLE 示例概述](https://docs.hisilicon.com/projects/fbb_ws63/master/samples/connectivity/sle/overview/index.html)

