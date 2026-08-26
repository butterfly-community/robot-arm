# fashionstar-uart

独立的 FashionStar UART 协议与串口库。生产代码只使用厂家协议已有的 1 Mbps、8N1、
Ping、Monitor、只读内部 PID 参数查询和同步多圈位置命令。常规命令使用 100 ms 串口
超时；内部参数查询按厂家脚本在发送后等待 200 ms。机械臂关节顺序、弧度换算和夹爪
正负号不属于本库。

只读查询 ID 0–6 当前内部参数：

```sh
cargo run --manifest-path arm/src/fashionstar-uart/Cargo.toml \
  --example read_internal_params -- /dev/ttyUSB0
```

单元测试：

```sh
cargo test --manifest-path arm/src/fashionstar-uart/Cargo.toml
```

与厂家 Python SDK 1.3.12 通过 `socat` 伪串口做双向交叉测试：

```sh
cargo build --manifest-path arm/src/fashionstar-uart/Cargo.toml --example cross_peer
python3 arm/src/fashionstar-uart/tests/python_sdk_cross.py \
  --sdk-wheel /path/to/fashionstar_uart_sdk-1.3.12-py3-none-any.whl \
  --rust-peer arm/src/fashionstar-uart/target/debug/examples/cross_peer
```

测试先让 Rust 客户端连接 Python 协议端，再让厂家 Python `PortHandler` 连接 Rust
协议端。每个方向默认执行 64 轮七舵机 Monitor 与同步位置写入，并覆盖乱序 Monitor、
串口分片、帧前噪声、损坏帧重同步、负角度、全部单字节帧长度、协议整数边界和
Python/Rust 半单位舍入一致性。
