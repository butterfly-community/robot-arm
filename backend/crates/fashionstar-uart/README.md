# fashionstar-uart

FashionStar UART 协议库。生产路径只包含厂家协议已有的 1 Mbps、8N1、Ping、Monitor、进阶
内部参数读写和同步多圈位置命令；关节符号和工具传动换算均由使用它的型号 execution 节点
负责。一次 Monitor 事务失败时，库清空残留输入并在同一串口完整重试一次；重试仍失败才把
错误交给 execution 节点执行重连。

单元测试：

```sh
cargo test -p fashionstar-uart --all-targets
```

与厂家 `fashionstar_uart_sdk==1.3.12` 双向交叉测试：

```sh
cargo build -p fashionstar-uart --example cross_peer
python3 crates/fashionstar-uart/tests/python_sdk_cross.py \
  --sdk-wheel /path/to/fashionstar_uart_sdk-1.3.12-py3-none-any.whl \
  --rust-peer target/debug/examples/cross_peer
```

脚本让 Rust 和厂家 SDK 分别作为客户端运行，另一侧逐字段核对 Ping、Monitor、同步位置帧、
校验和、分片读取、噪声恢复、一次丢包重试、合法的可选动作响应和角度舍入；它使用临时
PTY，不连接机械臂。
