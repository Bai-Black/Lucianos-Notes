# 编写标准 MCP 工具指南

> MCP（Model Context Protocol）是由 Anthropic 提出的开放协议，用于标准化 AI 模型与外部工具/数据源之间的通信。

官方：https://github.com/modelcontextprotocol

## 1. 核心概念

### 1.1 MCP 是什么

MCP 是一个 **客户端-服务端** 协议：

```
┌──────────────┐         JSON-RPC 2.0        ┌──────────────┐
│  MCP Client  │ ◄──────────────────────────► │  MCP Server  │
│ (AI 应用)    │                              │ (工具提供者)  │
└──────────────┘                              └──────────────┘
```

- **MCP Client**：AI 应用（如 Claude Desktop、IDE 插件），负责发起请求
- **MCP Server**：你编写的工具服务，负责暴露工具并执行具体逻辑
- **通信协议**：基于 JSON-RPC 2.0，通过 stdio / SSE / HTTP 传输

### 1.2 MCP 的三大能力

| 能力                  | 说明                           | 对应的方法                         |
| --------------------- | ------------------------------ | ---------------------------------- |
| **Tools（工具）**     | AI 可调用的函数/动作           | `tools/list`, `tools/call`         |
| **Resources（资源）** | 可读取的数据（文件、数据库等） | `resources/list`, `resources/read` |
| **Prompts（提示词）** | 预定义的提示词模板             | `prompts/list`, `prompts/get`      |

## 2. 环境准备

### 2.1 选择语言 SDK

MCP 官方提供以下 SDK：

| 语言       | 包名                          | 安装命令                                |
| ---------- | ----------------------------- | --------------------------------------- |
| Python     | `mcp`                         | `pip install mcp`                       |
| TypeScript | `@modelcontextprotocol/sdk`   | `npm install @modelcontextprotocol/sdk` |
| Java       | `io.modelcontextprotocol:sdk` | Maven/Gradle                            |
| Kotlin     | `io.modelcontextprotocol:sdk` | Gradle                                  |

### 2.2 Python 环境（推荐入门）

本文使用 uv 管理 Python 环境，安装 uv :

``` powershell
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
```

> 安装完成后若提示要将其添加至PATH，则手动复制运行其提示的命令

cd 切换至准备编写的 MCP 的项目文件夹，随后执行：

```bash
# 初始化虚拟环境
uv init . -p 3.14	#3.14是python版本

# 安装 MCP SDK
uv add "mcp[cli]"
```

## 3. 快速入门

```python
"""
FastMCP quickstart example.

Run from the repository root:
    uv run examples/snippets/servers/fastmcp_quickstart.py
"""

from mcp.server.fastmcp import FastMCP

# Create an MCP server
mcp = FastMCP("Demo", json_response=True)


# Add an addition tool
@mcp.tool()
def add(a: int, b: int) -> int:
    """Add two numbers"""		#不可省略，用自然语言向LLM描述工具
    return a + b


# Add a dynamic greeting resource
@mcp.resource("greeting://{name}")
def get_greeting(name: str) -> str:
    """Get a personalized greeting"""
    return f"Hello, {name}!"


# Run with streamable HTTP transport
if __name__ == "__main__":
    mcp.run(transport="streamable-http")
```