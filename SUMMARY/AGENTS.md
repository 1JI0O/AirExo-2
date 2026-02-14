# AGENT 公约

- **打印的输出**：
	- `print` 等日志输出使用简明的英文
- **注释**：
    - 对于生成的内容，需要添加必要的注释
	- 注释使用清晰明确的纯文字中文注释，不要出现辅助符号例如分界线
	- 注释中绝对不允许出现 Step xx、步骤 xx、Phase xx 或者序号 x
	- 注释中不允许出现emoji
	- 严禁使用行尾注释，必须使用单行注释
	- 注释越少越好，每个代码小节均只用一小句话注释；代码细节注释放到 cell 外部解释，不要写为内部注释
- **Markdown 公式书写**：
	- 块公式统一使用 `$$...$$`
	- 行内公式统一使用 `$...$`

## ipynb 安全读取策略

### 背景与问题
- 整本读取 `.ipynb` 会按文件体量线性吞入上下文。
- Notebook 若包含大输出、图像或 base64，容易触发上下文暴涨并冲击 700 KiB 级别阈值。
- 默认禁用整本直读，统一采用“元信息扫描 + 单 cell 定点读取 + 强制截断”。

### 本仓库实测数据

#### 1) 当前工作区 notebook 风险量化
| 指标 | 数值 |
| --- | --- |
| `.ipynb` 文件数 | 5 |
| 总体积 | 159001 B（约 155.27 KiB） |
| 最大文件 | `robot_reconstruction_Version4_two_arm.ipynb` = 71778 B（约 70.10 KiB） |
| 700 KiB 阈值 | 716800 B |
| 当前最大占阈值比例 | 约 10.0% |
| 超过 700 KiB 文件数 | 0 |

#### 2) 各 notebook cell 统计
| notebook | cells | max chars/cell | mean chars/cell |
| --- | ---: | ---: | ---: |
| `reconstruct_arm.ipynb` | 2 | 5446 | 5332.00 |
| `reconstruct_arm_fix.ipynb` | 2 | 6723 | 3472.00 |
| `reconstruct_two_arm.ipynb` | 8 | 5684 | 2136.75 |
| `robot_reconstruction_Version4.ipynb` | 24 | 5410 | 877.04 |
| `robot_reconstruction_Version4_two_arm.ipynb` | 29 | 7488 | 1708.00 |

#### 3) 单 cell 安全读取验证
- 文件：`robot_reconstruction_Version4_two_arm.ipynb`
- 目标：最长 cell（index=15）
- 原始长度：7488 字符
- 安全输出限制：最多 120 行 / 4000 字符
- 结果：已成功截断

### 风险判定
- 当前仓库在“文件体量”维度风险可控（暂无超过 700 KiB 的 notebook）。
- 机制性风险仍在：整本读取 `.ipynb` 会线性吞入上下文；若含大输出/base64，可能快速冲到 700 KiB 以上。
- 结论：默认禁用整本直读；采用“先元信息扫描，再按 index/关键词定点读取单 cell，并强制截断”的流程。

### 标准安全读取流程（SOP）
1. 先做元信息扫描：仅统计 notebook 数量、体积、cell 数与长度分布。
2. 通过 index 或关键词定位目标 cell。
3. 仅读取目标 cell 的 `source` 字段，不读取整本正文。
4. 强制截断输出：最多 120 行 / 4000 字符（取更严格边界）。
5. 若信息不足，再继续读取下一个目标 cell，重复步骤 2-4。
6. 默认不返回 `outputs` 和 base64 大字段。

### 最小命令模板

#### 1) 按 index 读取单 cell
```bash
python - <<'PY'
import json

nb_path = "robot_reconstruction_Version4_two_arm.ipynb"
cell_index = 15
MAX_LINES = 120
MAX_CHARS = 4000

with open(nb_path, "r", encoding="utf-8") as f:
    nb = json.load(f)

src = "".join(nb["cells"][cell_index].get("source", []))
safe = "\n".join(src.splitlines()[:MAX_LINES])[:MAX_CHARS]

print(safe)
print(f"\n[index]={cell_index}, raw_chars={len(src)}, returned_chars={len(safe)}, truncated={len(safe) < len(src)}")
PY
```

#### 2) 按关键词读取首个命中 cell
```bash
python - <<'PY'
import json

nb_path = "robot_reconstruction_Version4_two_arm.ipynb"
keyword = "TODO"
MAX_LINES = 120
MAX_CHARS = 4000

with open(nb_path, "r", encoding="utf-8") as f:
    cells = json.load(f)["cells"]

hit = None
for i, c in enumerate(cells):
    src = "".join(c.get("source", []))
    if keyword in src:
        hit = (i, src)
        break

if hit is None:
    raise SystemExit(f"未命中关键词: {keyword}")

idx, src = hit
safe = "\n".join(src.splitlines()[:MAX_LINES])[:MAX_CHARS]

print(f"[index]={idx}")
print(safe)
print(f"\n[keyword]={keyword}, raw_chars={len(src)}, returned_chars={len(safe)}, truncated={len(safe) < len(src)}")
PY
```

#### 3) 仅返回长度和摘要（不返全文）
```bash
python - <<'PY'
import json

nb_path = "robot_reconstruction_Version4_two_arm.ipynb"
cell_index = 15

with open(nb_path, "r", encoding="utf-8") as f:
    cell = json.load(f)["cells"][cell_index]

src = "".join(cell.get("source", []))
lines = src.splitlines()
first_nonempty = next((ln.strip() for ln in lines if ln.strip()), "")

print({
    "index": cell_index,
    "chars": len(src),
    "lines": len(lines),
    "summary": first_nonempty[:120]
})
PY
```

### 强制规则（Do/Don’t）

**Do**
- 先元信息扫描，再定点读取。
- 仅按 index/关键词读取单 cell。
- 强制执行 120 行 / 4000 字符截断。
- 能用长度与摘要解决时，优先不返回全文。

**Don’t**
- 不整本直读 `.ipynb` 并全量输出。
- 不返回 `outputs` 或 base64 大字段。
- 不跳过截断，或私自提高截断上限。
- 不在一次请求中批量返回多个长 cell 正文。
