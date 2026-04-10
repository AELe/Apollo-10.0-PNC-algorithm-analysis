# Apollo专栏文章Markdown格式化指南

本文档记录了为《Apollo10.0规划控制算法详解》专栏文章制定的Markdown格式化规范，方便在所有文章中保持一致的格式和风格。

## 📋 格式化规范总结

### 1. 标题层级结构

```markdown
# 文章主标题 (模块名称) - 文章副标题

## 1. 一级章节标题

### 1.1 二级小节标题

#### 1.1.1 三级小节标题（可选）
```

**示例：**
```markdown
# Routing 模块 (1) - 全局路线的生成

## 1. 通过外部请求 Routing 模块进行算路

### 1.1 请求方式

### 1.2 请求 Channel
```

### 2. 代码块格式

**C++代码：**
````markdown
```cpp
// C++代码示例
message LaneFollowCommand {
  optional apollo.common.Header header = 1;
  optional int64 command_id = 2 [default = -1];
}
```
````

**Protobuf消息：**
````markdown
```protobuf
// 虽然使用cpp语法高亮，但注明是Protobuf
message RoutingRequest {
  optional apollo.common.Header header = 1;
  repeated apollo.routing.LaneWaypoint waypoint = 2;
}
```
````

### 3. 列表和强调

**无序列表：**
```markdown
- **Dreamview/Dreamview Plus** - 可视化界面
- **planning_command 脚本** - 命令行工具
- **task_manager 模块** - 系统内部模块
```

**有序列表：**
```markdown
1. 创建 `lane_follow_command` 的 client
2. 获取 `LaneFollowCommand` 类型的数据
3. 通过 `SendRequest` 发送算路请求消息
```

**强调：**
```markdown
**重要概念** - 使用粗体强调
`代码或变量` - 使用行内代码标记
> **注意：** 重要的注意事项使用引用块
```

### 4. 图片引用格式

```markdown
![图片描述文字](images/图片文件名.png "图X：图片标题")

*图X：图片标题*
```

**示例：**
```markdown
![全局路线生成算法流程图](images/1.png "图1：算法流程图")

*图1：全局路线生成算法流程图*
```

### 5. 章节分隔

```markdown
---

## 下一章节标题
```

### 6. 文件引用

```markdown
**文件：** `modules/task_manager/task_manager_component.cc`
```

### 7. 参数说明

```markdown
**必须设置的参数：**
- `command_id` - 指令的唯一标识，可以设为 0
- `end_pose` - 目的地的坐标（以自车为参考系的坐标）
```

---

## 🎯 完整文章结构模板

```markdown
# [模块名称] ([序号]) - [文章标题]

## 1. [一级章节标题]

### 1.1 [二级小节标题]

[内容描述...]

```cpp
// 代码示例
```

### 1.2 [二级小节标题]

[内容描述...]

![图片描述](images/图片名.png "图1：图片标题")

*图1：图片标题*

---

## 2. [下一章节标题]

### 2.1 [二级小节标题]

[内容描述...]

> **注意：** 重要的注意事项

### 2.2 [二级小节标题]

**重要概念：**
- **概念1** - 说明
- **概念2** - 说明

---

## 总结

本文详细介绍了...，包括：

1. **功能1** - 描述
2. **功能2** - 描述
3. **功能3** - 描述

[总结性文字...]
```

---

## 📁 目录和文件命名规范

### 文章文件命名
```
[序号]-[模块名称]-[文章标题].md
```

**示例：**
- `01-routing-module-global-route-generation.md`
- `02-planning-module-reference-line-generation.md`

### 图片目录结构
```
images/
├── [模块名称]/
│   ├── 1.png
│   ├── 2.png
│   └── ...
└── [其他模块]/
```

---

## 🔧 快速格式化脚本

如果您需要批量格式化文章，可以使用以下Python脚本模板：

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Apollo专栏文章格式化脚本
"""

import re

def format_markdown_file(input_file, output_file):
    """格式化Markdown文件"""
    with open(input_file, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # 1. 修复标题层级
    content = re.sub(r'^####\s+(.+)$', r'### \1', content, flags=re.MULTILINE)
    content = re.sub(r'^#####\s+(.+)$', r'#### \1', content, flags=re.MULTILINE)
    
    # 2. 修复代码块
    content = re.sub(r'^```$', r'```cpp', content, flags=re.MULTILINE)
    
    # 3. 修复图片引用
    content = re.sub(r'!\\\[.*?\\\]\\((.+?)\s+null\\)', r'![\1](\1)', content)
    
    # 4. 添加强调
    content = re.sub(r'(\*\*)?(command_id|end_pose|waypoint)(\*\*)?', r'**\2**', content)
    
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(content)
    
    print(f"已格式化: {input_file} -> {output_file}")

# 使用示例
if __name__ == "__main__":
    format_markdown_file("原始文件.md", "格式化后文件.md")
```

---

## 📝 检查清单

在完成文章格式化后，请检查以下项目：

### 结构检查
- [ ] 标题层级是否正确（# → ## → ###）
- [ ] 是否有章节分隔线（---）
- [ ] 列表格式是否正确
- [ ] 代码块是否有正确的语法高亮

### 内容检查
- [ ] 所有代码变量使用 `行内代码` 标记
- [ ] 重要概念使用 **粗体** 强调
- [ ] 图片引用格式正确
- [ ] 文件引用格式正确

### 格式检查
- [ ] 使用中文标点符号
- [ ] 适当的空白行分隔段落
- [ ] 图片有标题和编号
- [ ] 代码有适当的注释

---

## 🚀 最佳实践建议

### 1. 保持一致性
- 所有文章使用相同的标题层级
- 代码块统一使用 `cpp` 语法高亮
- 图片使用相同的引用格式

### 2. 提高可读性
- 每段文字不超过5-7行
- 复杂概念使用列表说明
- 重要代码添加注释

### 3. 便于维护
- 使用相对路径引用图片
- 文件名使用英文，避免特殊字符
- 保持目录结构清晰

### 4. 适合多种平台
- 格式在GitHub、GitLab、本地编辑器都能正常显示
- 图片使用常见格式（PNG、JPG、SVG）
- 避免使用平台特定的Markdown扩展

---

## 📚 示例文件

完整的格式化示例请参考：
- `routing模块(1)-全局路线的生成.md` - 已格式化的完整示例
- `README.md` - 索引文件格式示例

---

## 🔄 更新日志

### 版本 1.0 (2026-04-10)
- 初始版本创建
- 基于《Routing模块(1)-全局路线的生成》制定规范
- 包含完整的格式模板和检查清单

### 后续更新
- 根据新文章类型添加更多模板
- 优化格式化脚本
- 添加更多实用工具

---

## 📞 使用帮助

如果在格式化过程中遇到问题：
1. 参考本指南的对应章节
2. 查看示例文件 `routing模块(1)-全局路线的生成.md`
3. 使用提供的Python脚本进行批量处理
4. 如有特殊需求，可调整模板以适应具体内容

---

*本指南由AI助手根据《Apollo10.0规划控制算法详解》专栏文章格式化需求创建*
*最后更新：2026-04-10*
```

这个指南包含了所有格式化规范、模板和工具，您可以在处理其他文章时参考使用。需要我帮您格式化其他文章吗？