# Git 协作流程手册

> 本文档整理自 2026-10-28 导师演示的完整操作流程（PowerShell 历史记录第 2146~2185 行），
> 已修正演示过程中的 4 处笔误，并补充每一步的**目的**、**验证方法**和**本工程注意事项**。
>
> 适用场景：单人开发练手 → 多人协作用同一套流程。

---

## 0. 三个必须先建立的概念

### 0.1 三个区域

```
工作区(working tree) ──add──► 暂存区(index) ──commit──► 仓库(.git)
```

`git status` 就是在告诉你"每个文件现在在哪个区"。**所有 Git 操作都是在挪动这三个区里的内容。**

### 0.2 提交是快照，不是差异

每次 commit 存的是**整个项目的完整快照**（Git 内部用指针复用没变的内容）。所以"回到某个版本"是瞬间的，不是回放差异。

### 0.3 分支只是指向提交的指针

```
        A ── B ── C  ← dev
                    \
                     D ── E  ← feature/xxx
```

`git switch` 只是把 `HEAD` 这个指针挪过去，**不复制任何文件**。这正是"建分支 0 成本"的原因，也是敢随便开分支的底气。

---

## 1. 完整流程（5 个阶段）

### 阶段 1：开工 —— 从 dev 拉最新，建个人分支

```bash
git switch dev
git pull origin dev
git switch -c feature/order-opt
```

| 命令 | 目的 |
| --- | --- |
| `git switch dev` | 切到团队集成分支 |
| `git pull origin dev` | **先同步**，保证后续基于最新代码 |
| `git switch -c feature/xxx` | 建个人功能分支（`-c` = create） |

**验证**：

```bash
git branch -vv
# * feature/order-opt  1dc2c4c  ...
#                           └─ 应该和 dev 指向同一个提交
```

**要点**：分支名带前缀，一眼看出类型：`feature/`（功能）、`fix/`（修复）、`hotfix/`（紧急修复）、`doc/`（文档）。

---

### 阶段 2：开发 —— 频繁小提交

```bash
git diff                        # ① 提交前自查：改了什么
git add .                       # ② 暂存
git commit -m "feat: 延时函数支持 0ms 快速返回"   # ③ 提交

git add .                       # 发现漏了文件
git commit --amend --no-edit    # ④ 并进上一次提交，不改信息

git commit -m "fix: 降低 SysTick 中断优先级"      # ⑤ 下一个提交
git log --oneline --graph -4    # ⑥ 看历史形状
```

| 命令 | 关键点 |
| --- | --- |
| `git diff` | **在 `add` 之前看**，才能看到真实改动。`add` 之后要看用 `git diff --staged` |
| `git add .` | 暂存所有改动 |
| `git commit -m "..."` | 消息必须在引号里，**漏引号会打开编辑器卡住** |
| `git commit --amend --no-edit` | **补提交利器**：刚提交发现漏文件，用它而不是再开一个"修补"提交 |
| `git log --oneline --graph` | `--graph` 才能看到分叉，`--oneline` 一行一个 |

**提交信息规范**（前缀必须小写 + 冒号 + 空格）：

| 前缀 | 用于 |
| --- | --- |
| `feat:` | 新功能 |
| `fix:` | 修复 bug |
| `docs:` | 文档 |
| `style:` | 格式（无逻辑变更） |
| `refactor:` | 重构（不改行为） |
| `perf:` | 性能优化 |
| `test:` | 测试 |
| `chore:` | 工程杂项（构建、依赖） |

**为什么不嫌提交多**：提交是本地操作，**不推送不影响任何人**。小提交的价值是出问题时能精准回退到某一个点。

---

### 阶段 3：同步上游 —— rebase 解决冲突 ⭐ 核心步骤

开发几天后，dev 上已经有别人的新提交了。要先把自己的提交"搬到"最新 dev 之上。

```bash
git switch dev
git pull origin dev                  # ① 更新 dev
git switch feature/order-opt         # ② 回到自己的分支
git rebase dev                       # ③ 变基
```

**rebase 做了什么**：

```
变基前：
        A ── B ── C ── D          ← dev（别人的 D）
              \
               E ── F              ← feature（你的）

变基后：
        A ── B ── C ── D
                       \
                        E' ── F'   ← 你的提交被"重放"到 D 之后
```

**⚠️ `E'` / `F'` 是新的提交（哈希变了）**，这就是 rebase 会"重写历史"的含义。

#### 冲突了怎么办

```bash
git status                  # ① 看哪些文件冲突（Unmerged paths 段）
git --no-pager diff         # ② 看冲突内容（--no-pager 防止卡在分页器）
# ③ 手动编辑文件，处理冲突标记
git add <冲突的文件>         # ④ 标记为已解决
git rebase --continue       # ⑤ 继续
```

**冲突标记长这样**：

```
<<<<<<< HEAD
这是 dev 上的内容（别人写的）
=======
这是你分支上的内容
>>>>>>> feature/order-opt
```

处理方式：**删掉三行标记，只留下最终想要的内容**（可以是其中一方，也可以是两者合并）。

**放弃 rebase**：

```bash
git rebase --abort          # 完整回到 rebase 之前，什么都不丢
```

#### rebase 之后必做的两件事

```bash
git --no-pager log --oneline --graph -5   # ① 确认历史是直线，提交都在
git status -sb                            # ② -sb = 简短 + 显示上游关系
```

#### ⚠️ rebase 的铁律

| 分支类型 | 能否 rebase |
| --- | --- |
| **个人功能分支** | ✅ 随便 rebase |
| **dev / main / master（共享分支）** | ❌ **绝对不要** —— 会改掉所有人的历史 |

如果 rebase 过个人分支后需要推送，用：

```bash
git push --force-with-lease
```

**不要用 `--force`** —— `--force-with-lease` 会在远端有别人新提交时拒绝覆盖，`--force` 会直接抹掉。

---

### 阶段 4：推送 + 提 MR

```bash
git add <遗漏的文件>
git commit --amend --no-edit                          # 补进上一次提交
git --no-pager show HEAD -- <文件路径>                 # 验证确实进去了
git status -sb
git push -u origin feature/order-opt                  # 推送
```

**`-u` = `--set-upstream`**，建立跟踪关系。之后在这个分支上直接 `git push` 就行，不用再写 `origin feature/xxx`。

**验证**：

```bash
git status -sb
# ## feature/order-opt...origin/feature/order-opt   ← 没有 ahead/behind 就是同步了
```

然后在**网页上提 MR（Merge Request）/ PR**：

```
源分支：feature/order-opt
目标分支：dev
```

**MR 描述建议模板**：

```markdown
## 改了什么
（一两句话）

## 为什么
（解决什么问题）

## 怎么验证
（复现/测试步骤）

## 风险
（可能影响哪些功能）
```

---

### 阶段 5：合并后清理

```bash
git switch dev
git pull origin dev                            # ① 拉下已合并的代码
git branch -d feature/order-opt                # ② 删本地分支
git push origin --delete feature/order-opt     # ③ 删远端分支
git fetch --prune                              # ④ 清理本地残留的远端引用
git branch -avv                                # ⑤ 确认干净
git --no-pager log --oneline --graph -8
```

| 命令 | 说明 |
| --- | --- |
| `git branch -d` | **`-d` 会检查是否已合并**，未合并会拒绝（安全） |
| `git branch -D` | 强制删除，**可能丢提交**（谨慎） |
| `git push origin --delete <分支>` | 删远端分支 |
| `git fetch --prune` | 清掉本地 `origin/xxx` 里远端已不存在的引用 |

---

## 2. 决策图：遇到情况怎么选

```
准备好提交了吗？
├─ 还没 → git diff 看改动 → git add . → git commit
└─ 刚提交发现漏了文件
   └─ git add . → git commit --amend --no-edit     ← 不要新开一个"修补"提交

要推送了，但 dev 已经更新了？
├─ 先 git switch dev → git pull → git switch 回来 → git rebase dev
└─ 冲突了
   ├─ 能解决 → 改文件 → git add → git rebase --continue
   └─ 搞不定 → git rebase --abort（回到起点，不丢任何东西）

推送被拒绝（non-fast-forward）？
├─ 个人分支且 rebase 过 → git push --force-with-lease
└─ 共享分支 → 先 git pull 再推，绝不能 force

想撤销？
└─ 见第 5 节
```

---

## 3. 演示过程中的 4 个真实笔误（值得警惕）

这些是实际操作时打错的命令。**共同点：git 只会报错，不会损坏仓库，重敲即可。**

| 错的 | 正确的 | 报错 | 教训 |
| --- | --- | --- | --- |
| `git commit -m` | `git commit -m "消息"` | 打开编辑器卡住 | `-m` 后面必须有引号包住的消息 |
| `git commot -m "..."` | `git commit -m "..."` | `git: 'commot' is not a git command` | 拼写错误 |
| `git add add 05_uart/hardware/clock.c` | `git add 05_uart/hardware/clock.c` | `pathspec 'add' did not match` | 多打一个 `add`，第一个被当成路径 |
| `git swtich -c dev` | `git switch -c dev` | `git: 'swtich' is not a git command` | 拼写错误 |

**防错建议**：

```bash
# ① 用 Tab 补全分支名/文件名（少打错字）
git switch feat<Tab>

# ② 命令不确定时先 --help
git rebase --help

# ③ 危险命令先看有没有 dry-run
git clean -n          # 只看会删什么，不真删
```

---

## 4. 本工程（MSPM0G3507_DRV8316）的特殊注意点

### 4.1 已在 `.gitignore` 里忽略的（正确）

```
Debug/  Release/              # CCS 构建输出
*.o  *.d  *.map  *.out        # 中间产物
subdir_rules.mk  subdir_vars.mk  etc.   # 自动生成的 makefile
```

**队友 clone 之后**：直接打开 CCS → 右键工程 → **Refresh（F5）**，CCS 会重新生成 `Debug/` 下所有文件。

**⚠️ 新增 `.c` 文件后**，必须 Refresh 或者手工往 `Debug/hardware/subdir_vars.mk` 里加一行，否则不会参与编译。

### 4.2 最容易冲突的文件 ⚠️

| 文件 | 为什么会冲突 | 对策 |
| --- | --- | --- |
| **`.cproject`** | 每个人的 include 路径 / 编译选项可能不同 | **尽量少改**；改之前先 `git pull`；冲突时仔细看是路径还是选项 |
| **`.syscfg`** | 图形化配置，改一次整个文件可能重排 | 提交前先同步；小改动优先 |
| `empty.syscfg` | 同上 | 同上 |
| `.settings/` | Eclipse/CCS 的工作区偏好 | 一般不需要改 |

### 4.3 生成文件与源文件的关系

```
empty.syscfg  ──SysConfig──►  Debug/ti_msp_dl_config.c/.h  （生成，已忽略）
                             Debug/device_linker.cmd       （生成，已忽略）
```

**只提交 `empty.syscfg`，不要提交生成的文件。**

### 4.4 行尾问题（CRLF / LF）

Windows 下 git 一直会警告：

```
warning: in the working copy of 'xxx.c', LF will be replaced by CRLF
```

多人协作时如果不统一，会出现"整文件都变了"的假冲突。**建议加 `.gitattributes`**：

```
* text=auto eol=lf
*.c     text eol=lf
*.h     text eol=lf
*.md    text eol=lf
*.syscfg text eol=lf
```

### 4.5 不要提交的东西

| 类型 | 例子 |
| --- | --- |
| 构建产物 | `*.out` `*.o` `*.map` |
| 备份文件 | `*.bak` `*.bak-*` `*.orig` |
| 编辑器/浏览器残留 | `README.html`（网页副本）、`content.txt`（对话记录） |
| 大文件 | 固件镜像、视频、PDF 手册 |

---

## 5. 撤销操作速查（能救命）

| 想做什么 | 命令 | 危险度 |
| --- | --- | --- |
| 撤销工作区改动（未 add） | `git restore <file>` | 🔶 改动会丢 |
| 撤销 add（未 commit） | `git restore --staged <file>` | ✅ 安全 |
| 改最后一次提交信息 | `git commit --amend` | ✅ 安全（**已推送的别用**） |
| 撤销某次提交（生成新提交） | `git revert <hash>` | ✅ 安全，可用于已推送 |
| 回到某次提交，**保留改动在暂存区** | `git reset --soft <hash>` | ✅ 安全 |
| 回到某次提交，**保留改动在工作区** | `git reset --mixed <hash>` | ✅ 安全 |
| 回到某次提交，**丢弃改动** | `git reset --hard <hash>` | ⚠️ 危险 |
| 撤销一个文件的改动到某次提交 | `git restore --source=<hash> <file>` | ✅ 安全 |
| **撤销任何误操作** | **`git reflog`** | ✅ 救命稻草 |

### `git reflog` —— 最重要的一条

它记录 **HEAD 去过的每一个位置**，包括被 `reset` 丢弃的提交。

```bash
git reflog
# 61edc2c HEAD@{10-28 14:04}: commit: test:test
# 683383c HEAD@{10-28 14:03}: checkout: moving from dev to test/test
# 683383c HEAD@{10-28 14:01}: reset: moving to 6833       ← 刚才手滑删掉的
```

找回：

```bash
git reset --hard 683383c
```

> **只要提交过，就几乎不可能真正丢代码**（除非删了 `.git` 目录）。
> 未提交的改动才是真会丢的 —— 所以**勤提交**。

---

## 6. SSH 免密配置（Gitea）

### 6.1 生成密钥

```powershell
ssh-keygen -t ed25519 -C "你的邮箱"
```

三个提示**全部直接回车**：

```
Enter file in which to save the key (...):     ← 回车
Enter passphrase (empty for no passphrase):    ← 回车
Enter same passphrase again:                   ← 回车
```

### 6.2 复制公钥到剪贴板

```powershell
Get-Content $env:USERPROFILE\.ssh\id_ed25519.pub | Set-Clipboard
```

> ⚠️ 不要手动选中复制 —— 终端折行会带进多余换行，粘贴到网页会报"密钥格式无效"。

### 6.3 添加到 Gitea

```
Gitea → 右上头像 → 设置 → SSH / GPG 密钥 → 添加密钥
  密钥名称：随便（建议写设备名，如"工作笔记本"）
  密钥内容：Ctrl+V
```

### 6.4 测试

```bash
ssh -T -p 2222 git@服务器地址
```

**首次连接**会问指纹：

```
Are you sure you want to continue connecting (yes/no/[fingerprint])?
```

输 **`yes`**（完整的三个字母，`y` 不认）。

**成功标志**：

```
Hi 你的账号! You've successfully authenticated, but Gitea does not provide shell access.
```

`does not provide shell access` 是**正常的** —— Gitea 只提供 git 操作，不给服务器命令行。

### 6.5 配置 remote

**非 22 端口必须用 `ssh://` 形式**：

```bash
# ✅ 正确（2222 端口）
git remote add origin ssh://git@服务器:2222/账号/仓库.git

# ❌ 错误（简写形式只能用于 22 端口）
git remote add origin git@服务器:2222/账号/仓库.git
```

**验证**：

```bash
git remote -v
ssh -T -p 2222 git@服务器地址    # 确认认证身份
```

> ⚠️ **`ssh -T` 返回的账号名必须和仓库所有者一致**，否则推送会失败：
> ```
> remote: error: User permission denied for writing.
> ```
> 这是**账号没有写权限**，不是 SSH 问题 —— 需要仓库所有者把你加为协作者，或者推到你自己名下的仓库。

---

## 7. `pull` / `fetch` / `rebase` 的区别

| 命令 | 做什么 | 影响工作区 |
| --- | --- | --- |
| `git fetch origin` | 只下载远端提交到本地缓存（`origin/xxx`） | ❌ 不动 |
| `git pull origin dev` | = `fetch` + `merge` | ✅ 会改 |
| `git pull --rebase` | = `fetch` + `rebase` | ✅ 会改，但历史保持直线 |

### 远端分支不能直接切

`origin/dev` 是"只读镜像"。要基于它建本地分支：

```bash
git fetch origin
git switch -c dev origin/dev          # 建本地 dev 并跟踪远端
# 或（Git 会自动识别同名远端分支）
git switch dev
```

### 设置/查看跟踪关系

```bash
# 设置
git branch -u origin/dev
git push -u origin dev                # 推送时顺带设置

# 查看
git branch -vv                        # 看方括号
git config branch.dev.remote          # gitea
git config branch.dev.merge           # refs/heads/dev

# 解除
git branch --unset-upstream dev
```

---

## 8. 常用速查

```bash
# ---- 日常 ----
git status                              # 现在什么状态（一天几十次）
git diff                                # 未 add 的改动
git diff --staged                       # 已 add 的改动
git log --oneline --graph -10           # 最近 10 条 + 图形
git log --oneline -- <文件>             # 某个文件的历史

# ---- 分支 ----
git branch                              # 本地分支
git branch -r                           # 远端分支
git branch -a                           # 全部
git branch -vv                          # 带跟踪信息
git switch <分支>                        # 切换
git switch -c <新分支>                   # 建并切
git switch -c <新分支> <起点>             # 从某处建

# ---- 远端 ----
git remote -v                           # 远端列表
git remote add <名> <url>               # 添加
git remote set-url <名> <新url>          # 改地址
git remote remove <名>                  # 删除
git fetch <名>                          # 只下载
git push -u <名> <分支>                  # 推送并建立跟踪

# ---- 排查 ----
git reflog                              # HEAD 历史（救命用）
git show <hash> --stat                  # 某次提交改了什么
git blame <文件>                         # 每行是谁什么时候改的
git log -S "关键字" --oneline            # 搜哪次提交引入了这段代码

# ---- 暂停分页器（避免卡住）----
git --no-pager <命令>
```

---

## 9. 学习路线（按优先级）

| 阶段 | 内容 | 判断标准 |
| --- | --- | --- |
| **1** | 三个区域 + 提交是快照 + 分支是指针 | 能画图解释 `git switch` 为什么不复制文件 |
| **2** | 日常 10 条命令 | 提交前习惯性 `git diff` |
| **3** | 撤销操作 + `reflog` | 敢用 `reset --hard`，因为知道怎么救 |
| **4** | 分支 + rebase 冲突 | 能独立完成一次带冲突的 rebase |
| **5** | 远端 + MR | 走完一次完整流程 |

**推荐资源**：**Learn Git Branching**（`learngitbranching.js.org`，有中文）
—— 可视化 + 可交互，专门教分支模型。做完前 3 关，分支理解就超过大多数人。

---

## 10. 一页速记

> **开工**：`switch dev` → `pull` → `switch -c feature/xxx`
> **开发**：`diff` → `add` → `commit -m "feat: ..."` （漏了就 `--amend --no-edit`）
> **同步**：`switch dev` → `pull` → `switch 回` → `rebase dev` → 冲突就 `add` + `rebase --continue`
> **提交**：`push -u origin feature/xxx` → 网页提 MR
> **清理**：`switch dev` → `pull` → `branch -d` → `push origin --delete` → `fetch --prune`
>
> **铁律**：共享分支永不 rebase；已推送的永不 `--amend`；出事了先 `git reflog`。
