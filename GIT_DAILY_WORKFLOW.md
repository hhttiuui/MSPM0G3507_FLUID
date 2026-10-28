# Git 日常协作流程（开工 / 收工 / 发布）

> 配套文档：`GIT_WORKFLOW.md`（那份讲**原理与命令全集**）。
> 本文件只讲两件事：**一天怎么过**，**一次发布怎么走**。
> 多人协作与单人开发通用，区别只在"评审"这一步有没有第二个人。

---

## 0. 分支职责

| 分支 | 谁在写 | 规则 |
| --- | --- | --- |
| `dev` | 所有人，通过 MR 合入 | 只做 `pull --rebase`，**永不 `rebase`** |
| `feature/*` `fix/*` `doc/*` | 只有你自己 | 随便 `rebase`，重推用 `--force-with-lease` |
| `main` | 只在发布时由负责人合入 | **只接受来自 `dev` 的合入**，不接个人分支 |

一句话：**`dev` 是"大家的东西"，`feature/*` 是"我的东西"，`main` 是"能用的东西"。**

---

## 1. 三条铁律

1. **共享分支（`dev` / `main`）只 `pull --rebase`，永远不 `rebase`。**
   这两个分支的历史是所有人共用的，改它等于改掉所有人的历史。
2. **`rebase` 只在自己还没被合入的个人分支上做。**
   已经推送过的分支重推，必须用 `--force-with-lease`，**绝不用 `--force`**。
3. **`main` 只从 `dev` 合入，日常开发完全不碰 `main`。**

---

## 2. 开工（每天早上，约 2 分钟）

```bash
git switch dev
git pull --rebase origin dev        # 拿到队友昨天推的提交
git switch -c feature/ganim-new     # 今天要干的这件事，一个分支一件事
```

**验证**：`git branch -vv`，新建的 `feature/ganim-new` 应该和 `dev` 指向**同一个提交哈希**。

**两个坑**：

- **昨天收工没推完** → 不要新建分支，先 `git switch <昨天那个分支>` 接着干。
  一个人同时挂好几个 feature 分支，是冲突的主要来源。
- **`pull` 必须带 `--rebase`**。不带就是 merge，会在 `dev` 上留一个 `Merge branch...` 节点，
  历史变网状，别人 review 时很难看清你到底改了什么。

---

## 3. 白天（频繁小提交，全部留在本地）

```bash
git diff                              # 必须在 add 之前看，add 之后就看不到真实改动了
git add hardware/scene_still.c
git commit -m "feat: 新增星系轨道动图"
```

- 本地提交**不推送就不影响任何人**，所以不用怕提交多。
- 每完成一个能独立描述的小步就提交一次，价值在于出问题时能精确回退到某一个点，
  而不是只能退回"昨天"。
- 漏了文件：`git add <漏的>` + `git commit --amend --no-edit`，不要另开一个"修补"提交。

---

## 4. 收工（下班前，约 5 分钟）

```bash
git status -sb                            # ① 有没有漏掉的文件
git add . && git commit -m "feat: ..."    # ② 收尾提交
git switch dev
git pull --rebase origin dev              # ③ 再同步一次（白天队友可能又推了）
git switch feature/ganim-new
git rebase dev                            # ④ 把今天的提交搬到最新 dev 之上
git push -u origin feature/ganim-new      # ⑤ 推个人分支
```

然后在网页上提 MR：`feature/ganim-new` → `dev`。
**合并动作由评审人做，自己不合自己的 MR。**

> "今天结束"的标志不是代码写完，而是**个人分支已推到远端 + MR 已开**。
> 这样明天电脑坏了、硬盘挂了，工作成果也还在。

---

## 5. 冲突只会在第 ④ 步出现

```bash
git status                 # 看 Unmerged paths 段，哪些文件冲突
# 手动编辑文件，删掉 <<<<<<< / ======= / >>>>>>> 三行标记，留下最终要的内容
git add <冲突文件>
GIT_EDITOR=true git rebase --continue    # 加 GIT_EDITOR 防止卡在编辑器里
git rebase --abort                       # 后悔了，随时可整体放弃，什么都不丢
```

冲突的含义是"**你和队友改了同一段代码**"，解决时**保留双方的意图**，不是二选一。
比如他改了注释、你改了实现，就把新注释和新实现拼在一起。

---

## 6. 异常速查

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| `! [rejected] ... non-fast-forward` | 你 rebase 过，本地哈希和远端不一致 | `git push --force-with-lease`（不是 `--force`） |
| `CONNECT tunnel failed, response 502`<br>`Empty reply from server` | 本机环境变量注入了代理 | `env -u HTTPS_PROXY -u HTTP_PROXY git push ...` |
| rebase 完发现搞错了 | 提交哈希已变 | `git reflog` 找 rebase 前的哈希 → `git reset --hard <hash>` |
| `rebase --continue` 卡住不动 | 打开了编辑器 | `GIT_EDITOR=true git rebase --continue` |
| 不知道该干什么 | —— | `git status -sb` + `git log --oneline --graph --all -15` |

---

## 7. 把 `dev` 合入 `main`（发布）

### 7.1 先确认两个分支是什么关系

```bash
git switch dev && git pull --rebase origin dev
git merge-base --is-ancestor main dev && echo "可以快进" || echo "有分叉，会产生合并提交"
```

- **`main` 是 `dev` 的祖先** → 快进合并，`main` 指针直接滑到 `dev`，不产生新提交，零冲突。
- **有分叉** → 会产生一个合并提交，可能需要在 `main` 上解一次冲突。

本工程当前状态（2026-10-28）是**前者**，`main` 落后 `dev` 4 个提交且完全被包含：

```
dev   ● a594e79  docs: README 更正已完成的功能状态，补 Git 手册索引
      ● 74aacee
      ● b4266a6
      ● 97895ad
main  ● 715ab91  加 brkmode 开关：可切换到变化率边沿判定，默认关闭
```

### 7.2 合并前先确认能编译

```bash
git switch dev
# 在 CCS 里 clean + build 一次，确认 0 error
```

发布前不验证，等于把问题推给 `main`。

### 7.3 方式 A：快进合并（推荐，历史最干净）

```bash
git switch main
git merge --ff-only dev        # --ff-only：不能快进就报错退出，绝不偷偷生成合并提交
git log --oneline -3           # 确认 main 已经指向 dev 的最新提交
```

结果：`main` 和 `dev` 指向同一个提交，历史一条直线。

`--ff-only` 的价值是**保护性参数**——如果分支关系和你以为的不一样，它会直接报错，
而不是默默地给你造一个合并节点。

### 7.4 方式 B：保留一个合并节点（`--no-ff`）

```bash
git switch main
git merge --no-ff dev -m "chore: 发布 v0.2.0（dev 合入 main）"
```

好处是 `git log --graph` 上能一眼看出"哪一批提交构成了一次发布"。
代价是多一个没有实际代码改动的节点。团队规定用哪种就照哪种，别混着来。

### 7.5 推送 `main`

```bash
git push origin main
```

**如果报 `non-fast-forward`**，说明 `main` 的历史被重写过（本工程就是这样），
需要：

```bash
git push --force-with-lease origin main
```

⚠️ **推之前必须确认没有别人基于旧 `main` 工作**。`--force-with-lease` 只能防"远端有你没拉到的新提交"，
防不了"别人已经拉过旧 `main` 并在上面开发"。后者一推就会让别人的分支变成孤儿。

### 7.6 打标签（可选但推荐）

```bash
git tag -a v0.2.0 -m "刹车高亮 + 场景框架 + 文字滚动"
git push origin v0.2.0
```

标签是发布点的永久书签，比记住提交哈希可靠得多。

### 7.7 合并之后

- **`dev` 继续往下开发**，不要删、不要重置。
- **不要反过来把 `main` 合回 `dev`**——除非 `main` 上直接修过紧急 bug（hotfix）。
  正常流程下 `dev` 永远包含 `main` 的全部内容。

### 7.8 如果 `main` 上修了紧急 bug

```bash
# 在 main 上直接修（只有这种情况允许直接改 main）
git switch main
git commit -m "hotfix: 修 xxx 崩溃"
git push origin main

# 回灌到 dev，否则下次发布会把它覆盖掉
git switch dev
git merge main                 # 或 git cherry-pick <hotfix 提交>
git push origin dev
```

---

## 8. 本工程的一次性待办

| 事项 | 命令 | 说明 |
| --- | --- | --- |
| 远端还没有 `dev` 分支 | `git push -u origin dev` | 推一次之后队友才能 `pull origin dev` |
| `main` 历史被重写过 | `git push --force-with-lease origin main` | 只在确认无人基于旧 `main` 工作时做 |

---

## 9. 一页速记

```
开工   switch dev → pull --rebase → switch -c feature/xxx
白天   diff → add → commit -m "feat: ..."
收工   commit → switch dev → pull --rebase → switch 回 → rebase dev
       → push -u origin feature/xxx → 网页提 MR
发布   switch dev → 编译验证 → switch main → merge --ff-only dev → push
```

> **铁律**：共享分支永不 rebase；已推送的永不 `--amend`；出事了先 `git reflog`。
