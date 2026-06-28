# 部署 BluePilot 到 comma four（public repo + installer / 車機 git）

> 目的：把本機（Windows）改好的 BluePilot 程式碼，部署到一台 **comma four** 上。
> 部署 repo `https://github.com/sevenbai/openpilot` 目前是 **public**，
> 所以車機安裝/更新都**不需要 token**。

---

## 0. 為什麼是這個做法（先看限制）

這幾條限制是從原始碼實測確認的，不是猜測：

1. **comma four 的 binary 只能在 comma 裝置本身上編譯。**
   camerad / modeld 等元件依賴裝置上的高通 GPU/DSP/SNPE 函式庫（`third_party/`），
   openpilot 也不支援從 x86 交叉編譯。Windows / PC / Mac 都做不出能在車機上跑的 binary。

2. **不可以直接把 Windows 的工作目錄 rsync/scp 到車機。**
   本機 checkout `core.autocrlf = true` 且 repo 沒有 `.gitattributes`，
   shell script（`launch_chffrplus.sh`、`launch_env.sh` …）在工作目錄裡是 **CRLF 換行**。
   直接複製過去，車機開機會出現 `/usr/bin/env bash\r: bad interpreter`，**車機開不起來**。
   → 必須讓**車機自己 git checkout**（Linux 端換行才會正確是 LF）。

3. **comma four 的內部代號是 `mici`**，屬於 AGNOS / tici 家族，bp-6.0 已完整支援。
   （`cereal/log.capnp` `DeviceType.mici`、`system/hardware/tici/hardware.py`、`system/ui/mici_setup.py`）

4. **此分支需要 AGNOS 16**（`launch_env.sh` 的 `AGNOS_VERSION="16"`）。

結論：**讓車機自己拉你的 GitHub repo 並在車機上編譯**，是唯一安全又簡單的路。

---

## 1. 整體流程概覽

```
Windows 端                          comma four 端
──────────                          ────────────
push orphan 快照
   │
   ▼
github.com/sevenbai/openpilot (public, bp-6.0)
   │
   ├──(A) installer 全新安裝 ── installer.comma.ai/sevenbai/bp-6.0
   │
   └──(B) SSH + git 就地切換 ── scripts/deploy_branch.sh
                                   │
                                   ▼
                          scons 在車機上編譯 → reboot
```

之後要再更新：本機 `.\scripts\push_deploy_snapshot.ps1` 推一版新快照 → 車機在線時自動 OTA。

---

## 2. Windows 端（推一版到 GitHub）

### 2.1 部署 repo
`https://github.com/sevenbai/openpilot`（**public**，本機 git remote 名稱 `mine`）。

- ⚠️ repo **必須叫 `openpilot`**，installer 短網址（見 3.A）才找得到——
  `installer.comma.ai/<使用者>/<branch>` 會固定去 clone `github.com/<使用者>/openpilot`。
- 它的 `bp-6.0` 分支是**無歷史的 orphan 快照**，純供車機部署，與本機/公開 BluePilotDev 的 `bp-6.0` 是獨立兩條線。

### 2.2 把改好的快照推上去（重要：不能直接 `git push`）
remote 已設好（`mine` → `https://github.com/sevenbai/openpilot.git`）。

> ⚠️ **不要用 `git push mine bp-6.0`。** 這個 repo 的歷史 commit 內含 Git LFS 物件，
> 把整個分支推上去會被 GitHub 伺服器端以 **GH008（unknown Git LFS objects）**
> 的 pre-receive hook 拒收；`--no-verify` 也沒用（那只跳過本機 hook，伺服器端照擋）。
> 這跟 repo 是 public 還是 private **無關**。
>
> 車機只需要分支的 **tip**，而 tip 沒有任何 LFS 檔，所以正確做法是推一個
> **沒有歷史的 orphan 快照**（無 LFS、體積小、不觸發 GH008）。

用內附腳本一行推一版（推薦）：

```powershell
.\scripts\push_deploy_snapshot.ps1
```

它會：用 `HEAD` 的 tree 建一個無 parent 的快照 commit → 確認無 LFS → force-push 成
`mine/bp-6.0`。常用選項：`-Branch <名稱>`、`-Message "說明"`、`-DryRun`（只建不推）、`-Remote <名稱>`。

> 等價的手動兩行（PowerShell）：
> ```powershell
> $snap = git commit-tree 'HEAD^{tree}' -m 'bp-6.0 deploy snapshot'
> git push mine --force "${snap}:refs/heads/bp-6.0"
> ```
> `'HEAD^{tree}'` 與 `${snap}:` 一定要加引號，否則 PowerShell 會誤解析 `^{}`、`:`。
> 車機端不論 installer 或 OTA 都是 `git checkout --force`，所以遠端 `bp-6.0`
> 每次被新快照覆蓋都沒問題。

---

## 3. 車機端：兩種安裝方式

### 3.A installer 全新安裝（最簡單，免 SSH、免 token）
在車機的設定 / 重裝流程，「自訂軟體 URL」輸入：

```
installer.comma.ai/sevenbai/bp-6.0
```

- 它會 clone `github.com/sevenbai/openpilot` 的 `bp-6.0`（這就是為什麼 repo 要叫 `openpilot`）。
- 因為是 public，**不需要任何 token 或 SSH**。
- 屬**全新安裝**（會重裝 openpilot），裝完在車機上編譯（首次約 20–40 分鐘）。
- 適合：第一次裝、從別的 fork 切過來、想乾淨重來。

### 3.B SSH + git 就地切換（不重裝、保留現有安裝）
若不想重裝、只想把現有 `/data/openpilot` 切到你的 repo：

1. 車機 → **Settings → 開發者(Developer)** → 開啟 **Enable SSH**、**SSH Keys** 填你的 GitHub 帳號。
2. 從 Windows：`ssh comma@<車機IP>`（使用者固定 `comma`，標準 22 port）。
3. 貼這段（public，**不用 token**）：

```bash
cd /data/openpilot
git remote set-url origin https://github.com/sevenbai/openpilot.git
git fetch origin bp-6.0
git checkout -f -B bp-6.0 origin/bp-6.0
git submodule update --init --recursive
rm -f prebuilt          # 確保開機會自編譯
scons -j8               # 先手動編譯，能當場看到錯誤（建議）
sudo reboot
```

或用一鍵腳本（public 不必加 `--token`）：

```bash
cd /data/openpilot
bash scripts/deploy_branch.sh --repo https://github.com/sevenbai/openpilot.git --branch bp-6.0
```

腳本會自動：設定 origin → fetch/checkout → 更新 submodule → 移除 `prebuilt`
→ 檢查 AGNOS 版本 → 關閉省電模式 → `scons`（失敗會自動降低 `-j` 重試）→ 詢問是否 reboot。
常用選項：`--no-build`、`--no-reboot`、`-y`、`--help`。

> 注意：3.B 用腳本時，第一次得先手動貼上面那段 git 指令（因為 `deploy_branch.sh`
> 是切 origin、checkout 之後才會出現在車機上）。若嫌麻煩，直接用 3.A installer 就沒這問題。

重開後 `launch_chffrplus.sh` 偵測到沒有 `prebuilt` → 跑 `system/manager/build.py` → scons 編譯 → 啟動 manager。

---

## 4. 兩個必須知道的行為（皆已從原始碼確認）

### 4.1 AGNOS 版本
- 此分支要求 **AGNOS 16**。
- 若車機目前是舊版，第一次開機 `agnos_init` 會自動下載並刷新 AGNOS 16，再重開一次
  （`launch_chffrplus.sh:23-30`）。第一次會多花一些時間，屬正常。

### 4.2 你的改動不會被 updater 洗掉
- `launch_chffrplus.sh:46-49`：launcher 偵測到 `/data/openpilot` 的 `.git` 比 `.overlay_init` 新，
  就會**跳過 overlay 更新**（保護本機開發/切分支的工作）。
- OTA updater（`system/updated/updated.py`）是去拉 **`origin` 的目標 branch**。
  裝好後 origin 指向 `sevenbai/openpilot`，所以它只會跟你的 `bp-6.0` 同步，不會回去拉公開 BluePilotDev。

---

## 5. 彩蛋：之後遠端 OTA（解決「車機不在手」）

一旦車機裝好（不論用 3.A 或 3.B）、origin 指向 `sevenbai/openpilot`、checkout 在 `bp-6.0`：

> 以後只要在本機跑 `.\scripts\push_deploy_snapshot.ps1` 推一版新快照，
> **車機在線時就會自己 OTA 拉取並編譯**，完全不用再碰車機、也不用再 SSH。

也就是說：**裝好一次之後，後續更新全遠端搞定。**

---

## 6. 疑難排解

| 症狀 | 原因 / 解法 |
|---|---|
| 開機卡住、log 出現 `bash\r: bad interpreter` | 你把 Windows 工作目錄直接複製過去了（CRLF）。改用本文件的 git / installer 方式。 |
| installer 顯示找不到 / 404 | repo 名稱不是 `openpilot`，或該 branch 不存在。確認 repo 叫 `openpilot`、`bp-6.0` 存在。 |
| `git submodule update` 失敗 | 網路問題。重跑 `git submodule update --init --recursive`（submodule 都是公開 remote，免認證）。 |
| scons 編譯到一半被 OOM kill | `build.py` 會自動以 `nproc → nproc/2 → 1` 重試降低並行度；手動編譯可改 `scons -j4` 或 `-j1`。 |
| 改了 code，車機卻沒更新 | 確認 `git remote -v` 的 origin 是 `sevenbai/openpilot`、branch 是 `bp-6.0`；確認車機在線。 |
| push 被拒：`GH008 ... unknown Git LFS objects` | 別直接 `git push`（歷史含 LFS）。改用 `scripts\push_deploy_snapshot.ps1` 推 orphan 快照。 |
| 想強制整包重來 | 車機上可用 `scripts/force_branch_update.sh <branch>`，或直接走 3.A installer 重裝。 |

---

## 7. 指令快速參考

**Windows 端（更新時）**
```powershell
.\scripts\push_deploy_snapshot.ps1
```

**車機端（A：installer 全新安裝）** — 車機設定流程輸入：
```
installer.comma.ai/sevenbai/bp-6.0
```

**車機端（B：SSH 就地切換，首次）**
```bash
cd /data/openpilot
git remote set-url origin https://github.com/sevenbai/openpilot.git
git fetch origin bp-6.0
git checkout -f -B bp-6.0 origin/bp-6.0
git submodule update --init --recursive
rm -f prebuilt
scons -j8
sudo reboot
```

**車機端（B：之後手動拉最新）**
```bash
cd /data/openpilot
git fetch origin bp-6.0
git checkout -f -B bp-6.0 origin/bp-6.0
git submodule update --init --recursive
scons -j8
sudo reboot
```
