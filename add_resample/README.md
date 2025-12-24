# リサンプリング機能追加用ファイル

es_slamのスキャン点リサンプリング機能をbackup/newFuzzy1に追加するためのファイル群です。

## フォルダ構成

```
add_resample/
├── README.md           # このファイル
├── common/             # 共通ファイル（両バージョンで使用）
│   ├── resampler.h     # ★新規追加（コピー先: slam.hと同じフォルダ）
│   └── resampler.cpp   # ★新規追加（プロジェクトに追加）
├── backup/             # backup版への変更パッチ
│   ├── Def.h.patch
│   ├── main.cpp.patch
│   ├── slam.cpp.patch
│   ├── slam.h.patch
│   └── Setting.ini.patch
└── newFuzzy1/          # newFuzzy1版への変更パッチ
    └── slam.cpp.patch  # ※他はbackupと同一
```

## 適用手順

### 1. 新規ファイルをコピー

`common/`フォルダ内のファイルを対象プロジェクトにコピー:

```
common/resampler.h   → Kinden_iRobot_slam_v1/resampler.h
common/resampler.cpp → Kinden_iRobot_slam_v1/resampler.cpp
```

### 2. Visual Studioプロジェクトに追加

`resampler.cpp`をプロジェクトのソースファイルに追加

### 3. パッチを適用

各`.patch`ファイルの内容に従って、既存ファイルを編集:

| ファイル | 変更内容 |
|----------|----------|
| Def.h | extern変数とデフォルト値を追加 |
| main.cpp | グローバル変数定義とSetting.ini読み込みを追加 |
| slam.h | `#include "resampler.h"`を追加 |
| slam.cpp | リサンプリング呼び出しを追加 |
| Setting.ini | パラメータを追加 |

### 4. Setting.ini設定

```ini
[ROBOT]
RESAMPLEMODE=1          ; 0=OFF, 1=ON
RESAMPLE_DTHRES=50      ; 点間隔閾値 [mm]
RESAMPLE_DTHREL=250     ; 最大間隔閾値 [mm]
```

## 動作確認

1. `RESAMPLEMODE=0`で従来動作と同一であることを確認
2. `RESAMPLEMODE=1`でリサンプリングが動作することを確認
3. 必要に応じてDTHRES/DTHRELを調整

## 備考

- 単位は全て[mm]（es_slamは[m]だったが変換済み）
- デフォルト値: 50mm間隔、最大250mm間隔
- slam()とslam2()の両方に対応
