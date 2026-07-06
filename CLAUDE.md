# Plasma CAM プロジェクト

## 概要

自作のプラズマCNC制御＋CAMソフト。DXFを読み込んで配置し、リードイン・
カーフ補正付きのGコードを生成、GRBLにシリアル送信して切断する。
GUIは tkinter + matplotlib、通信は pyserial。機械サイズ 1200×800mm。

## ファイル構成

- `main.py` — アプリ本体（約3300行）
  - `PlasmaCamApp` — メインウィンドウ。DXF読み込み/D&D/配置ドラッグ、
    設定タブ、GRBL接続・ジョグ・ストリーミング、Gコードプレビュー
  - `SimWindow` — トーチ動作シミュレーション。ホイールで拡大縮小
    （カーソル位置基準）、ドラッグで移動、ダブルクリックで全体表示。
    軌跡は機械座標で `_trace` に記録し、ズーム/リサイズ時に再描画する
  - `CamEditorWindow` — 切断順序・リードイン方向の手動編集
- `gcode_generator.py` — Gコード生成の中核
  - `generate_gcode()` — 自動順序で生成（現在は主にプラン経由が使われる）
  - `generate_from_plan()` — CAM計画（順序・リードイン方向指定）から生成
  - `_hierarchical_order()` — 入れ子深さで穴/外形判定＋切断順序（内→外）
- `dxf_reader.py` — DXF読み込み。エンティティ→Segment→Path に連結（chain_segments）
- `kerf_offset.py` — カーフ補正（shapelyオフセット。穴=縮小、外形=拡大）
- `settings_manager.py` — 設定/プリセットの保存（pierce_settings.json）
- `layouts.json` — 画面レイアウト保存
- `PlasmaCam.spec` / `setup.bat` / `start_app.bat` — PyInstallerビルドと起動
- 依存: pyserial, matplotlib, shapely, ezdxf, tkinterdnd2

## 重要な仕様

### 穴/外形の判定とリードイン

- `_hierarchical_order()` が輪郭の入れ子深さで判定：奇数深さ=穴、偶数=外形
- 通常モード: 穴→内側（捨て材）にピアス、外形→外側にピアス。
  カーフ補正も同じフラグで向きが決まる
- **ステンシルモード**（設定タブのチェックボックス、2026-07追加）:
  板が製品で全輪郭が「板の穴」になるため判定を反転
  （`_hierarchical_order(..., stencil=True)`）。
  文字切り抜きプロジェクトのステンシルDXFを切るときは必ずON。
  切替時は `_stencil_toggled()` がCAM計画を破棄して再計算させる
- リードイン始点の「内側」は `_find_inside_point()` が計算：
  始点中心の円と輪郭内部の交差から representative_point を取るため、
  C字形など凹んだ輪郭や角の始点でも必ず内部に入る（重心方向の推測は廃止）

### ゴミパスの除外

閉じているのに面積ほぼゼロ（0.05mm²未満）のパスは切断対象から除外。
DXF由来のゴミのほか、`chain_segments` が尖った先端（三日月の先など）で
輪郭を割って作る3点のカケラもこれで無害化される。開いた線分は普通に切る。

### CAM計画（plan）

`get_cam_plan()` が自動生成、CamEditorWindow で手動編集可能。
plan要素: `{'entry', 'path_idx', 'path', 'is_inner', 'leadin', 'label'}`。
`leadin` は 'inside'/'outside'。JSON保存/読み込み対応。
DXF変更・ステンシル切替で `_cam_plan = None` にリセットされる。

### Gコード

G21/G90/G94、M3/M5、ピアス待機 G4、Z制御は相対移動（G91で上下）。
円弧は G2/G3（GRBLの円弧誤差 error:8 を避けるため丸め後の値で検証し、
超える場合はG1近似にフォールバック）。

## 関連プロジェクト

- `../文字切り抜き` — 文字ステンシルDXFの生成元。ここのDXFを切るときは
  ステンシルモードをONにする
- `../地図を切断` — 都道府県輪郭のSVG/DXF生成

## 未解決問題

- **切断途中で止まる問題**（2026-06発生、2026-07時点で未解決）。
  診断ログは強化済み（50行ごとの進捗表示、エラー/ALARMの目立つ表示、完了メッセージ）。
  想定される原因: error:15（ソフトリミット超過）／ALARM（ハードリミット）／USB切断。
  次に実機で切断したら「コンソールに何が表示されたか」をユーザーに確認し、原因を絞り込むこと。

## 注意

- git管理されている。コミットはユーザーの指示があったときだけ
- 実機（GRBL）が無い環境でも「オフラインモード」でCAM機能は試せる
- 設定は `pierce_settings.json` に保存される（stencil_modeも永続化される。
  普通の部品を切る前にOFFに戻すこと）
