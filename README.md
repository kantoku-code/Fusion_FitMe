# Fusion_FitMe
Make parts fit together

An add-in for Autodesk Fusion that finds assembly candidates between two components from their holes, shafts, slots, cones, and spheres, previews them, and creates an As-built Joint.

---

## Installation

1. Quit Autodesk Fusion.
2. Download and extract the zip file.
3. Run `installer\install_Win.cmd` in the extracted folder.
4. Follow the on-screen instructions and enter `Y`.
5. The "FitMe" folder is copied to `%APPDATA%\Autodesk\Autodesk Fusion 360\API\AddIns\FitMe`.

To uninstall, delete the copied "FitMe" folder.

---

## Usage

1. Open a design and run **FitMe** from the "SOLID" tab → "ASSEMBLE" panel.
2. Choose the **Mode**: **Joint** (default) or **Move only**.
3. Select the **Fixed** component, then the **Moving** component.
4. Candidates are listed with a score, joint type, and description. Click the button at the left of a row to preview it.
5. Turn on **Joint animation** to see the motion of the joint (revolute, slider, etc.).
6. Click **OK** to move the component. In **Joint** mode, an As-built Joint is also created. In **Move only** mode, the position is captured without a joint.

### Detected features

- Cylindrical holes / shafts (metric tapped, clearance, and bolt diameters are matched)
- Slots (pin-slot / slider joints)
- Cones and spheres (ball joints)

Candidates that cause interference are excluded.

---

## 日本語

2つのコンポーネントの穴・軸・長穴・円錐・球から組付け候補を探し、プレビューして As-built ジョイントを作成するアドインです。

### インストール

1. Fusion を終了します。
2. `installer\install_Win.cmd` を実行し、`Y` を入力します。
3. `%APPDATA%\Autodesk\Autodesk Fusion 360\API\AddIns\FitMe` にコピーされます。

アンインストールはコピーされた「FitMe」フォルダーを削除してください。

### 使い方

1. 「ソリッド」タブ →「組み立て」パネルの **FitMe** を実行します。
2. **モード** を選択します。**ジョイント**(デフォルト)または **移動のみ**。
3. **固定側**、**移動側** のコンポーネントを選択します。
4. 候補がスコア・ジョイントタイプ・説明とともに一覧表示されます。行の左端のボタンで候補を選ぶとプレビューされます。
5. **ジョイントアニメーション** をONにすると、ジョイントの動き(回転・スライダ等)を確認できます。
6. **OK** で移動側を配置します。**ジョイント** モードでは As-built ジョイントも作成し、**移動のみ** モードではジョイントを作らず位置をキャプチャします。
