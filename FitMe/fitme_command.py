# Fusion360API Python add-in
# 2つのOccurrenceの穴/軸から組付け候補を探し、選択した候補をプレビュー → Jointを作成

import bisect
import itertools
import json
import math
import re
import threading
import time
import traceback
from pathlib import Path

import adsk
import adsk.core as core
import adsk.fusion as fusion

from .i18n import tr

_app: core.Application = core.Application.get()
_ui: core.UserInterface = _app.userInterface
_handlers = []  # コマンド毎のハンドラ (destroy 時にクリア)

TOL = 0.001  # 位置の許容値 (cm)
TOL_DIA = 0.05  # 径の許容値 (mm)
TOL_ANG = 1e-6  # 平行判定の許容値
TOL_VOL = 1e-4  # 干渉とみなす体積 (cm^3)
MAX_CANDIDATES = 100
ANIM_TIME = 0.4  # プレビュー切替のアニメーション時間 (秒)
ANIM_FRAMES = 20  # プレビュー切替のアニメーションのフレーム数
ANIM_INTERVAL = 0.03  # ジョイント動作アニメーションのフレーム間隔 (秒)
ANIM_EVENT_ID = "kantoku_fitme_anim"
ANIM_MOTION_TIME = 3.0  # ジョイント動作アニメーションの周期 (秒)
BALL_ANG = math.radians(20)  # ボールジョイントの首振り角
TOL_CONTACT = 0.01  # 平面接触の判定で押し込む量 (cm)
TOL_CONE_ANG = math.radians(0.1)  # 円錐の半角の許容値
MAX_PER_TYPE = 30  # 穴以外の条件毎の候補数の上限
SETTINGS_PATH = Path(__file__).resolve().parent / "resources" / "settings.json"

# ジョイントタイプ → messages.json のキー
JOINT_NAMES = {
    "rigid": "joint_rigid",
    "revolute": "joint_revolute",
    "cylindrical": "joint_cylindrical",
    "pinslot": "joint_pinslot",
    "slider": "joint_slider",
    "ball": "joint_ball",
    "planar": "joint_planar",
}

# (サイズ, 呼び径, 下穴径, めねじ内径, バカ穴径) 単位mm
METRIC_TABLE = [
    ("M2", 2.0, 1.6, 1.567, (2.2, 2.4, 2.6)),
    ("M2.5", 2.5, 2.05, 2.013, (2.7, 2.9, 3.1)),
    ("M3", 3.0, 2.5, 2.459, (3.2, 3.4, 3.5, 3.6)),
    ("M4", 4.0, 3.3, 3.242, (4.3, 4.5, 4.8)),
    ("M5", 5.0, 4.2, 4.134, (5.3, 5.5, 5.8)),
    ("M6", 6.0, 5.0, 4.917, (6.4, 6.6, 7.0)),
    ("M8", 8.0, 6.8, 6.647, (8.4, 9.0, 10.0)),
    ("M10", 10.0, 8.5, 8.376, (10.5, 11.0, 12.0)),
    ("M12", 12.0, 10.2, 10.106, (13.0, 13.5, 14.5)),
    ("M16", 16.0, 14.0, 13.835, (17.0, 17.5, 18.5)),
    ("M20", 20.0, 17.5, 17.294, (21.0, 22.0, 24.0)),
]

# 役割 → messages.json のキー
ROLE_NAMES = {
    "tap": "role_tap",
    "clr": "role_clr",
    "ext": "role_ext",
    "bolt": "role_bolt",
}

# 役割の組み合わせ毎の基本点
ROLE_PAIR_SCORE = {
    frozenset(("tap", "clr")): 100,
    frozenset(("ext", "tap")): 90,
    frozenset(("bolt", "tap")): 90,
    frozenset(("ext", "clr")): 80,
    frozenset(("bolt", "clr")): 80,
    frozenset(("clr",)): 60,
}
SCORE_SHAFT_HOLE = 90
SCORE_HOLE_HOLE = 50
SCORE_MATCH_BONUS = 10
SCORE_OUTER_SEAT = 10
SCORE_IMPROPER_SEAT = -20
SCORE_CONE = 90
SCORE_SPHERE = 80

_cands = []
_threadCache = {}
_tableGen = 0
_curMat: core.Matrix3D = None  # 現在表示中の移動側のワールド行列
_animating = False  # プレビュー切替のアニメーション中
_animAborted = False

# ジョイント動作アニメーション
# (タイマーから doExecutePreview を呼び、プレビュー内で実物を動かす。変更はトランザクションで破棄される)
_cmd: core.Command = None
_animTarget = None  # アニメーション対象の候補 (Noneで停止)
_animStart = 0.0
_animFrom: core.Matrix3D = None
_animPivot: core.Point3D = None  # 移動側の回転中心 (Occurrence内の座標)
_origMat: core.Matrix3D = None  # 移動側の元のワールド行列
_tickBusy = False  # タイマーのイベント処理中 (イベントを溜めない)
_stopFlag: threading.Event = None
_customEvent: core.CustomEvent = None


class Seat:
    def __init__(self, center, normal, edge, isProper, isOuter):
        self.center: core.Point3D = center
        self.normal: core.Vector3D = normal
        self.edge: fusion.BRepEdge = edge
        self.isProper = isProper
        self.isOuter = isOuter


class HoleFeat:
    def __init__(self, kind, dia, origin, axis):
        self.kind = kind  # 'hole' / 'shaft'
        self.dia = dia  # mm
        self.origin: core.Point3D = origin
        self.axis: core.Vector3D = axis
        self.seats = []
        self.thread = None
        self.roles = set()
        self.isSlot = False  # 長穴の場合 origin は長穴中心
        self.slotDir: core.Vector3D = None  # 長穴の方向
        self.slotEdge: fusion.BRepEdge = None  # 長穴の方向を示す直線エッジ
        self.slotLen = 0.0  # 長穴の円弧中心間の距離 (cm)
        self.shape = "cyl"  # 'cyl' / 'cone'  円錐の場合 origin は頂点, axis は開口方向
        self.halfAngle = 0.0  # 円錐の半角 (rad)
        self.rRange = (0.0, 0.0)  # 円錐の半径の範囲 (cm)

    def transformBy(self, mat: core.Matrix3D):
        self.origin.transformBy(mat)
        self.axis.transformBy(mat)
        if self.slotDir:
            self.slotDir.transformBy(mat)
        for s in self.seats:
            s.center.transformBy(mat)
            s.normal.transformBy(mat)


class SphereFeat:
    def __init__(self, kind, center, radius, face):
        self.kind = kind  # 'hole'(凹) / 'shaft'(凸)
        self.center: core.Point3D = center
        self.radius = radius  # cm
        self.face: fusion.BRepFace = face

    def transformBy(self, mat: core.Matrix3D):
        self.center.transformBy(mat)


class PartFeats:
    # 1つのOccurrenceから抽出した形状
    def __init__(self):
        self.holes = []  # HoleFeat (円筒/長穴/円錐)
        self.spheres = []  # SphereFeat

    def transformBy(self, mat: core.Matrix3D):
        for f in self.holes + self.spheres:
            f.transformBy(mat)


class Candidate:
    def __init__(
        self,
        mat,
        matched,
        score,
        label,
        edge,
        isFree,
        normal,
        jointType=None,
        geoKind="curve",
        geoEntity=None,
    ):
        self.mat: core.Matrix3D = mat
        self.matched = matched
        self.score = score
        self.label = label
        self.edge: fusion.BRepEdge = edge
        self.isFree = isFree
        self.normal: core.Vector3D = normal  # 固定側座面の法線
        self.jointType = jointType  # None の場合は穴の一致状態から決める
        self.slotEdge: fusion.BRepEdge = None
        self.geoKind = geoKind  # Joint基準 'curve' / 'planar' / 'nonplanar'
        self.geoEntity = geoEntity if geoEntity else edge
        # ジョイント動作アニメーション用(ワールド座標)
        self.motionOrigin: core.Point3D = None  # 回転中心
        self.motionAxis: core.Vector3D = None  # 回転軸
        self.motionSlide: core.Vector3D = None  # 長穴のスライド方向
        self.motionLen = 0.0  # スライドの振幅 (cm)
        self.slotMoving = False  # 長穴が移動側にある


class MyCommandCreatedHandler(core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args: core.CommandCreatedEventArgs):
        core.Application.get().log(args.firingEvent.name)
        try:
            global _handlers, _cmd, _curMat, _origMat
            cmd: core.Command = core.Command.cast(args.command)
            _cmd = cmd
            # アドインはモジュールが常駐する為、前回の状態をリセットする
            _handlers.clear()
            _curMat = None
            _origMat = None
            stop_joint_anim()
            inputs: core.CommandInputs = cmd.commandInputs

            onDestroy = MyCommandDestroyHandler()
            cmd.destroy.add(onDestroy)
            _handlers.append(onDestroy)

            onExecute = MyExecuteHandler()
            cmd.execute.add(onExecute)
            _handlers.append(onExecute)

            onExecutePreview = MyExecutePreviewHandler()
            cmd.executePreview.add(onExecutePreview)
            _handlers.append(onExecutePreview)

            onInputChanged = MyInputChangedHandler()
            cmd.inputChanged.add(onInputChanged)
            _handlers.append(onInputChanged)

            onValidateInputs = MyValidateInputsHandler()
            cmd.validateInputs.add(onValidateInputs)
            _handlers.append(onValidateInputs)

            ddMode: core.DropDownCommandInput = inputs.addDropDownCommandInput(
                "ddMode", tr("mode_label"), core.DropDownStyles.TextListDropDownStyle
            )
            # 前回の設定を読み込む
            settings = load_settings()
            modeIdx = 1 if settings.get("mode") == 1 else 0
            ddMode.listItems.add(tr("mode_joint"), modeIdx == 0)
            ddMode.listItems.add(tr("mode_move"), modeIdx == 1)

            selA: core.SelectionCommandInput = inputs.addSelectionInput(
                "selA", tr("fixed_label"), tr("fixed_prompt")
            )
            selA.addSelectionFilter("Occurrences")
            selA.setSelectionLimits(1, 1)

            selB: core.SelectionCommandInput = inputs.addSelectionInput(
                "selB", tr("moving_label"), tr("moving_prompt")
            )
            selB.addSelectionFilter("Occurrences")
            selB.setSelectionLimits(1, 1)

            tblCand: core.TableCommandInput = inputs.addTableCommandInput(
                "tblCand", tr("candidates_label"), 4, "2:2:2:8"
            )
            tblCand.maximumVisibleRows = 15
            tblCand.minimumVisibleRows = 5

            inputs.addBoolValueInput(
                "chkAnim", tr("anim_label"), True, "", settings.get("anim") is True
            )

            inputs.addTextBoxCommandInput(
                "txtInfo", tr("info_label"), tr("select_both"), 3, True
            )

            _cands.clear()
            start_anim_timer()

        except:
            _ui.messageBox(tr("failed", details=traceback.format_exc()))


class MyInputChangedHandler(core.InputChangedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args: core.InputChangedEventArgs):
        try:
            # 行クリックではイベントが出ない為、行のボタンで選択する
            m = re.match(r"cand_\d+_(\d+)_\d+$", args.input.id)
            if m:
                tbl: core.TableCommandInput = args.inputs.itemById("tblCand")
                tbl.selectedRow = int(m.group(1))
                return

            if args.input.id == "ddMode":
                save_settings(args.inputs)

            if args.input.id == "chkAnim":
                save_settings(args.inputs)
                if not args.input.value:
                    stop_joint_anim()
                return

            if args.input.id not in ("selA", "selB"):
                return

            inputs: core.CommandInputs = args.inputs
            selA: core.SelectionCommandInput = inputs.itemById("selA")
            selB: core.SelectionCommandInput = inputs.itemById("selB")
            if (
                args.input.id == "selA"
                and selA.selectionCount == 1
                and selB.selectionCount == 0
            ):
                selB.hasFocus = True

            update_candidates(inputs)

        except:
            _ui.messageBox(tr("failed", details=traceback.format_exc()))


class MyValidateInputsHandler(core.ValidateInputsEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args: core.ValidateInputsEventArgs):
        args.areInputsValid = get_selected_candidate(args.inputs) is not None


class MyExecutePreviewHandler(core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args: core.CommandEventArgs):
        global _curMat, _animAborted
        try:
            inputs = args.command.commandInputs
            cand = get_selected_candidate(inputs)
            if not cand:
                return

            occB = get_selected_occurrence(inputs, "selB")
            chkAnim: core.BoolValueCommandInput = inputs.itemById("chkAnim")

            # 切替アニメーション中に呼ばれた場合は中断させる
            if _animating:
                _animAborted = True

            if chkAnim.value:
                # タイマーから繰り返し呼ばれ、経過時間に応じた位置へ動かす
                start_joint_anim(cand)
                mat = get_anim_matrix()
                set_world_matrix(occB, mat)
                _curMat = mat
                return

            stop_joint_anim()
            if _animating:
                set_world_matrix(occB, cand.mat)
                _curMat = cand.mat.copy()
                return

            # プレビュー前に元の位置へ戻されている為、前回の位置から動かす
            fromMat = _curMat if _curMat else get_world_matrix(occB)
            if animate_occurrence(occB, fromMat, cand.mat):
                _curMat = cand.mat.copy()

        except:
            _ui.messageBox(tr("failed", details=traceback.format_exc()))


class MyExecuteHandler(core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args: core.CommandEventArgs):
        core.Application.get().log(args.firingEvent.name)
        try:
            stop_joint_anim()
            inputs = args.command.commandInputs
            cand = get_selected_candidate(inputs)
            if not cand:
                return

            occA = get_selected_occurrence(inputs, "selA")
            occB = get_selected_occurrence(inputs, "selB")
            set_world_matrix(occB, cand.mat)

            des: fusion.Design = fusion.Design.cast(_app.activeProduct)
            if des.designType != fusion.DesignTypes.ParametricDesignType:
                return

            if des.snapshots.hasPendingSnapshot:
                des.snapshots.add()

            # 移動のみモードは位置のキャプチャまでで終了
            ddMode: core.DropDownCommandInput = inputs.itemById("ddMode")
            if ddMode.selectedItem.index == 1:
                return

            geo = create_joint_geometry(cand)

            root: fusion.Component = des.rootComponent

            def add_joint():
                jointIpt: fusion.AsBuiltJointInput = root.asBuiltJoints.createInput(
                    occA, occB, geo
                )
                set_joint_motion(jointIpt, cand, geo is not None)
                root.asBuiltJoints.add(jointIpt)

            try:
                add_joint()
            except RuntimeError:
                # 既存のジョイントと重複する場合は置き換えるか確認する
                joints = get_joints_between(occA, occB)
                if not joints:
                    raise
                names = "\n".join("- " + j.name for j in joints)
                res = _ui.messageBox(
                    tr("joint_exists_confirm", names=names),
                    "FitMe",
                    core.MessageBoxButtonTypes.YesNoButtonType,
                    core.MessageBoxIconTypes.QuestionIconType,
                )
                if res != core.DialogResults.DialogYes:
                    args.executeFailed = True
                    args.executeFailedMessage = tr("joint_replace_cancelled")
                    return
                for j in joints:
                    j.deleteMe()
                add_joint()

        except:
            _ui.messageBox(tr("failed", details=traceback.format_exc()))


class MyCommandDestroyHandler(core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args: core.CommandEventArgs):
        core.Application.get().log(args.firingEvent.name)
        try:
            stop_anim_timer()
        except:
            _app.log(tr("cleanup_failed", details=traceback.format_exc()))
        _handlers.clear()


class MyAnimTickHandler(core.CustomEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args: core.CustomEventArgs):
        global _tickBusy
        try:
            anim_tick()
        except:
            _app.log(tr("anim_failed", details=traceback.format_exc()))
        finally:
            _tickBusy = False


class AnimThread(threading.Thread):
    # 一定間隔でカスタムイベントを発火する(Fusionの処理はメインスレッドのイベントで行う)
    def __init__(self, stopFlag: threading.Event):
        super().__init__(daemon=True)
        self.stopFlag = stopFlag

    def run(self):
        global _tickBusy
        while not self.stopFlag.wait(ANIM_INTERVAL):
            # 前回のイベントの処理が終わっていない場合は発火しない
            if _animTarget and not _tickBusy:
                _tickBusy = True
                _app.fireCustomEvent(ANIM_EVENT_ID, "")


# ---------- UI 補助 ----------


def get_selected_occurrence(inputs: core.CommandInputs, id: str) -> fusion.Occurrence:
    sel: core.SelectionCommandInput = inputs.itemById(id)
    if sel.selectionCount < 1:
        return None
    return fusion.Occurrence.cast(sel.selection(0).entity)


def get_joints_between(occA: fusion.Occurrence, occB: fusion.Occurrence) -> list:
    # A-B間を直接つなぐジョイント (Joint / AsBuiltJoint)
    pair = {occA.fullPathName, occB.fullPathName}
    result = []
    for joints in (occB.joints, occB.asBuiltJoints):
        for j in joints:
            o1, o2 = j.occurrenceOne, j.occurrenceTwo
            if not o1 or not o2:
                continue
            if {o1.fullPathName, o2.fullPathName} == pair:
                result.append(j)
    return result


def load_settings() -> dict:
    # ファイルが無い/壊れている場合は空(既定値を使う)
    try:
        with SETTINGS_PATH.open(encoding="utf-8") as f:
            data = json.load(f)
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def save_settings(inputs: core.CommandInputs):
    ddMode: core.DropDownCommandInput = inputs.itemById("ddMode")
    chkAnim: core.BoolValueCommandInput = inputs.itemById("chkAnim")
    if not ddMode or not chkAnim or not ddMode.selectedItem:
        return
    data = {"mode": ddMode.selectedItem.index, "anim": chkAnim.value}
    try:
        with SETTINGS_PATH.open("w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
    except Exception:
        core.Application.get().log(traceback.format_exc())


def get_selected_candidate(inputs: core.CommandInputs) -> Candidate:
    tbl: core.TableCommandInput = inputs.itemById("tblCand")
    if not tbl:
        return None
    idx = tbl.selectedRow
    if idx < 0 or idx >= len(_cands):
        return None
    return _cands[idx]


def add_table_row(tbl: core.TableCommandInput, row: int, texts):
    # 行を作り直す度に別IDにする
    for col, txt in enumerate(texts):
        id = "cand_{}_{}_{}".format(_tableGen, row, col)
        # 全列を行選択用のボタンにする(行のどこをクリックしても選択できるように)
        ipt = tbl.commandInputs.addBoolValueInput(id, "", False, "", False)
        ipt.text = txt
        tbl.addCommandInput(ipt, row, col)


def update_candidates(inputs: core.CommandInputs):
    global _tableGen, _curMat, _origMat, _animPivot
    tbl: core.TableCommandInput = inputs.itemById("tblCand")
    info: core.TextBoxCommandInput = inputs.itemById("txtInfo")
    tbl.clear()
    _tableGen += 1
    _cands.clear()
    stop_joint_anim()
    _curMat = None
    _origMat = None

    occA = get_selected_occurrence(inputs, "selA")
    occB = get_selected_occurrence(inputs, "selB")
    if not occA or not occB:
        info.text = tr("select_both")
        return
    if occA.entityToken == occB.entityToken:
        info.text = tr("same_component")
        return
    if occB.isGrounded:
        info.text = tr("moving_grounded")
        return

    _threadCache.clear()
    featsA = get_features(occA)

    # 移動側はOccurrence内の座標で保持する
    worldB = get_world_matrix(occB)
    featsB = get_features(occB)
    invB = worldB.copy()
    invB.invert()
    featsB.transformBy(invB)

    # 干渉チェック用の一時ボディ(移動側はOccurrence内の座標)
    tmpA = get_temp_bodies(occA)
    tmpB = get_temp_bodies(occB, invB)

    cands, excluded = search_candidates(featsA, featsB, worldB, tmpA, tmpB)
    _cands.extend(cands)

    _origMat = worldB.copy()
    _animPivot = get_local_center(occB)

    for idx, c in enumerate(_cands):
        add_table_row(
            tbl,
            idx,
            (str(idx + 1), str(c.score), tr(JOINT_NAMES[c.jointType]), c.label),
        )
    if _cands:
        tbl.selectedRow = 0

    info.text = tr(
        "info_summary",
        fixed=feat_summary(featsA),
        moving=feat_summary(featsB),
        count=len(_cands),
        excluded=excluded,
    )


def feat_summary(pf: PartFeats) -> str:
    return tr(
        "feat_summary",
        hole=count_kind(pf.holes, "hole"),
        slot=count_kind(pf.holes, "hole", True),
        shaft=count_kind(pf.holes, "shaft"),
        cone=len([f for f in pf.holes if f.shape == "cone"]),
        sphere=len(pf.spheres),
    )


def count_kind(feats, kind, isSlot=False) -> int:
    return len(
        [f for f in feats if f.kind == kind and f.isSlot == isSlot and f.shape == "cyl"]
    )


# ---------- 位置 ----------


def get_native(obj):
    native = obj.nativeObject
    return native if native else obj


def get_world_matrix(occ: fusion.Occurrence) -> core.Matrix3D:
    mat = core.Matrix3D.create()
    o = occ
    while o:
        mat.transformBy(get_native(o).transform2)
        o = o.assemblyContext
    return mat


def set_world_matrix(occ: fusion.Occurrence, mat: core.Matrix3D):
    parent = occ.assemblyContext
    if parent:
        inv = get_world_matrix(parent)
        inv.invert()
        local = mat.copy()
        local.transformBy(inv)
        mat = local
    get_native(occ).transform2 = mat


def animate_occurrence(
    occ: fusion.Occurrence, fromMat: core.Matrix3D, toMat: core.Matrix3D
) -> bool:
    # プレビュー切替のアニメーション 中断された場合はFalse
    global _animating, _animAborted
    _animating = True
    _animAborted = False
    try:
        pivot = get_local_center(occ)
        vp: core.Viewport = _app.activeViewport
        start = time.perf_counter()
        for i in range(1, ANIM_FRAMES):
            t = 0.5 - 0.5 * math.cos(math.pi * i / ANIM_FRAMES)
            set_world_matrix(occ, interpolate_matrix(fromMat, toMat, t, pivot))
            vp.refresh()
            adsk.doEvents()
            if _animAborted:
                return False
            wait = start + ANIM_TIME * i / ANIM_FRAMES - time.perf_counter()
            if wait > 0:
                time.sleep(wait)
        set_world_matrix(occ, toMat)
        return True
    finally:
        _animating = False


# ---------- ジョイント動作アニメーション ----------


def start_anim_timer():
    global _customEvent, _stopFlag, _tickBusy
    try:
        _app.unregisterCustomEvent(ANIM_EVENT_ID)
    except:
        pass
    _customEvent = _app.registerCustomEvent(ANIM_EVENT_ID)
    onTick = MyAnimTickHandler()
    _customEvent.add(onTick)
    _handlers.append(onTick)

    # 前回のコマンド終了時に未処理のイベントが破棄されると True のまま残る為、リセットする
    _tickBusy = False
    _stopFlag = threading.Event()
    AnimThread(_stopFlag).start()


def stop_anim_timer():
    global _customEvent, _cmd
    if _stopFlag:
        _stopFlag.set()
    stop_joint_anim()
    _cmd = None
    if _customEvent:
        _app.unregisterCustomEvent(ANIM_EVENT_ID)
        _customEvent = None


def cleanup():
    # アドイン停止時の後始末
    stop_anim_timer()
    _handlers.clear()


def start_joint_anim(cand: Candidate):
    # 候補が変わった場合は現在の位置から移動を始める
    global _animTarget, _animStart, _animFrom
    if cand is _animTarget:
        return
    _animTarget = cand
    _animFrom = _curMat if _curMat else _origMat
    _animStart = time.perf_counter()


def stop_joint_anim():
    global _animTarget
    _animTarget = None


def get_anim_matrix() -> core.Matrix3D:
    # 経過時間に応じた移動側のワールド行列
    cand = _animTarget
    elapsed = time.perf_counter() - _animStart
    if elapsed < ANIM_TIME:
        t = 0.5 - 0.5 * math.cos(math.pi * elapsed / ANIM_TIME)
        return interpolate_matrix(_animFrom, cand.mat, t, _animPivot)
    if has_motion(cand):
        return get_motion_matrix(cand, (elapsed - ANIM_TIME) / ANIM_MOTION_TIME)
    return cand.mat.copy()


def anim_tick():
    # タイマーから呼ばれ、プレビューを再実行させる
    cand = _animTarget
    if not cand or not _cmd or not _cmd.isValid:
        return
    if (
        not has_motion(cand)
        and time.perf_counter() - _animStart >= ANIM_TIME
        and _curMat
        and mat_key(_curMat) == mat_key(cand.mat)
    ):
        return  # 動きの無いジョイントは移動完了後は更新しない
    _cmd.doExecutePreview()


def has_motion(cand: Candidate) -> bool:
    if not cand.motionOrigin:
        return False
    if cand.jointType in ("slider", "pinslot") and not cand.motionSlide:
        return False
    return cand.jointType in ("revolute", "cylindrical", "slider", "pinslot", "ball")


def get_motion_matrix(cand: Candidate, t: float) -> core.Matrix3D:
    # 候補位置からジョイントの自由度に沿って動かした行列 (t は周期1, t=0 で候補位置)
    ang = math.pi * 2 * t
    jt = cand.jointType
    mat = cand.mat.copy()

    def rotate(angle, axis, origin):
        rot = core.Matrix3D.create()
        rot.setToRotation(angle, axis, origin)
        mat.transformBy(rot)

    def slide(vec, length):
        v = vec.copy()
        v.scaleBy(length)
        trans = core.Matrix3D.create()
        trans.translation = v
        mat.transformBy(trans)

    if jt == "ball":
        p1 = get_perpendicular(cand.motionAxis)
        p2 = cand.motionAxis.crossProduct(p1)
        rotate(BALL_ANG * math.sin(ang), p1, cand.motionOrigin)
        rotate(BALL_ANG * math.sin(ang * 2), p2, cand.motionOrigin)
        return mat

    dist = cand.motionLen * math.sin(ang)
    if jt == "revolute":
        rotate(ang, cand.motionAxis, cand.motionOrigin)
    elif jt == "cylindrical":
        rotate(ang, cand.motionAxis, cand.motionOrigin)
        slide(cand.motionAxis, dist)
    elif jt == "slider":
        slide(cand.motionSlide, dist)
    elif jt == "pinslot":
        # ピンが長穴から外れないよう、長穴側が動く場合は移動してから固定ピン周りに回転する
        if cand.slotMoving:
            slide(cand.motionSlide, dist)
            rotate(ang, cand.motionAxis, cand.motionOrigin)
        else:
            rotate(ang, cand.motionAxis, cand.motionOrigin)
            slide(cand.motionSlide, dist)
    return mat


def get_local_center(occ: fusion.Occurrence) -> core.Point3D:
    # Occurrence内の座標でのボディ中心
    bbox = get_native(occ).component.boundingBox
    if not bbox:
        return core.Point3D.create(0, 0, 0)
    p0 = bbox.minPoint
    p1 = bbox.maxPoint
    return core.Point3D.create(
        (p0.x + p1.x) * 0.5, (p0.y + p1.y) * 0.5, (p0.z + p1.z) * 0.5
    )


def interpolate_matrix(
    m0: core.Matrix3D, m1: core.Matrix3D, t: float, pivot: core.Point3D
) -> core.Matrix3D:
    # 回転はpivot周りに補間し、pivotの位置は直線で補間する
    c0 = pivot.copy()
    c0.transformBy(m0)
    c1 = pivot.copy()
    c1.transformBy(m1)

    angle, axis = get_relative_rotation(m0, m1)

    mat = m0.copy()
    if angle > TOL_ANG:
        rot = core.Matrix3D.create()
        rot.setToRotation(angle * t, axis, c0)
        mat.transformBy(rot)

    trans = core.Matrix3D.create()
    trans.translation = core.Vector3D.create(
        (c1.x - c0.x) * t, (c1.y - c0.y) * t, (c1.z - c0.z) * t
    )
    mat.transformBy(trans)
    return mat


def get_relative_rotation(m0: core.Matrix3D, m1: core.Matrix3D):
    # R = R1 * R0^T の回転角と回転軸
    _, x0, y0, z0 = m0.getAsCoordinateSystem()
    _, x1, y1, z1 = m1.getAsCoordinateSystem()
    r0 = [[v.x, v.y, v.z] for v in (x0, y0, z0)]
    r1 = [[v.x, v.y, v.z] for v in (x1, y1, z1)]
    r = [
        [sum(r1[k][i] * r0[k][j] for k in range(3)) for j in range(3)] for i in range(3)
    ]

    cos = max(-1.0, min(1.0, (r[0][0] + r[1][1] + r[2][2] - 1) * 0.5))
    angle = math.acos(cos)
    if angle < TOL_ANG:
        return 0.0, core.Vector3D.create(0, 0, 1)

    sin = math.sin(angle)
    if sin > 1e-6:
        axis = core.Vector3D.create(
            r[2][1] - r[1][2], r[0][2] - r[2][0], r[1][0] - r[0][1]
        )
    else:
        # 180度回転の場合は R + I の最も長い列を軸とする
        cols = [
            core.Vector3D.create(
                r[0][j] + (1 if j == 0 else 0),
                r[1][j] + (1 if j == 1 else 0),
                r[2][j] + (1 if j == 2 else 0),
            )
            for j in range(3)
        ]
        axis = max(cols, key=lambda v: v.length)
    axis.normalize()
    return angle, axis


# ---------- 穴/軸の抽出 ----------


def get_bodies(occ: fusion.Occurrence) -> list:
    bodies = [b for b in occ.bRepBodies if b.isSolid]
    for child in occ.childOccurrences:
        bodies.extend(get_bodies(child))
    return bodies


def get_temp_bodies(occ: fusion.Occurrence, mat: core.Matrix3D = None) -> list:
    tmpMgr = fusion.TemporaryBRepManager.get()
    bodies = []
    for body in get_bodies(occ):
        tmp = tmpMgr.copy(get_native(body))
        world = get_world_matrix(body.assemblyContext)
        if mat:
            world.transformBy(mat)
        tmpMgr.transform(tmp, world)
        bodies.append(tmp)
    return bodies


def get_features(occ: fusion.Occurrence) -> PartFeats:
    pf = PartFeats()
    feats = pf.holes
    for bodyIdx, body in enumerate(get_bodies(occ)):
        threadMap = get_thread_map(body)
        outerCache = {}
        arcFaces = []  # 長穴候補の部分円筒面
        for face in body.faces:
            geo = face.geometry
            if geo.surfaceType == core.SurfaceTypes.ConeSurfaceType:
                cone = get_cone_feature(face, core.Cone.cast(geo))
                if cone:
                    feats.append(cone)
                continue
            if geo.surfaceType == core.SurfaceTypes.SphereSurfaceType:
                pf.spheres.append(get_sphere_feature(face, core.Sphere.cast(geo)))
                continue
            if geo.surfaceType != core.SurfaceTypes.CylinderSurfaceType:
                continue
            cyl: core.Cylinder = core.Cylinder.cast(geo)
            axis = cyl.axis.copy()
            axis.normalize()
            kind = "hole" if is_concave(face, cyl.origin, axis) else "shaft"

            seats = get_seats(face, body, kind, axis, outerCache)
            if not seats:
                if kind == "hole":
                    arcFaces.append((face, cyl, axis))
                continue

            dia = cyl.radius * 20
            feat = find_feature(feats, kind, dia, cyl.origin, axis)
            if not feat:
                feat = HoleFeat(kind, dia, cyl.origin.copy(), axis)
                feats.append(feat)
            feat.seats.extend(seats)

            token = get_native(face).entityToken
            if token in threadMap:
                feat.thread = threadMap[token]

        feats.extend(get_slots(arcFaces, body, outerCache))

    for f in feats:
        f.roles = get_roles(f)
    return pf


def get_cone_feature(face: fusion.BRepFace, cone: core.Cone) -> HoleFeat:
    circleEdges = [
        e
        for e in face.edges
        if e.geometry.curveType == core.Curve3DTypes.Circle3DCurveType
    ]
    if not circleEdges:
        return None
    circles = [core.Circle3D.cast(e.geometry) for e in circleEdges]
    radii = [c.radius for c in circles]

    # 軸方向位置と半径の組から傾きを求める(軸の向きや半角の符号に依存しない為)
    axis = cone.axis.copy()
    axis.normalize()
    samples = [
        (cone.origin.vectorTo(c.center).dotProduct(axis), c.radius) for c in circles
    ]
    radial, t = split_vector(cone.origin.vectorTo(face.pointOnFace), axis)
    samples.append((t, radial.length))
    t0, r0 = min(samples, key=lambda s: s[0])
    t1, r1 = max(samples, key=lambda s: s[0])
    if t1 - t0 < TOL:
        return None
    slope = (r1 - r0) / (t1 - t0)
    if abs(slope) < 1e-6:
        return None

    # 頂点と開口方向(半径が増える方向)
    v = axis.copy()
    v.scaleBy(t0 - r0 / slope)
    apex = cone.origin.copy()
    apex.translateBy(v)
    openDir = axis.copy()
    if slope < 0:
        openDir.scaleBy(-1)

    kind = "hole" if is_concave(face, apex, openDir) else "shaft"
    feat = HoleFeat(kind, max(radii) * 20, apex.copy(), openDir.copy())
    feat.shape = "cone"
    feat.halfAngle = math.atan(abs(slope))
    feat.rRange = (min(radii) if len(radii) > 1 else 0.0, max(radii))

    # 座面は頂点 (穴:開口方向 / 軸:先端方向)
    normal = openDir.copy()
    if kind == "shaft":
        normal.scaleBy(-1)
    feat.seats.append(Seat(apex.copy(), normal, circleEdges[0], True, False))
    return feat


def get_sphere_feature(face: fusion.BRepFace, sphere: core.Sphere) -> SphereFeat:
    pnt = face.pointOnFace
    _, normal = face.evaluator.getNormalAtPoint(pnt)
    kind = "hole" if sphere.origin.vectorTo(pnt).dotProduct(normal) < 0 else "shaft"
    return SphereFeat(kind, sphere.origin.copy(), sphere.radius, face)


def is_concave(
    face: fusion.BRepFace, origin: core.Point3D, axis: core.Vector3D
) -> bool:
    pnt = face.pointOnFace
    _, normal = face.evaluator.getNormalAtPoint(pnt)
    radial, _ = split_vector(origin.vectorTo(pnt), axis)
    return radial.dotProduct(normal) < 0


def get_seats(
    face: fusion.BRepFace, body: fusion.BRepBody, kind, axis, outerCache
) -> list:
    seats = []
    pnt = face.pointOnFace
    for edge in face.edges:
        geo = edge.geometry
        if geo.curveType != core.Curve3DTypes.Circle3DCurveType:
            continue
        circle: core.Circle3D = core.Circle3D.cast(geo)
        for f2 in edge.faces:
            if f2.tempId == face.tempId:
                continue
            if f2.geometry.surfaceType != core.SurfaceTypes.PlaneSurfaceType:
                continue
            normal = get_face_normal(f2)
            if not is_parallel(normal, axis):
                continue

            side = circle.center.vectorTo(pnt).dotProduct(normal)
            if kind == "hole":
                if side > 0:
                    continue  # 穴底
                isProper = True
            else:
                isProper = side > 0  # 段付き軸の肩面

            seats.append(
                Seat(
                    circle.center.copy(),
                    normal,
                    edge,
                    isProper,
                    is_outer(body, circle.center, normal, outerCache),
                )
            )
    return seats


def get_slots(arcFaces, body: fusion.BRepBody, outerCache) -> list:
    # 2つの半円筒面が側壁の平面で繋がっているものを長穴とする
    slots = []
    used = set()
    for (f1, c1, axis), (f2, c2, _) in itertools.combinations(arcFaces, 2):
        if f1.tempId in used or f2.tempId in used:
            continue
        if abs(c1.radius - c2.radius) > 1e-5 or not is_parallel(axis, c2.axis):
            continue
        radial, _ = split_vector(c1.origin.vectorTo(c2.origin), axis)
        if radial.length < TOL:
            continue

        # 側壁(軸に平行な平面)を共有している
        walls1 = get_adjacent_planes(f1, axis, False)
        walls2 = get_adjacent_planes(f2, axis, False)
        shared = set(walls1) & set(walls2)
        if not shared:
            continue

        # 2面が互いに外側を向いている
        if c1.origin.vectorTo(f1.pointOnFace).dotProduct(radial) > 0:
            continue
        if c2.origin.vectorTo(f2.pointOnFace).dotProduct(radial) < 0:
            continue

        half = radial.copy()
        half.scaleBy(0.5)
        origin = c1.origin.copy()
        origin.translateBy(half)

        seats = get_slot_seats(f1, f2, body, axis, outerCache)
        if not seats:
            continue

        feat = HoleFeat("hole", c1.radius * 20, origin, axis.copy())
        feat.isSlot = True
        feat.slotDir = radial.copy()
        feat.slotDir.normalize()
        feat.slotEdge = get_line_edge(walls1[next(iter(shared))], feat.slotDir)
        feat.slotLen = radial.length
        feat.seats.extend(seats)
        slots.append(feat)
        used.add(f1.tempId)
        used.add(f2.tempId)
    return slots


def get_adjacent_planes(
    face: fusion.BRepFace, axis: core.Vector3D, isPerp: bool
) -> dict:
    # 隣接する平面 {tempId: 面}  isPerp=True:軸に垂直な平面 False:軸に平行な平面
    planes = {}
    for edge in face.edges:
        for f2 in edge.faces:
            if f2.tempId == face.tempId or f2.tempId in planes:
                continue
            if f2.geometry.surfaceType != core.SurfaceTypes.PlaneSurfaceType:
                continue
            normal = get_face_normal(f2)
            if isPerp:
                if is_parallel(normal, axis):
                    planes[f2.tempId] = f2
            elif abs(normal.dotProduct(axis)) < TOL_ANG:
                planes[f2.tempId] = f2
    return planes


def get_line_edge(face: fusion.BRepFace, direction: core.Vector3D) -> fusion.BRepEdge:
    # direction に平行な直線エッジ
    for edge in face.edges:
        geo = edge.geometry
        if geo.curveType != core.Curve3DTypes.Line3DCurveType:
            continue
        line: core.Line3D = core.Line3D.cast(geo)
        vec = line.startPoint.vectorTo(line.endPoint)
        vec.normalize()
        if is_parallel(vec, direction):
            return edge
    return None


def get_arc_centers(face: fusion.BRepFace) -> dict:
    # 円弧エッジの中心 {隣接面のtempId: 中心}
    centers = {}
    for edge in face.edges:
        geo = edge.geometry
        if geo.curveType != core.Curve3DTypes.Arc3DCurveType:
            continue
        arc: core.Arc3D = core.Arc3D.cast(geo)
        for f2 in edge.faces:
            if f2.tempId != face.tempId:
                centers[f2.tempId] = arc.center
    return centers


def get_slot_seats(
    f1: fusion.BRepFace,
    f2: fusion.BRepFace,
    body: fusion.BRepBody,
    axis: core.Vector3D,
    outerCache,
) -> list:
    seats = []
    pnt = f1.pointOnFace
    centers1 = get_arc_centers(f1)
    centers2 = get_arc_centers(f2)
    for tempId, plane in get_adjacent_planes(f1, axis, True).items():
        if tempId not in centers1 or tempId not in centers2:
            continue
        p1 = centers1[tempId]
        p2 = centers2[tempId]
        normal = get_face_normal(plane)
        if p1.vectorTo(pnt).dotProduct(normal) > 0:
            continue  # 穴底

        center = core.Point3D.create(
            (p1.x + p2.x) * 0.5, (p1.y + p2.y) * 0.5, (p1.z + p2.z) * 0.5
        )
        # 円弧の中心は長穴中心にならない為、Joint用のエッジは持たない
        seats.append(
            Seat(
                center,
                normal,
                None,
                True,
                is_outer(body, center, normal, outerCache),
            )
        )
    return seats


def get_face_normal(face: fusion.BRepFace) -> core.Vector3D:
    _, normal = face.evaluator.getNormalAtPoint(face.pointOnFace)
    normal.normalize()
    return normal


def is_outer(
    body: fusion.BRepBody, pnt: core.Point3D, normal: core.Vector3D, cache
) -> bool:
    key = (round(normal.x, 4), round(normal.y, 4), round(normal.z, 4))
    if key not in cache:
        obb: core.OrientedBoundingBox3D = _app.measureManager.getOrientedBoundingBox(
            body, normal, get_perpendicular(normal)
        )
        cache[key] = obb.centerPoint.asVector().dotProduct(normal) + obb.length * 0.5
    return abs(pnt.asVector().dotProduct(normal) - cache[key]) < TOL


def find_feature(feats, kind, dia, origin, axis) -> HoleFeat:
    for f in feats:
        if f.kind != kind or abs(f.dia - dia) > 1e-3:
            continue
        if not is_parallel(f.axis, axis):
            continue
        if point_line_distance(origin, f.origin, f.axis) < TOL:
            return f
    return None


def get_thread_map(body: fusion.BRepBody) -> dict:
    comp: fusion.Component = body.parentComponent
    if comp.id in _threadCache:
        return _threadCache[comp.id]

    threadMap = {}
    for tf in comp.features.threadFeatures:
        try:
            size = parse_size(tf.threadInfo.threadDesignation)
            if not size:
                continue
            for f in get_feature_faces(tf, ("faces", "inputCylindricalFaces")):
                threadMap[f.entityToken] = size
        except:
            pass

    for hf in comp.features.holeFeatures:
        try:
            # APIのバージョンにより threadInfo が無い場合あり
            info = getattr(hf, "threadInfo", None)
            if not info:
                continue
            size = parse_size(info.threadDesignation)
            if not size:
                continue
            for f in get_feature_faces(hf, ("sideFaces",)):
                threadMap[f.entityToken] = size
        except:
            pass

    _threadCache[comp.id] = threadMap
    return threadMap


def get_feature_faces(feat, names) -> list:
    faces = []
    for name in names:
        try:
            objs = getattr(feat, name, None)
            if objs:
                faces.extend([f for f in objs if fusion.BRepFace.cast(f)])
        except:
            pass
    return faces


def parse_size(txt: str) -> str:
    m = re.search(r"M\s*(\d+(?:\.\d+)?)", txt or "")
    if not m:
        return None
    return "M{:g}".format(float(m.group(1)))


def get_roles(feat: HoleFeat) -> set:
    roles = set()
    if feat.shape != "cyl":
        return roles
    if feat.thread:
        roles.add(("tap" if feat.kind == "hole" else "ext", feat.thread))
        return roles

    d = feat.dia
    for size, nominal, tapDrill, minor, clearances in METRIC_TABLE:
        if feat.kind == "hole":
            if not feat.isSlot and (is_near_dia(d, tapDrill) or is_near_dia(d, minor)):
                roles.add(("tap", size))
            if any(is_near_dia(d, c) for c in clearances):
                roles.add(("clr", size))
        else:
            if is_near_dia(d, nominal):
                roles.add(("bolt", size))
    return roles


# ---------- 候補 ----------


def kind_name(f: HoleFeat) -> str:
    if f.isSlot:
        return tr("kind_slot")
    return tr("kind_shaft") if f.kind == "shaft" else tr("kind_hole")


def role_name(role: str, f: HoleFeat) -> str:
    return tr("kind_slot") if f.isSlot else tr(ROLE_NAMES[role])


def get_compat(fa: HoleFeat, fb: HoleFeat):
    if fa.shape == "cone" or fb.shape == "cone":
        return get_cone_compat(fa, fb)
    if fa.isSlot and fb.isSlot:
        return None  # 長穴同士は対象外

    best = None
    for ra, sa in fa.roles:
        for rb, sb in fb.roles:
            if sa != sb:
                continue
            score = ROLE_PAIR_SCORE.get(frozenset((ra, rb)))
            if score and (not best or score > best[0]):
                best = (
                    score,
                    "{} {}×{}".format(sa, role_name(ra, fa), role_name(rb, fb)),
                )

    if is_near_dia(fa.dia, fb.dia):
        diaTxt = "φ{:g}".format(round(fa.dia, 3))
        txt = "{}×{}".format(kind_name(fa), kind_name(fb))
        if fa.kind != fb.kind:
            score = SCORE_SHAFT_HOLE
        elif fa.kind == "hole" and not (fa.thread and fb.thread):
            score = SCORE_HOLE_HOLE
        else:
            score = None
        if score and (not best or score > best[0]):
            best = (score, "{} {}".format(diaTxt, txt))

    return best


def get_cone_compat(fa: HoleFeat, fb: HoleFeat):
    # 円錐同士で凹×凸、半角が同じで接触する半径範囲が重なるもの
    if fa.shape != fb.shape or fa.kind == fb.kind:
        return None
    if abs(fa.halfAngle - fb.halfAngle) > TOL_CONE_ANG:
        return None
    overlap = min(fa.rRange[1], fb.rRange[1]) - max(fa.rRange[0], fb.rRange[0])
    if overlap < TOL:
        return None
    return (
        SCORE_CONE,
        tr(
            "label_cone",
            angle=round(math.degrees(fa.halfAngle * 2), 1),
            a=kind_name(fa),
            b=kind_name(fb),
        ),
    )


def search_candidates(pa: PartFeats, pb: PartFeats, worldB: core.Matrix3D, tmpA, tmpB):
    featsA, featsB = pa.holes, pb.holes
    cands = gen_hole_candidates(featsA, featsB, worldB)
    cands.extend(limit_cands(gen_sphere_candidates(pa.spheres, pb.spheres, worldB)))

    # スコア順にしてから同じ位置の候補を統合する
    cands.sort(key=lambda c: (-c.score, -len(c.matched)))
    keySet = set()
    uniq = []
    for c in cands:
        key = mat_key(c.mat)
        if key in keySet:
            continue
        keySet.add(key)
        uniq.append(c)

    # 干渉するものは除外する(上位から順に判定)
    result = []
    excluded = 0
    for c in uniq:
        if len(result) >= MAX_CANDIDATES:
            break
        if is_interfering(c.mat, tmpA, tmpB, featsA, featsB, c.matched):
            excluded += 1
            continue
        if c.jointType is None:
            c.jointType, c.slotEdge = get_joint_type(c, featsA, featsB, tmpA, tmpB)
        set_motion(c, featsA, featsB)
        result.append(c)
    return result, excluded


def set_motion(c: Candidate, featsA, featsB):
    # ジョイント動作アニメーション用の回転軸/スライド方向 (長穴を含むペアを優先)
    if not c.matched:
        return
    i, k = c.matched[0]
    for m in c.matched:
        if featsA[m[0]].isSlot or featsB[m[1]].isSlot:
            i, k = m
            break
    fa, fb = featsA[i], featsB[k]
    c.motionOrigin = fa.origin.copy()
    c.motionAxis = fa.axis.copy()
    c.motionLen = fa.dia / 10
    if fa.isSlot:
        c.motionSlide = fa.slotDir.copy()
        c.motionLen = fa.slotLen * 0.5
    elif fb.isSlot:
        c.motionSlide = fb.slotDir.copy()
        c.motionSlide.transformBy(c.mat)
        c.motionLen = fb.slotLen * 0.5
        c.slotMoving = True


def mat_key(mat: core.Matrix3D) -> tuple:
    return tuple(round(v, 3) for v in mat.asArray())


def limit_cands(cands) -> list:
    cands.sort(key=lambda c: -c.score)
    return cands[:MAX_PER_TYPE]


def gen_sphere_candidates(spheresA, spheresB, worldB: core.Matrix3D) -> list:
    # 球×球面座: 姿勢は現在のまま、中心を一致させる
    cands = []
    for sa in spheresA:
        for sb in spheresB:
            if sa.kind == sb.kind or not is_near_dia(sa.radius * 20, sb.radius * 20):
                continue
            center = sb.center.copy()
            center.transformBy(worldB)
            trans = core.Matrix3D.create()
            trans.translation = center.vectorTo(sa.center)
            mat = worldB.copy()
            mat.transformBy(trans)

            # 回転の自由度が残る為、1穴一致と同様に半分にする
            score = (SCORE_SPHERE + SCORE_MATCH_BONUS) // 2
            label = "SR{:g} {}×{}".format(
                round(sa.radius * 10, 3),
                tr("kind_socket") if sa.kind == "hole" else tr("kind_ball"),
                tr("kind_socket") if sb.kind == "hole" else tr("kind_ball"),
            )
            c = Candidate(
                mat,
                [],
                score,
                label,
                None,
                True,
                None,
                jointType="ball",
                geoKind="nonplanar",
                geoEntity=sa.face,
            )
            c.motionOrigin = sa.center.copy()
            c.motionAxis = core.Vector3D.create(0, 0, 1)
            cands.append(c)
    return cands


def gen_hole_candidates(featsA, featsB, worldB: core.Matrix3D) -> list:
    compat = {}
    for i, fa in enumerate(featsA):
        for k, fb in enumerate(featsB):
            c = get_compat(fa, fb)
            if c:
                compat[(i, k)] = c
    if not compat:
        return []

    seatsA = [(i, s) for i, f in enumerate(featsA) for s in f.seats]
    seatsB = [(k, s) for k, f in enumerate(featsB) for s in f.seats]

    mats = []  # (mat, 固定側座面, 移動側座面)

    # 2穴パターン
    pairsB = []
    for (k, sk), (k2, sk2) in itertools.permutations(seatsB, 2):
        if k == k2 or not is_same_dir(sk.normal, sk2.normal):
            continue
        radial, axial = split_vector(sk.center.vectorTo(sk2.center), sk.normal)
        if radial.length < TOL:
            continue
        pairsB.append((radial.length, axial, k, sk, k2, radial))
    pairsB.sort(key=lambda p: p[0])
    keys = [p[0] for p in pairsB]

    for (i, si), (j, sj) in itertools.combinations(seatsA, 2):
        if i == j or not is_same_dir(si.normal, sj.normal):
            continue
        radialA, axialA = split_vector(si.center.vectorTo(sj.center), si.normal)
        dist = radialA.length
        if dist < TOL:
            continue
        lo = bisect.bisect_left(keys, dist - TOL)
        hi = bisect.bisect_right(keys, dist + TOL)
        for _, axialB, k, sk, k2, radialB in pairsB[lo:hi]:
            if (i, k) not in compat or (j, k2) not in compat:
                continue
            flush = is_flush(si, sk)
            if not flush:
                axialB = -axialB
            if abs(axialA - axialB) > TOL:
                continue
            mat = get_align_matrix(
                sk.center,
                radialB,
                get_seat_dir(sk, flush),
                si.center,
                radialA,
                si.normal,
            )
            mats.append((mat, si, sk))

    # 1穴パターン(回転方向は現在の向きに近いものにする)
    for i, si in seatsA:
        for k, sk in seatsB:
            if (i, k) not in compat:
                continue
            fromZ = get_seat_dir(sk, is_flush(si, sk))
            fromX = get_perpendicular(fromZ)
            toX = fromX.copy()
            toX.transformBy(worldB)
            toX, _ = split_vector(toX, si.normal)
            if toX.length < 1e-3:
                toX = get_perpendicular(si.normal)
            mat = get_align_matrix(
                sk.center,
                fromX,
                fromZ,
                si.center,
                toX,
                si.normal,
            )
            mats.append((mat, si, sk))

    # 評価・重複統合
    cands = []
    keySet = set()
    for mat, si, sk in mats:
        key = tuple(round(v, 3) for v in mat.asArray())
        if key in keySet:
            continue
        keySet.add(key)

        matched = get_matched(mat, featsA, featsB, compat)
        if not matched:
            continue

        isFree = len(matched) < 2
        score = sum(compat[m][0] for m in matched) + SCORE_MATCH_BONUS * len(matched)
        if si.isOuter:
            score += SCORE_OUTER_SEAT
        if sk.isOuter:
            score += SCORE_OUTER_SEAT
        if not (si.isProper and sk.isProper):
            score += SCORE_IMPROPER_SEAT
        if isFree:
            score //= 2

        best = max(matched, key=lambda m: compat[m][0])
        label = tr("label_matched", base=compat[best][1], count=len(matched))

        cands.append(
            Candidate(mat, matched, score, label, si.edge or sk.edge, isFree, si.normal)
        )
    return cands


def is_flush(si: Seat, sk: Seat) -> bool:
    # 軸の端面(肩面でない)が含まれる場合は、端面を相手の面と面一にして穴へ挿入する
    # それ以外(穴×穴、段付き軸の肩面×穴)は座面同士を接触させる
    return not (si.isProper and sk.isProper)


def get_seat_dir(sk: Seat, flush: bool) -> core.Vector3D:
    # 固定側座面の法線に合わせる移動側の方向
    return sk.normal.copy() if flush else reverse_vector(sk.normal)


def get_joint_type(c: Candidate, featsA, featsB, tmpA, tmpB):
    # 一致した穴(穴×穴)と長穴(穴×長穴)の数からジョイントタイプを決める
    holeCount = 0
    hasCone = False  # 円錐は軸方向も拘束される
    slots = []  # (ワールドでの長穴方向, 長穴エッジ)
    for i, k in c.matched:
        fa, fb = featsA[i], featsB[k]
        if fa.shape == "cone":
            hasCone = True
        if fa.isSlot:
            slots.append((fa.slotDir, fa.slotEdge))
        elif fb.isSlot:
            vec = fb.slotDir.copy()
            vec.transformBy(c.mat)
            slots.append((vec, fb.slotEdge))
        else:
            holeCount += 1

    if holeCount >= 2 or (holeCount == 1 and slots):
        return "rigid", None
    if holeCount == 1:
        if hasCone or has_plane_contact(
            c.mat, c.normal, tmpA, tmpB, featsA, featsB, c.matched
        ):
            return "revolute", None
        return "cylindrical", None
    if len(slots) == 1:
        return "pinslot", slots[0][1]
    if all(is_parallel(slots[0][0], vec) for vec, _ in slots[1:]):
        return "slider", slots[0][1]
    return "rigid", None


def has_plane_contact(
    mat: core.Matrix3D, normal: core.Vector3D, tmpA, tmpB, featsA, featsB, matched
) -> bool:
    # 座面の法線方向へ少し押し込んで干渉すれば平面で接触している
    vec = reverse_vector(normal)
    vec.scaleBy(TOL_CONTACT)
    trans = core.Matrix3D.create()
    trans.translation = vec
    shifted = mat.copy()
    shifted.transformBy(trans)
    return is_interfering(shifted, tmpA, tmpB, featsA, featsB, matched)


def set_joint_motion(jointIpt: fusion.AsBuiltJointInput, cand: Candidate, hasGeo: bool):
    # Jointの基準(円エッジ中心)のZ軸が穴軸になる
    jt = cand.jointType if hasGeo else "rigid"
    slotEdge = cand.slotEdge if cand.slotEdge and cand.slotEdge.isValid else None
    dirZ = fusion.JointDirections.ZAxisJointDirection
    dirCustom = fusion.JointDirections.CustomJointDirection
    if jt == "revolute":
        jointIpt.setAsRevoluteJointMotion(dirZ)
    elif jt == "cylindrical":
        jointIpt.setAsCylindricalJointMotion(dirZ)
    elif jt == "pinslot" and slotEdge:
        jointIpt.setAsPinSlotJointMotion(dirZ, dirCustom, None, slotEdge)
    elif jt == "slider" and slotEdge:
        jointIpt.setAsSliderJointMotion(dirCustom, slotEdge)
    elif jt == "ball":
        jointIpt.setAsBallJointMotion(dirZ, fusion.JointDirections.XAxisJointDirection)
    elif jt == "planar":
        jointIpt.setAsPlanarJointMotion(dirZ)
    else:
        jointIpt.setAsRigidJointMotion()


def create_joint_geometry(cand: Candidate) -> fusion.JointGeometry:
    ent = cand.geoEntity
    if not ent or not ent.isValid:
        return None
    keyPnt = fusion.JointKeyPointTypes.CenterKeyPoint
    try:
        if cand.geoKind == "planar":
            return fusion.JointGeometry.createByPlanarFace(ent, None, keyPnt)
        if cand.geoKind == "nonplanar":
            return fusion.JointGeometry.createByNonPlanarFace(ent, keyPnt)
        return fusion.JointGeometry.createByCurve(ent, keyPnt)
    except:
        _app.log(tr("joint_geometry_failed", details=traceback.format_exc()))
        return None


def is_interfering(mat: core.Matrix3D, tmpA, tmpB, featsA, featsB, matched) -> bool:
    tmpMgr = fusion.TemporaryBRepManager.get()
    for bodyB in tmpB:
        b = tmpMgr.copy(bodyB)
        tmpMgr.transform(b, mat)
        for bodyA in tmpA:
            if not bodyA.boundingBox.intersects(b.boundingBox):
                continue
            try:
                body = tmpMgr.copy(bodyA)
                tmpMgr.booleanOperation(
                    body, tmpMgr.copy(b), fusion.BooleanTypes.IntersectionBooleanType
                )
                if body.faces.count < 1 or body.volume < TOL_VOL:
                    continue

                # 軸×穴(おねじ×タップ等)の重なりは干渉としない
                for i, k in matched:
                    fa, fb = featsA[i], featsB[k]
                    if fa.kind == fb.kind or fa.shape != "cyl":
                        continue
                    shaft = fa if fa.kind == "shaft" else fb
                    cyl = create_axis_cylinder(
                        body.boundingBox, fa.origin, fa.axis, shaft.dia / 20 + TOL
                    )
                    tmpMgr.booleanOperation(
                        body, cyl, fusion.BooleanTypes.DifferenceBooleanType
                    )
                    if body.faces.count < 1:
                        break

                if body.faces.count > 0 and body.volume > TOL_VOL:
                    return True
            except:
                _app.log(tr("interference_failed", details=traceback.format_exc()))
    return False


def create_axis_cylinder(
    bbox: core.BoundingBox3D, origin: core.Point3D, axis: core.Vector3D, radius: float
) -> fusion.BRepBody:
    # bbox を貫く長さの円柱を軸線上に作る
    length = bbox.minPoint.distanceTo(bbox.maxPoint) + 1
    center = core.Point3D.create(
        (bbox.minPoint.x + bbox.maxPoint.x) * 0.5,
        (bbox.minPoint.y + bbox.maxPoint.y) * 0.5,
        (bbox.minPoint.z + bbox.maxPoint.z) * 0.5,
    )
    t = origin.vectorTo(center).dotProduct(axis)
    pnts = []
    for s in (t - length, t + length):
        v = axis.copy()
        v.scaleBy(s)
        p = origin.copy()
        p.translateBy(v)
        pnts.append(p)
    return fusion.TemporaryBRepManager.get().createCylinderOrCone(
        pnts[0], radius, pnts[1], radius
    )


def get_matched(mat: core.Matrix3D, featsA, featsB, compat) -> list:
    matched = []
    used = set()
    for k, fb in enumerate(featsB):
        origin = fb.origin.copy()
        origin.transformBy(mat)
        axis = fb.axis.copy()
        axis.transformBy(mat)
        for i, fa in enumerate(featsA):
            if i in used or (i, k) not in compat:
                continue
            if not is_parallel(fa.axis, axis):
                continue
            if fa.shape == "cone":
                # 円錐は向きと頂点も一致させる
                if not is_same_dir(fa.axis, axis) or origin.distanceTo(fa.origin) > TOL:
                    continue
            if point_line_distance(origin, fa.origin, fa.axis) < TOL:
                matched.append((i, k))
                used.add(i)
                break
    return matched


def get_align_matrix(fromO, fromX, fromZ, toO, toX, toZ) -> core.Matrix3D:
    fx, fz, tx, tz = [v.copy() for v in (fromX, fromZ, toX, toZ)]
    for v in (fx, fz, tx, tz):
        v.normalize()
    fy = fz.crossProduct(fx)
    ty = tz.crossProduct(tx)
    mat = core.Matrix3D.create()
    mat.setToAlignCoordinateSystems(fromO, fx, fy, fz, toO, tx, ty, tz)
    return mat


# ---------- 幾何補助 ----------


def is_near_dia(d1, d2) -> bool:
    return abs(d1 - d2) <= TOL_DIA


def is_parallel(v1: core.Vector3D, v2: core.Vector3D) -> bool:
    return abs(v1.dotProduct(v2)) > 1 - TOL_ANG


def is_same_dir(v1: core.Vector3D, v2: core.Vector3D) -> bool:
    return v1.dotProduct(v2) > 1 - TOL_ANG


def reverse_vector(v: core.Vector3D) -> core.Vector3D:
    r = v.copy()
    r.scaleBy(-1)
    return r


def get_perpendicular(v: core.Vector3D) -> core.Vector3D:
    if abs(v.x) < 0.9:
        ref = core.Vector3D.create(1, 0, 0)
    else:
        ref = core.Vector3D.create(0, 1, 0)
    p = v.crossProduct(ref)
    p.normalize()
    return p


def split_vector(v: core.Vector3D, n: core.Vector3D):
    # v を n方向成分(スカラー)と n に垂直な成分(ベクトル)に分ける
    axial = v.dotProduct(n)
    a = n.copy()
    a.scaleBy(axial)
    radial = v.copy()
    radial.subtract(a)
    return radial, axial


def point_line_distance(
    pnt: core.Point3D, origin: core.Point3D, axis: core.Vector3D
) -> float:
    radial, _ = split_vector(origin.vectorTo(pnt), axis)
    return radial.length
