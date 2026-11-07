# Fusion 360 API Python add-in
"""2つのコンポーネントの穴/軸から組付け候補を探し、ジョイントを作成する。"""

from __future__ import annotations

import traceback
from pathlib import Path

import adsk.core as core

from . import fitme_command
from .i18n import tr

# Fusion に登録するコマンドの一意な ID。
_COMMAND_ID = "kantoku_fitme"
# コマンドボタンを配置するパネル ID の候補 (先に見つかったものを使う)。
_PANEL_IDS = ("AssemblePanel", "AssembleJointsPanel", "SolidScriptsAddinsPanel")
# Fusion のイベントに登録したハンドラを保持し、GC による解除を防ぐ。
_handlers = []


def _report_error() -> None:
    """現在の例外を Fusion のメッセージボックスへ表示する。"""
    core.Application.get().userInterface.messageBox(
        tr("failed", details=traceback.format_exc())
    )


def _find_panel(ui: core.UserInterface) -> core.ToolbarPanel | None:
    """ボタンを配置するパネルを取得する。"""
    for panel_id in _PANEL_IDS:
        panel = ui.allToolbarPanels.itemById(panel_id)
        if panel is not None:
            return panel
    return None


def _remove_command(ui: core.UserInterface) -> None:
    """登録済みのボタンとコマンド定義を削除する。"""
    for panel_id in _PANEL_IDS:
        panel = ui.allToolbarPanels.itemById(panel_id)
        if panel is None:
            continue
        control = panel.controls.itemById(_COMMAND_ID)
        if control is not None:
            control.deleteMe()
    command_def = ui.commandDefinitions.itemById(_COMMAND_ID)
    if command_def is not None:
        command_def.deleteMe()


def run(context: dict) -> None:
    """FitMe コマンドを Fusion のツールバーへ登録する。"""
    try:
        ui = core.Application.get().userInterface
        if _handlers:
            return
        panel = _find_panel(ui)
        if panel is None:
            raise RuntimeError(tr("panel_missing"))
        _remove_command(ui)
        command_def = ui.commandDefinitions.addButtonDefinition(
            _COMMAND_ID,
            tr("button_name"),
            tr("button_description"),
            str(Path(__file__).resolve().parent / "resources"),
        )
        created = fitme_command.MyCommandCreatedHandler()
        command_def.commandCreated.add(created)
        _handlers.append(created)
        control = panel.controls.addCommand(command_def)
        control.isPromotedByDefault = True
        control.isPromoted = True
    except Exception:
        _report_error()
        stop(context)


def stop(context: dict) -> None:
    """登録したコマンドとアニメーション用のイベントを解放する。"""
    try:
        fitme_command.cleanup()
        _remove_command(core.Application.get().userInterface)
    except Exception:
        _report_error()
    finally:
        _handlers.clear()
