# -*- coding: utf-8 -*-
import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from Utils.ScenarioLoader import g_scenario_loader

is_linux = sys.platform.startswith("linux")
if is_linux:
    from TessngLib.Tessng import *


WORKSPACE = Path(__file__).resolve().parent
DLL_DIR = WORKSPACE / "TessngLib"
TESSNG_PLUGIN_DIR = WORKSPACE / "TessngPlugin"


def prepare_runtime():
    os.environ["PATH"] = str(DLL_DIR) + os.pathsep + os.environ["PATH"]
    os.environ["PATH"] = str(TESSNG_PLUGIN_DIR) + os.pathsep + os.environ["PATH"]
    sys.path.insert(0, str(DLL_DIR))
    sys.path.insert(0, str(TESSNG_PLUGIN_DIR))


def build_parser():
    parser = argparse.ArgumentParser(description="TESSNG 仿真入口")
    parser.add_argument(
        "--batch-config",
        help="批量仿真配置 JSON 文件路径。传入后会逐条启动子进程执行仿真。",
    )
    parser.add_argument(
        "--single-run",
        action="store_true",
        help="内部参数。批量模式会用它启动单次仿真子进程。",
    )
    return parser


def _gen_batch_item(
    item,
    name,
    data_dir,
    bg_model_filename,
    bg_model_full_path,
):
    batch_item = {
        "name": name,
        "BG_MODEL_FILENAME": bg_model_filename,
        "DATA_DIR": data_dir,
        "USE_TEST_LOGIC": item.get("USE_TEST_LOGIC", False),
        "EGO_MODEL_FILENAME": item.get("EGO_MODEL_FILENAME"),
        "FILTER_SCENES": item.get("FILTER_SCENES"),
        "REPEAT_SINGLE_SCENARIO": item.get("REPEAT_SINGLE_SCENARIO"),
        "TRAIN_MODE": item.get("TRAIN_MODE"),
        "TRAIN_TOTAL_TIMESTEPS": item.get("TRAIN_TOTAL_TIMESTEPS"),
        "TRAJ_OUTPUT": item.get("TRAJ_OUTPUT"),
    }

    if bg_model_filename:
        batch_item["BG_MODEL_FILENAME"] = bg_model_filename
    if bg_model_full_path:
        batch_item["BG_MODEL_FULL_PATH"] = bg_model_full_path

    return batch_item

def load_batch_configs(config_path):
    with open(config_path, "r", encoding="utf-8") as file:
        configs = json.load(file)

    if not isinstance(configs, list):
        raise ValueError("批量配置文件必须是 JSON 数组")

    normalized = []
    for index, item in enumerate(configs):
        if not isinstance(item, dict):
            raise ValueError(f"第 {index} 项不是对象")

        net_path = item.get("NET_PATH")
        bg_model_filename = item.get("BG_MODEL_FILENAME")
        bg_model_fullpath = item.get("BG_MODEL_FULL_PATH")
        data_dir = item.get("DATA_DIR")

        missing = [
            key
            for key, value in (
                # ("NET_PATH", net_path),
                ("DATA_DIR", data_dir),
                (
                    "Only one: BG_MODEL_FILENAME or BG_MODEL_FULLPATH",
                    (bg_model_filename or bg_model_fullpath),
                ),
            )
            if not value
        ]
        if missing:
            raise ValueError(f"第 {index} 项缺少必填字段: {', '.join(missing)}")

        is_use_test_logic = item.get("USE_TEST_LOGIC", False)
        if is_use_test_logic:
            name = item.get("name", f"scenario_{index:03d}")
            batch_item = _gen_batch_item(
                item,
                name=name,
                data_dir=data_dir,
                bg_model_filename=bg_model_filename,
                bg_model_full_path=bg_model_fullpath,
            )
            normalized.append(batch_item)
        else:
            sub_idx = 0
            for file_item in os.listdir(data_dir):
                name = item.get("name", f"scenario_{index:03d}_{sub_idx:03d}")
                batch_item = _gen_batch_item(
                    item,
                    name=name,
                    data_dir=os.path.join(data_dir, file_item),
                    bg_model_filename=bg_model_filename,
                    bg_model_full_path=bg_model_fullpath,
                )
                normalized.append(batch_item)
                sub_idx += 1

    return normalized


def apply_batch_env(env, config):
    env["TESSNG_DATA_DIR"] = str(config["DATA_DIR"])
    env["TESSNG_EXIT_ON_SIMULATION_STOP"] = "1"

    def foreach_env_key(env_keys, is_lower=False):
        for k in env_keys:
            env_v = config.get(k) or None
            if not env_v:
                continue
            if is_lower:
                env[f"TESSNG_{k}"] = str(env_v).lower()
            else:
                env[f"TESSNG_{k}"] = str(env_v)

    env_keys = [
        "BG_MODEL_FULL_PATH",
        "BG_MODEL_FILENAME",
        "NET_PATH",
        "EGO_MODEL_FILENAME",
    ]
    env_keys_lower = [
        "USE_TEST_LOGIC",
        "REPEAT_SINGLE_SCENARIO",
        "TRAIN_MODE",
        "EGO_MODEL_FILENAME",
    ]
    foreach_env_key(env_keys)
    foreach_env_key(env_keys_lower, is_lower=True)

    if config.get("FILTER_SCENES") is not None:
        env["TESSNG_FILTER_SCENES"] = json.dumps(
            config["FILTER_SCENES"], ensure_ascii=False
        )

    if config.get("TRAIN_TOTAL_TIMESTEPS") is not None:
        env["TRAIN_TOTAL_TIMESTEPS"] = str(config["TRAIN_TOTAL_TIMESTEPS"])

    if config.get("TRAJ_OUTPUT") is not None:
        env["TRAJ_OUTPUT"] = str(config["TRAJ_OUTPUT"])
    

def run_batch(config_path):
    configs = load_batch_configs(config_path)
    total = len(configs)

    if total == 0:
        print("[批量] 配置为空，没有任务可执行")
        return 0

    print(f"[批量] 共 {total} 个仿真任务，配置文件: {config_path}")

    for index, config in enumerate(configs, start=1):
        print("=" * 80)
        print(f"[批量] 开始第 {index}/{total} 个任务: {config['name']}")
        # print(f"[批量]   NET_PATH={config['NET_PATH']}")
        print(f"[批量]   BG_MODEL_FILENAME={config['BG_MODEL_FILENAME']}")
        print(f"[批量]   DATA_DIR={config['DATA_DIR']}")

        env = os.environ.copy()
        apply_batch_env(env, config)

        command = [sys.executable, str(WORKSPACE / "main.py"), "--single-run"]
        result = subprocess.run(command, cwd=str(WORKSPACE), env=env)

        if not is_linux and result.returncode != 0:
            print(f"[批量] 任务失败: {config['name']}, returncode={result.returncode}")
            return result.returncode

        print(f"[批量] 任务完成: {config['name']}")

    print("=" * 80)
    print("[批量] 所有仿真任务执行完成")
    return 0


def run_single():
    prepare_runtime()

    from PySide2.QtWidgets import QApplication
    import Tessng
    from TessngPlugin.MyPlugin import MyPlugin, TessngFactory
    from Utils.Constant import NET_PATH

    app = QApplication()

    net_path = g_scenario_loader.getNetPath()
    print(f"[单次] 启动仿真，NET_PATH={net_path}")
    config = {
        "__workspace": os.fspath(WORKSPACE),
        "__netfilepath": net_path,
        "__simuafterload": True,
        "__custsimubysteps": False,
        "__autosave": False,
    }
    plugin = MyPlugin()
    factory = TessngFactory()
    tessng = factory.build(plugin, config)
    if tessng is None:
        print("Tessng 初始化失败")
        return 1

    return app.exec_()


def main():
    parser = build_parser()
    args = parser.parse_args()

    if args.batch_config:
        return run_batch(args.batch_config)

    return run_single()


if __name__ == "__main__":
    sys.exit(main())
