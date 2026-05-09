# -*- coding: utf-8 -*-
import argparse
import json
import os
import subprocess
import sys
from pathlib import Path


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
        data_dir = item.get("DATA_DIR")

        missing = [
            key
            for key, value in (
                ("NET_PATH", net_path),
                ("BG_MODEL_FILENAME", bg_model_filename),
                ("DATA_DIR", data_dir),
            )
            if not value
        ]
        if missing:
            raise ValueError(
                f"第 {index} 项缺少必填字段: {', '.join(missing)}"
            )

        normalized.append(
            {
                "name": item.get("name", f"scenario_{index:03d}"),
                "NET_PATH": net_path,
                "BG_MODEL_FILENAME": bg_model_filename,
                "DATA_DIR": data_dir,
                "EGO_MODEL_FILENAME": item.get("EGO_MODEL_FILENAME"),
                "FILTER_SCENES": item.get("FILTER_SCENES"),
                "USE_TEST_LOGIC": item.get("USE_TEST_LOGIC"),
                "REPEAT_SINGLE_SCENARIO": item.get("REPEAT_SINGLE_SCENARIO"),
                "TRAIN_MODE": item.get("TRAIN_MODE"),
            }
        )

    return normalized


def apply_batch_env(env, config):
    env["TESSNG_NET_PATH"] = str(config["NET_PATH"])
    env["TESSNG_BG_MODEL_FILENAME"] = str(config["BG_MODEL_FILENAME"])
    env["TESSNG_DATA_DIR"] = str(config["DATA_DIR"])
    env["TESSNG_EXIT_ON_SIMULATION_STOP"] = "1"

    if config.get("EGO_MODEL_FILENAME"):
        env["TESSNG_EGO_MODEL_FILENAME"] = str(config["EGO_MODEL_FILENAME"])
    if config.get("FILTER_SCENES") is not None:
        env["TESSNG_FILTER_SCENES"] = json.dumps(
            config["FILTER_SCENES"], ensure_ascii=False
        )
    if config.get("USE_TEST_LOGIC") is not None:
        env["TESSNG_USE_TEST_LOGIC"] = str(config["USE_TEST_LOGIC"]).lower()
    if config.get("REPEAT_SINGLE_SCENARIO") is not None:
        env["TESSNG_REPEAT_SINGLE_SCENARIO"] = str(
            config["REPEAT_SINGLE_SCENARIO"]
        ).lower()
    if config.get("TRAIN_MODE") is not None:
        env["TESSNG_TRAIN_MODE"] = str(config["TRAIN_MODE"]).lower()


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
        print(f"[批量]   NET_PATH={config['NET_PATH']}")
        print(f"[批量]   BG_MODEL_FILENAME={config['BG_MODEL_FILENAME']}")
        print(f"[批量]   DATA_DIR={config['DATA_DIR']}")

        env = os.environ.copy()
        apply_batch_env(env, config)

        command = [sys.executable, str(WORKSPACE / "main.py"), "--single-run"]
        result = subprocess.run(command, cwd=str(WORKSPACE), env=env)

        if result.returncode != 0:
            print(
                f"[批量] 任务失败: {config['name']}, returncode={result.returncode}"
            )
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
    config = {
        "__workspace": os.fspath(WORKSPACE),
        "__netfilepath": NET_PATH,
        "__simuafterload": True,
        "__custsimubysteps": False,
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
