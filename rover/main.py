"""Entrypoint for AI Recon Rover mission control server."""
from __future__ import annotations

import argparse
import uvicorn

from rover.config import load_calibration, load_config
from rover.detector import SyntheticPersonDetector, YoloDetector
from rover.frames import MjpegFrameSource, SyntheticFrameSource
from rover.mission import MissionController
from rover.models import EvidenceMode
from rover.robot import Esp32Robot
from rover.server import create_app
from rover.store import MissionStore
from tools.mock_esp32 import MockEsp32Server


def main() -> None:
    parser = argparse.ArgumentParser(description="AI Recon Rover Mission Server v5.5")
    parser.add_argument("--mode", choices=["real", "mock"], default="real")
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--calibration", default="calibration.json")
    args = parser.parse_args()

    cfg = load_config(args.config)
    cal = load_calibration(args.calibration)
    store = MissionStore(runs_root=cfg.server.runs_dir)

    mock_srv = None
    if args.mode == "real":
        robot = Esp32Robot(cfg.robot.devkit_url, cfg.robot, cal)
        frame_source = MjpegFrameSource(cfg.camera)
        detector = YoloDetector(cfg.vision)
        ev_mode = EvidenceMode.REAL_HARDWARE
    else:
        mock_srv = MockEsp32Server(host="127.0.0.1", port=8081, speed_scale=20.0)
        mock_srv.start()
        robot = Esp32Robot(mock_srv.base_url, cfg.robot, cal)
        frame_source = SyntheticFrameSource()
        detector = SyntheticPersonDetector()
        ev_mode = EvidenceMode.MOCK

    frame_source.start()
    mission = MissionController(cfg, cal, robot, frame_source, detector, store, evidence_mode=ev_mode)
    app = create_app(mission, config_path=args.config, calibration_path=args.calibration)

    try:
        uvicorn.run(app, host=cfg.server.host, port=cfg.server.port, log_level="info")
    finally:
        frame_source.stop()
        if mock_srv is not None:
            mock_srv.stop()


if __name__ == "__main__":
    main()
