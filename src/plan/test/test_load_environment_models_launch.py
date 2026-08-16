import importlib.util
from pathlib import Path


def load_module():
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "load_environment_models.launch.py"
    spec = importlib.util.spec_from_file_location("load_environment_models_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_load_environment_models_launch_uses_real_robot_description():
    module = load_module()

    source = Path(module.__file__).read_text()
    assert "MoveItConfigsBuilder" in source
    assert "moveit_config.robot_description" in source
    assert '"<robot name=\\"empty\\" />"' not in source
