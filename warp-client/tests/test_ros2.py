from artefacts.ros2 import generate_parameter_output
import yaml


def test_generate_parameter_outptut(tmp_path):
    params = {"turtle/speed": 5}
    file_path = tmp_path / "params.yaml"
    generate_parameter_output(params, file_path)
    with open(file_path) as f:
        ros2_params = yaml.load(f, Loader=yaml.Loader)
    assert ros2_params == {"turtle": {"ros__parameters": {"speed": 5}}}
